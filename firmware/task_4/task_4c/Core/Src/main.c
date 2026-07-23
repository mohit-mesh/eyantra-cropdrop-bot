/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */
#include "Color.h"
#include <stdio.h>
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Line follower configuration */
#define NUM_SENSORS 5
volatile uint16_t adc_buf[NUM_SENSORS];

/* sensor weight for position calculation (left -> right) */
const float sensor_weights[NUM_SENSORS] = {-2.02f, -1.0f, 0.0f, 1.0f, 2.0f};

/* PID parameters (tweak on robot) */
/* Increased Kp/Kd for faster corrective response; reduce if oscillation occurs */
static float Kp = 36500.0f;
static float Ki = 0.6f;
static float Kd = 2.0f;

static float integral = 0.0f;
static float prev_error = 0.0f;

/* base pwm (0-255), tune for motors / battery */
static int base_speed = 60;

/* sensor inversion mode: -1 = auto-detect, 0 = no invert, 1 = force invert
  Set to 1 to treat lower ADC values as line (useful if your line is darker).
*/
static int sensor_invert_mode = 0;

/* prototypes */
void motors_set_speed(int16_t left, int16_t right);
float compute_line_error(void);
/* box / pickup state */
typedef enum {BOX_NONE=0, BOX_RED=1, BOX_GREEN=2, BOX_BLUE=3} box_color_t;
static volatile int carrying_box = 0; /* 0 = not carrying, 1 = carrying */
static box_color_t detected_box_color = BOX_NONE;
/* anti-retrigger timing (ms) after pickup or drop) */
static uint32_t last_box_action_tick = 0;
static const uint32_t BOX_ACTION_LOCK_MS = 2000; /* 2s lockout to avoid re-pick/drop */
/* timestamp when we started carrying the current box (ms) */
static uint32_t carrying_since_tick = 0;
/* helper: read TCS3200 and decide color (returns BOX_*). This mirrors logic in Color.c */
static box_color_t detect_box_color_from_tcs(void)
{
  uint32_t r = Color_ReadRed();
  uint32_t g = Color_ReadGreen();
  uint32_t b = Color_ReadBlue();

  uint32_t r_n = r;
  uint32_t g_n = (uint32_t)((float)g * COLOR_G_GAIN);
  uint32_t b_n = (uint32_t)((float)b * COLOR_B_GAIN);

  /* default: none */
  box_color_t res = BOX_NONE;

  if ((r_n > g_n + COLOR_DIFF - 100) && (r_n > b_n + COLOR_DIFF - 95)) {
    res = BOX_RED;
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET); /* ON */
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
  } else if ((g_n > r_n + COLOR_DIFF) && (g_n > b_n + COLOR_DIFF)) {
    res = BOX_GREEN;
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET); /* ON */
    HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
  } else if ((b_n > r_n + COLOR_DIFF) && (b_n > g_n + COLOR_DIFF)) {
    res = BOX_BLUE;
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET); /* ON */
  } else {
    /* mixed/unknown: turn off RGB */
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
    res = BOX_NONE;
  }

  /* send debug over USART so user can tune thresholds if needed */
  char dbuf[128];
  int bl = snprintf(dbuf, sizeof(dbuf), "TCS RAW R=%lu G=%lu B=%lu -> Rn=%lu Gn=%lu Bn=%lu\r\n", r, g, b, r_n, g_n, b_n);
  if (bl > 0) HAL_UART_Transmit(&huart2, (uint8_t*)dbuf, (uint16_t)bl, 50);

  return res;
}

/* helper: debounce box detect sensor
   Returns 1 if sensor reports presence. Many IR reflectance/presence sensors
   are active-low on this board; use active-low sampling (RESET = presence).
*/
static int box_sensor_asserted(void)
{
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    if (HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin) == GPIO_PIN_RESET) count++;
    HAL_Delay(10);
  }
  return (count >= 3) ? 1 : 0;
}

/* smoothing / anti-overshoot helpers */
static float filtered_error = 0.0f;
static float derivative_filtered = 0.0f;
static float center_weight = 0.0f; /* normalized weight of middle sensor (0..1) */
static float prev_left_out = 0.0f;
static float prev_right_out = 0.0f;
/* flag set by compute_line_error() to indicate the line is currently seen */
static volatile int line_present = 1;
/* hysteresis counters for robust detection */
static int no_line_count = 0;
static int line_confirm_count = 0;

/* tuning constants for smoothing and limits */
/* Responsiveness tuning: higher EWMA_ALPHA -> faster reaction to new error
  DERIV_ALPHA closer to 1 retains more of the new derivative (faster react).
  MAX_CORRECTION increased to allow stronger correction power when needed. */
const float ERROR_EWMA_ALPHA = 0.70f;    /* 0..1, higher = faster to react */
const float DERIV_ALPHA = 0.80f;         /* derivative low-pass (higher = faster react) */
const float MAX_CORRECTION = 255.0f;     /* maximum correction applied to motors */
const float SLOWDOWN_ERROR = 1.f;       /* error magnitude that triggers max slowdown */
const float MIN_SPEED_SCALE = 0.03f;     /* minimum fraction of base speed during sharp turns */
const float INTEGRAL_WINDUP_LIMIT = 500.0f;
/* Detection / stability tuning */
const float MIN_WEIGHT_SUM = 0.02f;      /* minimum summed weight to consider the line present */
const float ERROR_DEADBAND = 0.02f;      /* small-error deadband to avoid wobble */
const float MAX_OUTPUT_DELTA = 80.0f;    /* max change in PWM value per cycle to smooth outputs */
/* additional detection tuning */
const float MIN_CONTRAST = 300.0f;       /* min (max-min) ADC difference to consider valid (0..4095) */
/* Amplify stronger sensor readings (white) via gamma >1 to make detection more decisive */
const float SENSOR_RESPONSE_GAMMA = 30;
const int NO_LINE_CONFIRM_COUNT = 3;     /* consecutive frames needed to declare no-line */
const int LINE_CONFIRM_COUNT = 2;        /* consecutive frames needed to declare line present */
/* When deciding to drop a carried box, require the recent positional error
  to be small so that sharp turns (which briefly lose the line) are not
  mistaken for the drop-off area. Tune this if you still see false drops. */
const float DROP_ERROR_THRESHOLD = 3.0f; /* max |error| allowed to consider a true drop (sensor-weight units) */
/* Require the previous motor outputs to indicate forward, straight motion
  before treating a sustained no-line as a drop. This helps avoid slow
  turning (small position error but non-forward motion) being misdetected
  as a drop-off. Tweak these values on the robot if needed. */
const float DROP_WHEEL_DIFF_THRESHOLD = 40.0f; /* max allowed |left-right| PWM difference */
const float DROP_MIN_FORWARD_SUM = 20.0f;     /* min sum(|left|+|right|) indicating the robot was moving */
/* Minimum time after pickup before a drop is allowed unconditionally (ms).
  This prevents immediate false drops and ensures the robot travels to a drop
  area; adjust to your course length and speeds. */
const uint32_t DROP_MIN_TRAVEL_MS = 4500;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* Initialize TCS3200 color sensor (uses TIM3 for input capture) */
  Color_Init(&htim3);
  /* USER CODE BEGIN 2 */

  /* Start PWM channels for motors (TIM2 CH1..CH4) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* Ensure initial duty = 0 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  /* Start ADC in DMA mode to continuously update adc_buf */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, NUM_SENSORS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    /* Simple PID line-following loop, sample time ~10ms */
    const float dt = 0.01f; /* seconds */

    float error = compute_line_error();

    /* --- Box pickup / drop state machine ---
       - If not carrying a box and box-detect sensor triggers, stop and read colour
       - Activate electromagnet to pick box and continue following
       - If carrying and IR array reports sustained no-line (drop zone), stop and release
    */
    if (!carrying_box) {
      /* sensor is wired active-low on this board: presence == GPIO_PIN_RESET
         Use debounce helper to avoid false triggers */
      /* enforce lockout: don't pickup again for BOX_ACTION_LOCK_MS after last action */
      if ((HAL_GetTick() - last_box_action_tick) >= BOX_ACTION_LOCK_MS) {
        /* only attempt pickup while robot is on the line (avoid pickups in the drop/black area) */
        if (line_present && box_sensor_asserted()) {
        /* stop motors to perform detection */
        motors_set_speed(0, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        HAL_Delay(50);

        /* detect colour using timer-based reads and set on-board RGB */
        detected_box_color = detect_box_color_from_tcs();

        /* engage electromagnet to pick up box (assumed active HIGH) */
        HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_SET);
        /* turn on indicator LED (LD2) to show box is held; keep until drop */
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        carrying_box = 1;
        /* record when we started carrying so we can require some travel before drop */
        carrying_since_tick = HAL_GetTick();
        /* start lockout timer to avoid immediate re-trigger */
        last_box_action_tick = HAL_GetTick();
        HAL_Delay(300);
        }
      }
    } else {
      /* carrying a box: check for drop-off zone (no-line sustained) */
        /* Drop condition: only use timing-based check to decide drop.
           This avoids false negatives from wheel/alignment checks on slow turns. */
        if (!line_present && (no_line_count >= NO_LINE_CONFIRM_COUNT) &&
            ((HAL_GetTick() - carrying_since_tick) >= DROP_MIN_TRAVEL_MS)) {
        /* arrived at drop area: stop and release */
        motors_set_speed(0, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        HAL_Delay(50);


        /* release electromagnet and clear indicator LED */
        HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_RESET);
        carrying_box = 0;
        /* start lockout timer so robot doesn't immediately pick up again */
        last_box_action_tick = HAL_GetTick();
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

        /* ensure RGB LED is turned off (common-anode: HIGH = off) */
        HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);

        HAL_Delay(200);
      }
    }

    /* If no line is present, immediately stop the motors and clear state
       so the robot doesn't continue turning when it shouldn't. */
    if (!line_present) {
      integral = 0.0f;
      prev_error = 0.0f;
      filtered_error = 0.0f;
      derivative_filtered = 0.0f;
      prev_left_out = 0.0f;
      prev_right_out = 0.0f;
      /* Force PWM compare to zero on all TIM2 channels used for motors to ensure stop */
      motors_set_speed(0, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      HAL_Delay(0);
      continue;
    }

     /* Smoothed PID + speed scaling to keep middle sensor centered on curves
       - EWMA filter the error to reduce noise
       - Low-pass derivative to avoid spikes
       - Anti-windup on integral
       - Clamp correction magnitude
       - Scale base speed down on large errors to keep the robot from cutting the corner
       - Reduce allowed correction when the center sensor strongly detects the line
     */
     /* EWMA error */
     filtered_error = (ERROR_EWMA_ALPHA * error) + ((1.0f - ERROR_EWMA_ALPHA) * filtered_error);

     /* derivative (raw delta on filtered error) and LPF it */
     float deriv_raw = (filtered_error - prev_error) / dt;
     derivative_filtered = (DERIV_ALPHA * deriv_raw) + ((1.0f - DERIV_ALPHA) * derivative_filtered);

     /* integral with windup guard */
     integral += filtered_error * dt;
     if (integral > INTEGRAL_WINDUP_LIMIT) integral = INTEGRAL_WINDUP_LIMIT;
     if (integral < -INTEGRAL_WINDUP_LIMIT) integral = -INTEGRAL_WINDUP_LIMIT;

     /* compute correction using existing Kp/Ki/Kd (tuned for PWM scale) */
     float correction = (Kp * filtered_error) + (Ki * integral) + (Kd * derivative_filtered);

     /* reduce max correction when center sensor strongly sees the line (avoid crossing) */
     float center_influence = 1.0f - center_weight; /* center 1.0 -> allow less correction */
     float max_corr_allowed = MAX_CORRECTION * (0.3f + 0.2f * center_influence); /* between 0.3..1.0 factor */
     if (correction > max_corr_allowed) correction = max_corr_allowed;
     if (correction < -max_corr_allowed) correction = -max_corr_allowed;

     /* speed scaling: reduce base speed proportionally to absolute filtered error */
     float err_mag = fabsf(filtered_error);
     float speed_scale = 0.8f;
     if (err_mag >= SLOWDOWN_ERROR) speed_scale = MIN_SPEED_SCALE;
     else speed_scale = 1.0f - ((err_mag / SLOWDOWN_ERROR) * (1.0f - MIN_SPEED_SCALE));

     float base_f = (float)base_speed * speed_scale;
     float left_f = base_f - correction;
     float right_f = base_f + correction;

    /* If this is a sharp turn, set the inner (slower) wheel to zero so
       the robot can pivot cleanly instead of trying to reverse the inner
       wheel. Trigger when correction is near the allowed maximum OR when
       the absolute filtered error is large. */
    const float SHARP_TURN_RATIO = 0.8f; /* fraction of allowed correction */

    /* small-error deadband to avoid wobble around center */
    if (fabsf(filtered_error) < ERROR_DEADBAND) {
      correction = 0.0f;
      /* reduce integral to avoid slow recovery oscillation */
      integral = -1.0f;
    }

    if ((fabsf(correction) >= (SHARP_TURN_RATIO * max_corr_allowed)) || (err_mag >= SLOWDOWN_ERROR)) {
      if (correction > 0.0f) {
        /* turning left => left wheel is inner, stop it */
        left_f = 0.0f;
      } else if (correction < 0.0f) {
        /* turning right => right wheel is inner, stop it */
        right_f = 0.0f;
      }
    }

     /* update prev_error used for derivative (store filtered) */
     prev_error = filtered_error;

     /* clamp to -255..255 */
     if (left_f > 255.0f) left_f = 255.0f;
     if (left_f < -255.0f) left_f = -255.0f;
     if (right_f > 255.0f) right_f = 255.0f;
     if (right_f < -255.0f) right_f = -255.0f;

     /* Smooth motor outputs: limit how fast we change PWM to reduce wobble
       and help the robot start moving consistently. */
     float desired_left = left_f;
     float desired_right = right_f;
     float delta_left = desired_left - prev_left_out;
     float delta_right = desired_right - prev_right_out;
     if (delta_left > MAX_OUTPUT_DELTA) delta_left = MAX_OUTPUT_DELTA;
     if (delta_left < -MAX_OUTPUT_DELTA) delta_left = -MAX_OUTPUT_DELTA;
     if (delta_right > MAX_OUTPUT_DELTA) delta_right = MAX_OUTPUT_DELTA;
     if (delta_right < -MAX_OUTPUT_DELTA) delta_right = -MAX_OUTPUT_DELTA;
     float out_left = prev_left_out + delta_left;
     float out_right = prev_right_out + delta_right;
     prev_left_out = out_left;
     prev_right_out = out_right;

  /* Helper functions were previously (incorrectly) defined inside the main loop.
     They are implemented at file scope below so they have proper linkage. */


    motors_set_speed((int16_t)out_left, (int16_t)out_right);

    HAL_Delay(0);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/* File-scope helper: compute line error from adc_buf values
   Normalizes readings and computes weighted average position.
   Returns position error in sensor-weight units (negative = left, positive = right).
*/
float compute_line_error(void)
{
  float vals[NUM_SENSORS];
  float maxv = 1.0f;
  float minv = 65535.0f;
  float sum_vals = 0.0f;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    vals[i] = (float)adc_buf[i];
    if (vals[i] > maxv) maxv = vals[i];
    if (vals[i] < minv) minv = vals[i];
    sum_vals += vals[i];
  }
  /* avoid divide by zero */
  if (maxv < 1.0f) maxv = 1.0f;

  /* Decide whether the line is darker or lighter than the background.
     Many reflectance sensors return larger ADC values on white and smaller
     on black; in that case we must invert the normalized values so the
     line contributes larger weight. We detect this at runtime using the
     min/max/average readings. */
  float avg = sum_vals / (float)NUM_SENSORS;
  float mid = (maxv + minv) * 0.5f;
  int auto_invert = (avg > mid) ? 1 : 0; /* 1 => background brighter than line */
  int invert;
  if (sensor_invert_mode == -1) {
    invert = auto_invert; /* auto-detect */
  } else {
    invert = (sensor_invert_mode == 1) ? 1 : 0; /* forced by user */
  }

  float sum = 0.0f;
  float weight_sum = 0.0f;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    float norm = vals[i] / maxv; /* 0..1 */
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    /* raw weight (0..1) depending on invert; then apply gamma to emphasize strong readings */
    float w_raw = invert ? (1.0f - norm) : norm;
    float w = powf(w_raw, SENSOR_RESPONSE_GAMMA);
    sum += w * sensor_weights[i];
    weight_sum += w;
    if (i == 2) {
      /* record center sensor normalized weight (0..1) for turn smoothing */
      center_weight = w;
    }
  }
  /* compute simple contrast (max - min) to ensure sensors see a real change */
  float contrast = maxv - minv;

  if ((weight_sum <= MIN_WEIGHT_SUM) || (contrast < MIN_CONTRAST)) {
    /* possible no-line: increment counter and only declare absent after several frames */
    no_line_count++;
    line_confirm_count = 0;
    if (no_line_count >= NO_LINE_CONFIRM_COUNT) {
      line_present = 0;
      return 0.0f;
    }
    /* still tentative; return previous error to keep behavior until confirmed */
    return prev_error > 0 ? 3.0f : -3.0f;
  }
  /* valid line detected: reset no_line_count, require a couple frames to confirm presence */
  no_line_count = 0;
  line_confirm_count++;
  if (line_confirm_count < LINE_CONFIRM_COUNT) {
    /* wait for stable detection */
    return prev_error > 0 ? 3.0f : -3.0f;
  }
  line_present = 1;
  float pos = sum / weight_sum;
  /* error is desired position (0) minus measured position */
  return 0.0f - pos;
}

/* File-scope helper: set motor speeds using TIM2 channels and direction pins
   Adjust direction GPIO names in `main.h` if your board uses different pins.
*/
void motors_set_speed(int16_t left, int16_t right)
{
#ifdef MOTOR_A_DIR1_Pin
  if (left >= 0) {
    HAL_GPIO_WritePin(MOTOR_A_DIR1_GPIO_Port, MOTOR_A_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_DIR2_GPIO_Port, MOTOR_A_DIR2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);
  } else {
    HAL_GPIO_WritePin(MOTOR_A_DIR1_GPIO_Port, MOTOR_A_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_DIR2_GPIO_Port, MOTOR_A_DIR2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -left);
  }
#else
  if (left >= 0) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);
  else __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -left);
#endif

#ifdef MOTOR_B_DIR1_Pin
  if (right >= 0) {
    HAL_GPIO_WritePin(MOTOR_B_DIR1_GPIO_Port, MOTOR_B_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_DIR2_GPIO_Port, MOTOR_B_DIR2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right);
  } else {
    HAL_GPIO_WritePin(MOTOR_B_DIR1_GPIO_Port, MOTOR_B_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_DIR2_GPIO_Port, MOTOR_B_DIR2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -right);
  }
#else
  if (right >= 0) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right);
  else __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -right);
#endif
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Electromagnet_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Electromagnet_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin GREEN_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Box_detect_Pin */
  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
