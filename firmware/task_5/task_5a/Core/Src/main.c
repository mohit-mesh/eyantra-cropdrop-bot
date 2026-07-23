/* Improved non-blocking line follower core
 * Filename: main_line_follow.c
 * This file is intended to be compiled in place of main.c or integrated
 * into the project. It assumes CubeMX-generated MX_* init functions exist
 * in other project files (SystemClock_Config, MX_GPIO_Init, MX_ADC1_Init, MX_TIM2_Init, etc.).
 */

#include "main.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

volatile uint32_t adc_buffer[5];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

static float Kp = 110.0f;
static float Kd = 18.0f;
static int BASE_SPEED = 70;
static const int PWM_MAX = 255;
static const int PWM_MIN = -255;

#define NUM_SENSORS 5
static const float sensor_weights[NUM_SENSORS] = {-2.0f,-1.0f,0.0f,1.0f,2.0f};
static float prev_error = 0.0f;
static float filtered_error = 0.0f;
static const float ERROR_EWMA_ALPHA = 0.7f;
static float prev_left_out = 0.0f, prev_right_out = 0.0f;
static volatile int line_present = 1;
static volatile int all_sensors_on_line = 0;
static uint32_t last_loop_ms = 0;

typedef enum { TS_IDLE=0, TS_GTURN, TS_RIGHT90, TS_PULSATE_LEFT } TurnState_e;
static TurnState_e turn_state = TS_IDLE;
static uint32_t turn_start_ms = 0;
static int turn_phase = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

static float compute_line_error(void);
static void motors_set_speed(int16_t left, int16_t right);
static void start_right_turn_90(void);
static void start_g_turn(void);
static void start_pulsate_left(void);
static void process_turn_state(void);

static float clampf(float v, float lo, float hi) { if (v<lo) return lo; if (v>hi) return hi; return v; }

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUM_SENSORS);
  last_loop_ms = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_loop_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;

    process_turn_state();
    float error = compute_line_error();

    if (!line_present) {
      if (turn_state == TS_IDLE) start_pulsate_left();
      HAL_Delay(1);
      last_loop_ms = now; continue;
    }

    if (all_sensors_on_line && turn_state == TS_IDLE) {
      start_right_turn_90(); last_loop_ms = now; continue;
    }

    filtered_error = (ERROR_EWMA_ALPHA * error) + ((1.0f - ERROR_EWMA_ALPHA) * filtered_error);
    float derivative = (filtered_error - prev_error) / dt;
    float correction = Kp * filtered_error + Kd * derivative;
    float err_mag = fabsf(filtered_error);
    float speed_scale = 1.0f;
    if (err_mag > 0.2f) speed_scale = 0.45f;

    float base_f = (float)BASE_SPEED * speed_scale;
    float left_cmd = base_f - correction;
    float right_cmd = base_f + correction;
    left_cmd = clampf(left_cmd, (float)PWM_MIN, (float)PWM_MAX);
    right_cmd = clampf(right_cmd, (float)PWM_MIN, (float)PWM_MAX);

    const float MAX_DELTA = 80.0f;
    float dleft = left_cmd - prev_left_out;
    float dright = right_cmd - prev_right_out;
    if (dleft > MAX_DELTA) dleft = MAX_DELTA;
    if (dleft < -MAX_DELTA) dleft = -MAX_DELTA;
    if (dright > MAX_DELTA) dright = MAX_DELTA;
    if (dright < -MAX_DELTA) dright = -MAX_DELTA;
    float out_left = prev_left_out + dleft;
    float out_right = prev_right_out + dright;
    prev_left_out = out_left; prev_right_out = out_right;

    motors_set_speed((int16_t)out_left, (int16_t)out_right);

    prev_error = filtered_error;
    last_loop_ms = now;
    HAL_Delay(2);
  }
}

static float compute_line_error(void)
{
  float vals[NUM_SENSORS]; float minv=65535.0f, maxv=0.0f;
  for (int i=0;i<NUM_SENSORS;++i) { vals[i] = (float)adc_buffer[i]; if (vals[i]<minv) minv=vals[i]; if (vals[i]>maxv) maxv=vals[i]; }
  if (maxv-minv < 1.0f) maxv = minv + 1.0f;
  float sum_w=0.0f, sum=0.0f; int on_count=0;
  for (int i=0;i<NUM_SENSORS;++i) {
    float norm = (vals[i]-minv)/(maxv-minv); if (norm<0.0f) norm=0.0f; if (norm>1.0f) norm=1.0f;
    float w = norm; if (w>0.5f) on_count++;
    sum += w * sensor_weights[i]; sum_w += w;
  }
  float contrast = maxv - minv; static int lost_cnt=0;
  if (contrast < 80.0f) { lost_cnt++; if (lost_cnt>3) { line_present = 0; all_sensors_on_line = 0; } return prev_error; }
  line_present = 1; lost_cnt = 0; all_sensors_on_line = (on_count>=3)?1:0;
  if (sum_w < 0.01f) return prev_error; float pos = sum / sum_w; return -pos;
}

static void motors_set_speed(int16_t left, int16_t right)
{
  if (left >= 0) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)left); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); }
  else { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(-left)); }
  if (right >= 0) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)right); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); }
  else { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(-right)); }
}

static void start_right_turn_90(void) { if (turn_state!=TS_IDLE) return; turn_state=TS_RIGHT90; turn_start_ms=HAL_GetTick(); turn_phase=0; }
static void start_g_turn(void) { if (turn_state!=TS_IDLE) return; turn_state=TS_GTURN; turn_start_ms=HAL_GetTick(); turn_phase=0; }
static void start_pulsate_left(void) { if (turn_state==TS_PULSATE_LEFT) return; turn_state=TS_PULSATE_LEFT; turn_start_ms=HAL_GetTick(); turn_phase=0; }

static void process_turn_state(void)
{
  if (turn_state==TS_IDLE) return;
  uint32_t now = HAL_GetTick(); uint32_t elapsed = now - turn_start_ms;
  switch (turn_state) {
    case TS_RIGHT90:
      if (turn_phase==0) { motors_set_speed(BASE_SPEED, BASE_SPEED); if (elapsed>350) { turn_phase=1; turn_start_ms=now; } }
      else if (turn_phase==1) { motors_set_speed(80,-80); float center = ((adc_buffer[1]+adc_buffer[2]+adc_buffer[3]) / 3.0f); if (center>2000.0f || (now-turn_start_ms)>2500) { motors_set_speed(0,0); turn_phase=2; turn_start_ms=now; } }
      else { if ((now-turn_start_ms)>250) turn_state=TS_IDLE; }
      break;
    case TS_GTURN:
      if (turn_phase==0) { motors_set_speed(160,-160); if (elapsed>700) { turn_phase=1; turn_start_ms=now; } }
      else if (turn_phase==1) { motors_set_speed(100,-100); float center = ((adc_buffer[1]+adc_buffer[2]+adc_buffer[3]) / 3.0f); if (center>2000.0f || (now-turn_start_ms)>4000) { motors_set_speed(0,0); turn_phase=2; turn_start_ms=now; } }
      else { if ((now-turn_start_ms)>300) turn_state=TS_IDLE; }
      break;
    case TS_PULSATE_LEFT:
      if (turn_phase==0) { motors_set_speed(0,0); if (elapsed>200) { turn_phase=1; turn_start_ms=now; } }
      else if (turn_phase==1) { motors_set_speed(-120,120); float maxv=0; for (int i=0;i<NUM_SENSORS;++i) if (adc_buffer[i]>maxv) maxv=adc_buffer[i]; if (maxv>1800.0f) { motors_set_speed(0,0); turn_state=TS_IDLE; break; } if ((now-turn_start_ms)>120) { turn_phase=0; turn_start_ms=now; } }
      break;
    default: turn_state=TS_IDLE; break;
  }
}
