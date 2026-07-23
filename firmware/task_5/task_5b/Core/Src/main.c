/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Refactored Reliable Line Follower (Dynamic Contrast)
  ******************************************************************************
  * @attention
  *
  * REWRITTEN FOR RELIABILITY
  * Implements Dynamic Contrast Detection to handle changing floor colors.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Box_Handling.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS 5

// --- TUNING PARAMETERS ---
#define KP 35.0f
#define KD 100.0f
#define BASE_SPEED 100
#define MAX_PWM 200
#define TURN_SPEED 90

// LOGIC THRESHOLDS
// If Average Sensor Value > 2500, we assume White Floor (High values)
#define FLOOR_COLOR_THRESHOLD 2500

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
volatile uint16_t adc_buffer[NUM_SENSORS];

// Normalized Line Position (0-1000) for each sensor
float line_map[NUM_SENSORS];

// Weights: Left ... Center ... Right
const float sensor_weights[NUM_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

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
void Motor_SetSpeed(int left, int right);
void Handle_Turn(int direction);
void Calibrate_Sensors(void); // Keeping prototype but calibration is dynamic now
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_SetSpeed(int left, int right) {
  if (left > 255) left = 255; else if (left < -255) left = -255;
  if (right > 255) right = 255; else if (right < -255) right = -255;

  if (left >= 0) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -left);
  }

  if (right >= 0) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -right);
  }
}

// Robust Turn Function using Dynamic Detection
void Handle_Turn(int direction) {
    // 1. Stop
    Motor_SetSpeed(0, 0);
    HAL_Delay(100);

    // 2. Move Forward slightly
    Motor_SetSpeed(80, 80);
    HAL_Delay(250);

    // 3. Start Spin
    int spd_L = (direction == -1) ? -TURN_SPEED : TURN_SPEED;
    int spd_R = (direction == -1) ? TURN_SPEED : -TURN_SPEED;
    Motor_SetSpeed(spd_L, spd_R);
    HAL_Delay(300); // Blind spin

    // 4. Wait for Line
    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < 2000) { // 2s Timeout
        // Determine Contrast
        long sum = 0;
        for(int i=0; i<NUM_SENSORS; i++) sum += adc_buffer[i];
        long avg = sum / NUM_SENSORS;

        int center_val = adc_buffer[2];
        int on_line = 0;

        // Dynamic Threshold Check
        if (avg > FLOOR_COLOR_THRESHOLD) {
             // White Floor: Line is Low (Dark)
             if (center_val < (avg - 600)) on_line = 1;
        } else {
             // Black Floor: Line is High (White)
             if (center_val > (avg + 600)) on_line = 1;
        }

        if (on_line) {
            Motor_SetSpeed(0, 0);
            return;
        }

        HAL_Delay(5);
    }
    // Timeout - Stop
    Motor_SetSpeed(0, 0);
}

void Calibrate_Sensors(void) {
    // Legacy support: Just blink to show readiness
    // Since we use dynamic contrast, we don't need MIN/MAX arrays hardcoded.
    HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUM_SENSORS);

  // Wait to start
  HAL_Delay(1000);
  Calibrate_Sensors();

  /* Initialize Box Handling (Color Sensor + Electromagnet) */
  Box_Init(&htim3);

  float last_error = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // --- 1. SENSOR ANALYSIS ---
    long total_raw = 0;
    uint16_t max_val = 0;
    uint16_t min_val = 4096;

    for(int i=0; i<NUM_SENSORS; i++) {
        total_raw += adc_buffer[i];
        if (adc_buffer[i] > max_val) max_val = adc_buffer[i];
        if (adc_buffer[i] < min_val) min_val = adc_buffer[i];
    }
    long avg_raw = total_raw / NUM_SENSORS;

    // --- 2. DYNAMICAL NORMALIZATION ---
    // Detect Floor Color
    int white_floor = (avg_raw > FLOOR_COLOR_THRESHOLD);

    float weighted_sum = 0;
    float total_line_val = 0;

    for(int i=0; i<NUM_SENSORS; i++) {
        float val;

        if (white_floor) {
            // WHITE FLOOR (BackgroundHigh, LineLow)
            // Signal = (Background - Sensor)
            // Use local max as baseline for background to be robust
            if (max_val < 3800) max_val = 4095;

            val = (float)(max_val - adc_buffer[i]);
            if (val < 0) val = 0;
        } else {
            // BLACK FLOOR (BackgroundLow, LineHigh)
            // Signal = (Sensor - Background)
             // Use local min as baseline
            val = (float)(adc_buffer[i] - min_val);
            if (val < 0) val = 0;
        }

        line_map[i] = val;

        // Accumulate if signal is significant (> 300 to filter noise)
        if (val > 400.0f) {
            weighted_sum += val * sensor_weights[i];
            total_line_val += val;
        }
    }

    // --- 3. TURN DETECTION ---
    // If Edge Sensors are strongly triggered relative to center
    // Check normalized values strength (>700 is strong contrast)
    int turn_detected = 0;

    // Robust check: S0 is Strong AND triggers logic
    if (line_map[0] > 700 && line_map[0] > line_map[2] * 1.5) {
         Handle_Turn(-1);
         turn_detected = 1;
    }
    else if (line_map[4] > 700 && line_map[4] > line_map[2] * 1.5) {
         Handle_Turn(1);
         turn_detected = 1;
    }

    if (turn_detected) {
        last_error = 0;
        continue;
    }

    // --- 4. PID CALCULATION ---
    float position;
    if (total_line_val > 500) {
        position = weighted_sum / total_line_val;
    } else {
        // Line Lost -> Use last error to sweep
        if (last_error > 0) position = 2.5f;
        else position = -2.5f;
    }

    float error = position;
    float P = error;
    float D = error - last_error;
    last_error = error;

    float correction = (KP * P) + (KD * D);

    int left_motor = BASE_SPEED + (int)correction;
    int right_motor = BASE_SPEED - (int)correction;

    Motor_SetSpeed(left_motor, right_motor);

    /* Box Handling Process (Keep running this) */
    Box_Process();

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}
//...
// (Previous MX Functions follow, identical to before)
//...
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_10; sConfig.Rank = ADC_REGULAR_RANK_1; sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_11; sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_12; sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_8; sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_9; sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin|LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Electromagnet_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
