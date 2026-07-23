/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define IR1_Pin GPIO_PIN_0
#define IR1_GPIO_Port GPIOC
#define IR2_Pin GPIO_PIN_1
#define IR2_GPIO_Port GPIOC
#define IR3_Pin GPIO_PIN_2
#define IR3_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Electromagnet_Pin GPIO_PIN_4
#define Electromagnet_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Color_out_Pin GPIO_PIN_6
#define Color_out_GPIO_Port GPIOA
#define IR4_Pin GPIO_PIN_0
#define IR4_GPIO_Port GPIOB
#define IR5_Pin GPIO_PIN_1
#define IR5_GPIO_Port GPIOB
#define In3_Pin GPIO_PIN_10
#define In3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_11
#define IN4_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_12
#define S0_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_13
#define S1_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_14
#define S2_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_15
#define S3_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_6
#define RED_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_8
#define GREEN_GPIO_Port GPIOC
#define BLUE_Pin GPIO_PIN_9
#define BLUE_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Box_detect_Pin GPIO_PIN_15
#define Box_detect_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
