#ifndef __COLOR_H
#define __COLOR_H

#include "stm32f1xx_hal.h"
#include <stdio.h>

/* ================= PIN CONFIG ================= */

/* TCS3200 Control Pins */
#define COLOR_S0_PORT   GPIOB
#define COLOR_S0_PIN    GPIO_PIN_12

#define COLOR_S1_PORT   GPIOB
#define COLOR_S1_PIN    GPIO_PIN_13

#define COLOR_S2_PORT   GPIOB
#define COLOR_S2_PIN    GPIO_PIN_14

#define COLOR_S3_PORT   GPIOB
#define COLOR_S3_PIN    GPIO_PIN_15

/* RGB LED Pins (COMMON ANODE) */
#define COLOR_LED_R_PORT GPIOC
#define COLOR_LED_R_PIN  GPIO_PIN_6

#define COLOR_LED_G_PORT GPIOC
#define COLOR_LED_G_PIN  GPIO_PIN_8

#define COLOR_LED_B_PORT GPIOC
#define COLOR_LED_B_PIN  GPIO_PIN_9

/* ================= TUNING PARAMETERS ================= */
#define COLOR_G_GAIN    2.9f
#define COLOR_B_GAIN    1.99f
#define COLOR_DIFF      110

/* ================= API ================= */
void Color_Init(TIM_HandleTypeDef *htim);
void Color_Detect(void);

/* Optional raw access */
uint32_t Color_ReadRed(void);
uint32_t Color_ReadGreen(void);
uint32_t Color_ReadBlue(void);

#endif
