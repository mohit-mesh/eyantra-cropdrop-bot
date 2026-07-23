#include "Color.h"

/* ================= PRIVATE VARIABLES ================= */
static TIM_HandleTypeDef *color_tim;

static volatile uint32_t ic_val1 = 0;
static volatile uint32_t ic_val2 = 0;
static volatile uint8_t  is_first_captured = 0;
static volatile uint32_t frequency = 0;

/* ================= PRIVATE MACROS ================= */
#define S0(x) HAL_GPIO_WritePin(COLOR_S0_PORT, COLOR_S0_PIN, x)
#define S1(x) HAL_GPIO_WritePin(COLOR_S1_PORT, COLOR_S1_PIN, x)
#define S2(x) HAL_GPIO_WritePin(COLOR_S2_PORT, COLOR_S2_PIN, x)
#define S3(x) HAL_GPIO_WritePin(COLOR_S3_PORT, COLOR_S3_PIN, x)

/* RGB LED (COMMON ANODE → LOW = ON) */
#define LED_R_ON()   HAL_GPIO_WritePin(COLOR_LED_R_PORT, COLOR_LED_R_PIN, GPIO_PIN_RESET)
#define LED_R_OFF()  HAL_GPIO_WritePin(COLOR_LED_R_PORT, COLOR_LED_R_PIN, GPIO_PIN_SET)

#define LED_G_ON()   HAL_GPIO_WritePin(COLOR_LED_G_PORT, COLOR_LED_G_PIN, GPIO_PIN_RESET)
#define LED_G_OFF()  HAL_GPIO_WritePin(COLOR_LED_G_PORT, COLOR_LED_G_PIN, GPIO_PIN_SET)

#define LED_B_ON()   HAL_GPIO_WritePin(COLOR_LED_B_PORT, COLOR_LED_B_PIN, GPIO_PIN_RESET)
#define LED_B_OFF()  HAL_GPIO_WritePin(COLOR_LED_B_PORT, COLOR_LED_B_PIN, GPIO_PIN_SET)

/* ================= PRIVATE FUNCTIONS ================= */
static void RGB_LED_Off(void)
{
    LED_R_OFF();
    LED_G_OFF();
    LED_B_OFF();
}

static void RGB_LED_Red(void)
{
    LED_R_ON();
    LED_G_OFF();
    LED_B_OFF();
}

static void RGB_LED_Green(void)
{
    LED_R_OFF();
    LED_G_ON();
    LED_B_OFF();
}

static void RGB_LED_Blue(void)
{
    LED_R_OFF();
    LED_G_OFF();
    LED_B_ON();
}

/* ================= TIMER CALLBACK ================= */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == color_tim)
    {
        if (!is_first_captured)
        {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_captured = 1;
        }
        else
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (ic_val2 > ic_val1)
                frequency = 1000000 / (ic_val2 - ic_val1);
            else
                frequency = 1000000 / ((0xFFFF - ic_val1) + ic_val2);

            is_first_captured = 0;
        }
    }
}

/* ================= PUBLIC FUNCTIONS ================= */

void Color_Init(TIM_HandleTypeDef *htim)
{
    color_tim = htim;

    /* 100% frequency scaling */
    S0(GPIO_PIN_SET);
    S1(GPIO_PIN_SET);

    RGB_LED_Off();
    HAL_TIM_IC_Start_IT(color_tim, TIM_CHANNEL_1);
}

uint32_t Color_ReadRed(void)
{
    S2(GPIO_PIN_RESET);
    S3(GPIO_PIN_RESET);
    HAL_Delay(50);
    return frequency;
}

uint32_t Color_ReadGreen(void)
{
    S2(GPIO_PIN_SET);
    S3(GPIO_PIN_SET);
    HAL_Delay(50);
    return frequency;
}

uint32_t Color_ReadBlue(void)
{
    S2(GPIO_PIN_RESET);
    S3(GPIO_PIN_SET);
    HAL_Delay(50);
    return frequency;
}

void Color_Detect(void)
{
    uint32_t r = Color_ReadRed();
    uint32_t g = Color_ReadGreen();
    uint32_t b = Color_ReadBlue();

    uint32_t r_n = r;
    uint32_t g_n = g * COLOR_G_GAIN;
    uint32_t b_n = b * COLOR_B_GAIN;

    RGB_LED_Off();

    if ((r_n > g_n + COLOR_DIFF - 100) && (r_n > b_n + COLOR_DIFF - 95))
    {
        printf("RED   R=%lu G=%lu B=%lu\n", r_n, g_n, b_n);
        RGB_LED_Red();
    }
    else if ((g_n > r_n + COLOR_DIFF) && (g_n > b_n + COLOR_DIFF))
    {
        printf("GREEN R=%lu G=%lu B=%lu\n", r_n, g_n, b_n);
        RGB_LED_Green();
    }
    else if ((b_n > r_n + COLOR_DIFF) && (b_n > g_n + COLOR_DIFF))
    {
        printf("BLUE  R=%lu G=%lu B=%lu\n", r_n, g_n, b_n);
        RGB_LED_Blue();
    }
    else
    {
        printf("MIXED R=%lu G=%lu B=%lu\n", r_n, g_n, b_n);
        RGB_LED_Off();
    }
}
