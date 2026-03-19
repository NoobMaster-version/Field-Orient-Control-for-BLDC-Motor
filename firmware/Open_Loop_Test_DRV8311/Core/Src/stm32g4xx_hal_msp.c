/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_hal_msp.c
  * @brief   MSP for DRV8311H 3x PWM + AS5600 encoder
  *
  * ============================================================
  *  FIX vs original hal_msp.c
  * ============================================================
  * BUG (original): HAL_TIM_MspPostInit initialised PB13/14/15 LOW with
  *   the comment "3x PWM MODE: INLx MUST be LOW". This is WRONG for the
  *   independent 3x PWM mode used here.
  *
  *   In DRV8311H 3x PWM mode the MCU drives INHx AND INLx independently:
  *     INLx=0 + INHx=0  → Hi-Z   (floating phase)
  *     INLx=1 + INHx=1  → H      (high-side ON)
  *     INLx=1 + INHx=0  → L      (low-side ON)
  *   apply_commutation_step() in main.c sets INLx correctly per step.
  *
  *   If MspPostInit forces INLx LOW after main.c has already set up the
  *   GPIO pins, every commutation step that relies on INLx=HIGH becomes
  *   broken: no current flows through the low-side phase, the motor gets
  *   no torque in those steps, and it vibrates/rocks instead of spinning.
  *
  *   FIX: HAL_TIM_MspPostInit now ONLY configures the AF PWM pins
  *   (PA8/PA9/PA10). The INLx GPIO direction is still set here so the
  *   pin is correctly initialised as an output, but the initial level is
  *   LOW (Hi-Z), and main.c / apply_commutation_step() takes full
  *   ownership of their runtime state.
  *
  * Peripherals initialised here:
  *   ADC1   PB0        ADC1_IN15  analog   AS5600 analog OUT
  *   I2C1   PA15/PB7   AF4        100kHz   AS5600 I2C
  *   TIM1   PA8/9/10   AF6        PWM      INHA/INHB/INHC
  *          PB13/14/15 GPIO LOW            INLA/INLB/INLC (runtime by main.c)
  *   USART3 PB10/11    AF7        115200   debug UART
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_DisableUCPDDeadBattery();
}

/* ===========================================================================
 * ADC1 — PB0 = ADC1_IN15 — AS5600 analog OUT
 * No DMA needed — continuous free-running, polled with HAL_ADC_GetValue()
 * =========================================================================*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit   = {0};

    if (hadc->Instance == ADC1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_ADC12_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin  = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC12_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
    }
}

/* ===========================================================================
 * I2C1 — PA15=SCL(AF4)  PB7=SDA(AF4)
 * =========================================================================*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit   = {0};

    if (hi2c->Instance == I2C1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
        PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PA15 → I2C1_SCL */
        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PB7 → I2C1_SDA */
        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
    }
}

/* ===========================================================================
 * TIM1 base clock
 * =========================================================================*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if (htim_base->Instance == TIM1)
        __HAL_RCC_TIM1_CLK_ENABLE();
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if (htim_base->Instance == TIM1)
        __HAL_RCC_TIM1_CLK_DISABLE();
}

/* ===========================================================================
 * HAL_TIM_MspPostInit — GPIO for TIM1 PWM outputs + INLx direction only
 *
 * PA8/PA9/PA10 → TIM1 CH1/2/3 (INHA/INHB/INHC) AF6  [PWM outputs]
 * PB13/14/15   → INLA/INLB/INLC GPIO output, initialised LOW (Hi-Z)
 *
 * !! DO NOT force INLx HIGH here !!
 * In DRV8311H independent 3x PWM mode:
 *   INLx = 0  → that phase's low-side FET is OFF  → combined with INHx=0 = Hi-Z
 *   INLx = 1  → low-side FET is enabled by INHx state
 * apply_commutation_step() in main.c sets INLx per commutation step at
 * runtime. This function only sets the GPIO direction; it must not override
 * the runtime INLx level.
 * =========================================================================*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA8/PA9/PA10 → TIM1 CH1/2/3 AF6 (INHA/INHB/INHC) */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PB13/14/15 → INLA/INLB/INLC
         * Direction: output push-pull.
         * Initial level: LOW (all phases Hi-Z at power-on).
         * Runtime level: controlled exclusively by apply_commutation_step()
         *                in main.c — do NOT set HIGH here.                  */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOB,
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                          GPIO_PIN_RESET);   /* LOW = Hi-Z at startup */
        GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = 0;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ===========================================================================
 * USART3 — PB10=TX(AF7)  PB11=RX(AF7)
 * =========================================================================*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit   = {0};

    if (huart->Instance == USART3)
    {
        PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_USART3;
        PeriphClkInit.Usart3ClockSelection  = RCC_USART3CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    }
}
