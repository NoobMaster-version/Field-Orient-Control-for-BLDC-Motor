/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Open-loop 3-PWM BLDC bring-up for STM32G431CBT6 + DRV8311H
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD_TICKS         1699U
#define PWM_CENTER_TICKS         ((PWM_PERIOD_TICKS + 1U) / 2U)
#define PWM_FREQ_HZ              50000.0f

#define ADC_FULL_SCALE           4095.0f
#define ADC_VREF_V               3.3f
#define SHUNT_RESISTOR_OHMS      0.010f
#define CSA_GAIN_V_V             20.0f

#define OPEN_LOOP_ALIGN_MOD      0.025f
#define OPEN_LOOP_RUN_MOD        0.030f
#define OPEN_LOOP_ALIGN_MS       1200U
#define OPEN_LOOP_TARGET_EHZ     0.8f
#define OPEN_LOOP_RAMP_EHZ_S     0.25f

#define UART_PRINT_PERIOD_MS     100U
#define CURRENT_OFFSET_SAMPLES   256U
#define DRV_POWER_SETTLE_MS      50U
#define DRV_WAKE_TIMEOUT_MS      100U
#define DRV_FAULT_RELEASE_MS     2U

#define PI_F                     3.14159265359f
#define TWO_PI_F                 (2.0f * PI_F)

#define AS5600_I2C_ADDR          (0x36U << 1)
#define AS5600_REG_ANGLE_MSB     0x0EU

#define DRV_NSLEEP_GPIO_Port     GPIOC
#define DRV_NSLEEP_Pin           GPIO_PIN_13
#define DRV_NFAULT_GPIO_Port     GPIOC
#define DRV_NFAULT_Pin           GPIO_PIN_14

#define DRV_INL_GPIO_Port        GPIOB
#define DRV_INLA_Pin             GPIO_PIN_13
#define DRV_INLB_Pin             GPIO_PIN_14
#define DRV_INLC_Pin             GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static volatile uint32_t adc1_dma_buf[2];

static float ia_offset_counts = 2048.0f;
static float ib_offset_counts = 2048.0f;
static float ic_offset_counts = 2048.0f;

static float electrical_angle = 0.0f;
static float electrical_hz = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Error_Stop(const char *msg);
static void uart_write(const char *msg);
static void drv_disable(void);
static void drv_enable(void);
static uint8_t drv_has_fault(void);
static uint8_t drv_fault_released_for_ms(uint32_t hold_ms);
static void phase_inputs_low_side_enable(void);
static void phase_inputs_disable(void);
static uint8_t drv_wake_and_check(uint32_t retries, char *msg, uint32_t msg_len);
static void set_three_phase_duty(float duty_a, float duty_b, float duty_c);
static void set_open_loop_voltage(float angle_rad, float modulation);
static uint16_t as5600_read_raw_angle(void);
static float as5600_read_deg(void);
static uint32_t adc2_read_raw_blocking(void);
static void calibrate_current_offsets(void);
static float current_from_adc_counts(float raw_counts, float offset_counts);
static float wrap_angle_0_2pi(float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_write(const char *msg)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1U, 10U);
  return ch;
}

static void Error_Stop(const char *msg)
{
  set_three_phase_duty(0.5f, 0.5f, 0.5f);
  drv_disable();
  if (msg != NULL)
  {
    uart_write(msg);
  }
  while (1)
  {
  }
}

static void drv_disable(void)
{
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_RESET);
}

static void drv_enable(void)
{
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_SET);
}

static uint8_t drv_has_fault(void)
{
  return (HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint8_t drv_fault_released_for_ms(uint32_t hold_ms)
{
  uint32_t stable_since;

  if (drv_has_fault() != 0U)
  {
    return 0U;
  }

  stable_since = HAL_GetTick();
  while ((HAL_GetTick() - stable_since) < hold_ms)
  {
    if (drv_has_fault() != 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static void phase_inputs_low_side_enable(void)
{
  HAL_GPIO_WritePin(DRV_INL_GPIO_Port, DRV_INLA_Pin | DRV_INLB_Pin | DRV_INLC_Pin, GPIO_PIN_SET);
}

static void phase_inputs_disable(void)
{
  HAL_GPIO_WritePin(DRV_INL_GPIO_Port, DRV_INLA_Pin | DRV_INLB_Pin | DRV_INLC_Pin, GPIO_PIN_RESET);
}

static uint8_t drv_wake_and_check(uint32_t retries, char *msg, uint32_t msg_len)
{
  uint32_t attempt;
  uint32_t start_tick;

  drv_disable();
  HAL_Delay(2U);

  for (attempt = 0U; attempt < retries; attempt++)
  {
    if (attempt != 0U)
    {
      drv_disable();
      HAL_Delay(2U);
    }

    drv_enable();
    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < DRV_WAKE_TIMEOUT_MS)
    {
      if (drv_fault_released_for_ms(DRV_FAULT_RELEASE_MS) != 0U)
      {
        if (msg != NULL)
        {
          snprintf(msg, msg_len,
                   "DRV wake OK at attempt=%lu ns=%u nf=%u\r\n",
                   (unsigned long)(attempt + 1U),
                   (unsigned int)HAL_GPIO_ReadPin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin),
                   (unsigned int)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin));
        }
        return 1U;
      }

      HAL_Delay(1U);
    }
  }

  if (msg != NULL)
  {
    snprintf(msg, msg_len,
             "DRV wake failed after %lu attempts ns=%u nf=%u (check nFAULT pull-up to AVDD, VM, MODE strap)\r\n",
             (unsigned long)retries,
             (unsigned int)HAL_GPIO_ReadPin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin),
             (unsigned int)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin));
  }
  return 0U;
}

static float clamp01(float x)
{
  if (x < 0.0f)
  {
    return 0.0f;
  }
  if (x > 1.0f)
  {
    return 1.0f;
  }
  return x;
}

static void set_three_phase_duty(float duty_a, float duty_b, float duty_c)
{
  uint32_t ccr_a = (uint32_t)(clamp01(duty_a) * (float)(PWM_PERIOD_TICKS + 1U));
  uint32_t ccr_b = (uint32_t)(clamp01(duty_b) * (float)(PWM_PERIOD_TICKS + 1U));
  uint32_t ccr_c = (uint32_t)(clamp01(duty_c) * (float)(PWM_PERIOD_TICKS + 1U));

  if (ccr_a > PWM_PERIOD_TICKS)
  {
    ccr_a = PWM_PERIOD_TICKS;
  }
  if (ccr_b > PWM_PERIOD_TICKS)
  {
    ccr_b = PWM_PERIOD_TICKS;
  }
  if (ccr_c > PWM_PERIOD_TICKS)
  {
    ccr_c = PWM_PERIOD_TICKS;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

static float wrap_angle_0_2pi(float angle)
{
  while (angle >= TWO_PI_F)
  {
    angle -= TWO_PI_F;
  }
  while (angle < 0.0f)
  {
    angle += TWO_PI_F;
  }
  return angle;
}

static void set_open_loop_voltage(float angle_rad, float modulation)
{
  float va;
  float vb;
  float vc;

  if (modulation < 0.0f)
  {
    modulation = 0.0f;
  }
  if (modulation > 0.12f)
  {
    modulation = 0.12f;
  }

  angle_rad = wrap_angle_0_2pi(angle_rad);

  va = sinf(angle_rad);
  vb = sinf(angle_rad - (TWO_PI_F / 3.0f));
  vc = sinf(angle_rad + (TWO_PI_F / 3.0f));

  set_three_phase_duty(0.5f + 0.5f * modulation * va,
                       0.5f + 0.5f * modulation * vb,
                       0.5f + 0.5f * modulation * vc);
}

static uint16_t as5600_read_raw_angle(void)
{
  uint8_t reg = AS5600_REG_ANGLE_MSB;
  uint8_t data[2] = {0U, 0U};

  if (HAL_I2C_Master_Transmit(&hi2c1, AS5600_I2C_ADDR, &reg, 1U, 20U) != HAL_OK)
  {
    return 0xFFFFU;
  }

  if (HAL_I2C_Master_Receive(&hi2c1, AS5600_I2C_ADDR, data, 2U, 20U) != HAL_OK)
  {
    return 0xFFFFU;
  }

  return (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8) | data[1]);
}

static float as5600_read_deg(void)
{
  uint16_t raw = as5600_read_raw_angle();
  if (raw == 0xFFFFU)
  {
    return -1.0f;
  }
  return ((float)raw * 360.0f) / 4096.0f;
}

static uint32_t adc2_read_raw_blocking(void)
{
  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc2, 5U) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc2);
    return 0U;
  }

  uint32_t value = HAL_ADC_GetValue(&hadc2);
  HAL_ADC_Stop(&hadc2);
  return value;
}

static void calibrate_current_offsets(void)
{
  uint32_t i;
  uint64_t sum_a = 0U;
  uint64_t sum_b = 0U;
  uint64_t sum_c = 0U;

  set_three_phase_duty(0.5f, 0.5f, 0.5f);
  HAL_Delay(20U);

  for (i = 0U; i < CURRENT_OFFSET_SAMPLES; i++)
  {
    HAL_Delay(1U);
    sum_a += adc1_dma_buf[0];
    sum_b += adc2_read_raw_blocking();
    sum_c += adc1_dma_buf[1];
  }

  ia_offset_counts = (float)sum_a / (float)CURRENT_OFFSET_SAMPLES;
  ib_offset_counts = (float)sum_b / (float)CURRENT_OFFSET_SAMPLES;
  ic_offset_counts = (float)sum_c / (float)CURRENT_OFFSET_SAMPLES;
}

static float current_from_adc_counts(float raw_counts, float offset_counts)
{
  float delta_v = ((raw_counts - offset_counts) / ADC_FULL_SCALE) * ADC_VREF_V;
  return delta_v / (CSA_GAIN_V_V * SHUNT_RESISTOR_OHMS);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t last_tick;
  uint32_t last_print_tick;
  uint32_t now;
  uint32_t dt_ms;
  float dt_s;
  uint32_t raw_b;
  float ia;
  float ib;
  float ic;
  float enc_deg;
  int32_t ia_mA;
  int32_t ib_mA;
  int32_t ic_mA;
  int32_t sum_mA;
  int32_t enc_t10;
  int32_t ehz_x1000;
  char tx[192];

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();

  drv_disable();
  phase_inputs_disable();
  set_three_phase_duty(0.5f, 0.5f, 0.5f);

  uart_write("\r\nOpen-loop mode (safe low power) start\r\n");

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  set_three_phase_duty(0.5f, 0.5f, 0.5f);
  phase_inputs_low_side_enable();
  HAL_Delay(1U);
  HAL_Delay(DRV_POWER_SETTLE_MS);

  if (drv_wake_and_check(2U, tx, sizeof(tx)) == 0U)
  {
    uart_write(tx);
    uart_write("DRV fault asserted during wake sequence; holding diagnostic state\r\n");
    while (1)
    {
      snprintf(tx, sizeof(tx),
               "WAKE_FAIL ns=%u nf=%u (check external nFAULT pull-up, VM, MODE)\r\n",
               (unsigned int)HAL_GPIO_ReadPin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin),
               (unsigned int)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin));
      uart_write(tx);
      HAL_Delay(250U);
    }
  }
  uart_write(tx);

  if (drv_has_fault() != 0U)
  {
    snprintf(tx, sizeof(tx),
             "Fault after PWM/INL enable: ns=%u nf=%u\r\n",
             (unsigned int)HAL_GPIO_ReadPin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin),
             (unsigned int)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin));
    uart_write(tx);
    Error_Stop("DRV fault asserted after PWM start\r\n");
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_buf, 2U) != HAL_OK)
  {
    Error_Handler();
  }

  calibrate_current_offsets();
  set_open_loop_voltage(0.0f, OPEN_LOOP_ALIGN_MOD);
  HAL_Delay(OPEN_LOOP_ALIGN_MS);

  electrical_angle = 0.0f;
  electrical_hz = 0.0f;
  last_tick = HAL_GetTick();
  last_print_tick = last_tick;

  while (1)
  {
    now = HAL_GetTick();
    dt_ms = now - last_tick;
    if (dt_ms == 0U)
    {
      continue;
    }
    last_tick = now;
    dt_s = (float)dt_ms * 0.001f;

    if (drv_has_fault() != 0U)
    {
      Error_Stop("DRV fault during run\r\n");
    }

    if (electrical_hz < OPEN_LOOP_TARGET_EHZ)
    {
      electrical_hz += OPEN_LOOP_RAMP_EHZ_S * dt_s;
      if (electrical_hz > OPEN_LOOP_TARGET_EHZ)
      {
        electrical_hz = OPEN_LOOP_TARGET_EHZ;
      }
    }

    electrical_angle += TWO_PI_F * electrical_hz * dt_s;
    electrical_angle = wrap_angle_0_2pi(electrical_angle);
    set_open_loop_voltage(electrical_angle, OPEN_LOOP_RUN_MOD);

    raw_b = adc2_read_raw_blocking();
    ia = current_from_adc_counts((float)adc1_dma_buf[0], ia_offset_counts);
    ib = current_from_adc_counts((float)raw_b, ib_offset_counts);
    ic = current_from_adc_counts((float)adc1_dma_buf[1], ic_offset_counts);
    enc_deg = as5600_read_deg();
    ia_mA = (int32_t)(ia * 1000.0f);
    ib_mA = (int32_t)(ib * 1000.0f);
    ic_mA = (int32_t)(ic * 1000.0f);
    sum_mA = ia_mA + ib_mA + ic_mA;
    enc_t10 = (enc_deg < 0.0f) ? -1 : (int32_t)(enc_deg * 10.0f);
    ehz_x1000 = (int32_t)(electrical_hz * 1000.0f);

    if ((now - last_print_tick) >= UART_PRINT_PERIOD_MS)
    {
      last_print_tick = now;
      snprintf(tx, sizeof(tx),
               "ehz_x1000=%ld ns=%u nf=%u IA_mA=%ld IB_mA=%ld IC_mA=%ld SUM_mA=%ld ENC_t10=%ld SOB=%lu\r\n",
               (long)ehz_x1000,
               (unsigned int)HAL_GPIO_ReadPin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin),
               (unsigned int)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin),
               (long)ia_mA,
               (long)ib_mA,
               (long)ic_mA,
               (long)sum_mA,
               (long)enc_t10,
               (unsigned long)raw_b);
      uart_write(tx);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_PERIOD_TICKS;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_CENTER_TICKS;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, DRV_NSLEEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DRV_INLA_Pin | DRV_INLB_Pin | DRV_INLC_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRV_NSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DRV_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DRV_INLA_Pin | DRV_INLB_Pin | DRV_INLC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
