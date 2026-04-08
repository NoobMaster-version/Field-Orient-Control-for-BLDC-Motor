#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart3;

typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ALIGNING,
    MOTOR_RUNNING,
    MOTOR_FAULT
} MotorState_t;

/* ── AS5600 ──────────────────────────────────────────────────────────────── */
#define AS5600_ADDR               (0x36U << 1)
#define AS5600_RAW_ANGLE_REG      0x0CU
#define AS5600_STATUS_REG         0x0BU
#define AS5600_COUNTS_PER_REV     4096.0f

/* ── Motor ───────────────────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS          6.0f
#define ROTATION_DIRECTION        1.0f   /* 1 = CCW, -1 = CW (swap if wrong) */

/* ── PWM ─────────────────────────────────────────────────────────────────── */
#define PWM_FREQ_HZ               20000U
#define PWM_PERIOD                4250U  /* ARR: 170 MHz / (2 × 4250) = 20 kHz */
#define PWM_HALF_PERIOD           (PWM_PERIOD / 2U)
#define DEAD_TIME_COUNTS          20U

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define CONTROL_PERIOD_US         500U   /* 2 kHz control loop                */
#define DEBUG_PERIOD_MS           250U

/* ── Alignment ───────────────────────────────────────────────────────────── */
/*
 * Keep the DRV8311H in 3x PWM mode from the moment the windings are energised.
 * The previous approach locked the rotor with A-high / B-low / C-Hi-Z and then
 * enabled the third phase during the sine handoff. In 3x PWM mode that Hi-Z to
 * active transition happens at the instantaneous INHx level, not at a true
 * "neutral" output, which can inject a full half-bridge step and trip OCP on a
 * low-inductance 1104 motor.
 *
 * Instead, enable all three phases while the timer is already outputting the
 * same 50% duty on A/B/C (zero line-line voltage), then ramp a fixed-angle sine
 * vector to pull the rotor onto a known electrical axis before open-loop run.
 */
#define ALIGN_START_ELEC_RAD      (PI_F / 3.0f)  /* 60° electrical */
#define ALIGN_LOCK_MODULATION     0.10f          /* ~10.4% VAB, close to old 6-step 10% start */
#define ALIGN_RAMP_MS             1000U
#define ALIGN_HOLD_MS             2000U

/* ── Open-loop ramp ──────────────────────────────────────────────────────── */
/* Start with the same modulation used during sine-mode alignment to avoid a
 * torque jump when the open-loop angle begins advancing. Keep startup voltage
 * conservative; a 4300 KV 1104 motor has low phase inductance and can trip the
 * DRV8311H OCP if the stator vector is too strong at standstill.
 */
#define RUN_START_MODULATION      ALIGN_LOCK_MODULATION
#define RUN_TARGET_MODULATION     0.14f
#define MODULATION_RAMP_MS        3200U
#define TARGET_MECH_SPEED_RPS     0.30f           /* mechanical rps                   */
#define SPEED_RAMP_MS             10500U //6500 initial value

/* ── Misc ────────────────────────────────────────────────────────────────── */
#define I2C_TIMEOUT_MS            10U
#define STARTUP_IDLE_MS           100U

#define TWO_PI_F                  6.28318530718f
#define PI_F                      3.14159265359f
#define TWO_PI_BY_THREE_F         2.09439510239f

/* ── Debug state (inspectable via debugger / printf) ─────────────────────── */
static volatile MotorState_t motor_state       = MOTOR_IDLE;
static volatile uint16_t     encoder_raw_dbg   = 0U;
static volatile float        encoder_deg_dbg   = 0.0f;
static volatile float        theta_e_deg_dbg   = 0.0f;
static volatile float        omega_e_dbg       = 0.0f;
static volatile float        modulation_dbg    = 0.0f;
static volatile uint8_t      magnet_ok_dbg     = 0U;

/* ── Function prototypes ─────────────────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

static uint8_t as5600_read_raw(uint16_t *raw_angle);
static uint8_t as5600_magnet_detected(void);
static void    outputs_hiz(void);
static void    outputs_enable_sine_mode(void);
static void    apply_sine_pwm(float theta_e, float modulation);
static uint8_t driver_wake_and_check_fault(void);
static void    driver_fault_stop(const char *reason);
static void    motor_align(void);
static uint32_t micros32(void);
static void    dwt_init(void);
static float   clampf(float x, float lo, float hi);
static float   normalize_angle(float angle);

/* ── printf routing ──────────────────────────────────────────────────────── */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
    return ch;
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float normalize_angle(float angle)
{
    while (angle >= TWO_PI_F) angle -= TWO_PI_F;
    while (angle < 0.0f)      angle += TWO_PI_F;
    return angle;
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros32(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* ── AS5600 ──────────────────────────────────────────────────────────────── */
static uint8_t as5600_read_raw(uint16_t *raw_angle)
{
    uint8_t rx[2] = {0U, 0U};

    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT, rx, 2U, I2C_TIMEOUT_MS) != HAL_OK)
        return 0U;

    *raw_angle = (uint16_t)((((uint16_t)rx[0] << 8) | (uint16_t)rx[1]) & 0x0FFFU);
    return 1U;
}

static uint8_t as5600_magnet_detected(void)
{
    uint8_t status = 0U;

    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT, &status, 1U, I2C_TIMEOUT_MS) != HAL_OK)
        return 0U;

    return (status & (1U << 5)) ? 1U : 0U;
}

/* ── Phase output control ────────────────────────────────────────────────── */
static void outputs_hiz(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}

static void outputs_enable_sine_mode(void)
{
    /* All INLx HIGH: low-side switches are enabled — timer controls high side */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
}

/* apply_sine_pwm — direct 3-phase sinusoidal PWM
 *
 * theta_e   : electrical angle, radians [0, 2π)
 * modulation: voltage modulation index [0, 1]
 *
 * Phase duties centered on 0.5 (= Vmid):
 *   da = 0.5 + (M/2) * sin(θ)
 *   db = 0.5 + (M/2) * sin(θ − 2π/3)
 *   dc = 0.5 + (M/2) * sin(θ + 2π/3)
 *
 * At θ = π/3 (60°, ALIGN_START_ELEC_RAD):
 *   da = 0.5 + 0.866*(M/2)  → A HIGH   matches 2-phase alignment A-high ✓
 *   db = 0.5 − 0.866*(M/2)  → B LOW    matches 2-phase alignment B-low  ✓
 *   dc = 0.5 + 0            → C Vmid   C was Hi-Z; driven to neutral    ✓
 *
 * NOTE: this replaces the previous Park-transform version which had a 90°
 * field offset and used 5π/3 (300°) — that caused a 60° stator field jump
 * at handoff from 2-phase alignment, spiking current and tripping nFAULT.
 */
static void apply_sine_pwm(float theta_e, float modulation)
{
    float half_M = clampf(modulation, 0.0f, 0.95f) * 0.5f;

    float da = 0.5f + half_M * sinf(theta_e);
    float db = 0.5f + half_M * sinf(theta_e - TWO_PI_BY_THREE_F);
    float dc = 0.5f + half_M * sinf(theta_e + TWO_PI_BY_THREE_F);

    uint32_t ccr_a = (uint32_t)clampf(da * (float)PWM_PERIOD, 1.0f, (float)(PWM_PERIOD - 1U));
    uint32_t ccr_b = (uint32_t)clampf(db * (float)PWM_PERIOD, 1.0f, (float)(PWM_PERIOD - 1U));
    uint32_t ccr_c = (uint32_t)clampf(dc * (float)PWM_PERIOD, 1.0f, (float)(PWM_PERIOD - 1U));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

/* ── Driver ──────────────────────────────────────────────────────────────── */
static uint8_t driver_wake_and_check_fault(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5U);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET)
        return 1U;

    printf("WARNING: nFAULT low at startup, resetting driver\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5U);

    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) ? 1U : 0U;
}

static void driver_fault_stop(const char *reason)
{
    motor_state = MOTOR_FAULT;
    outputs_hiz();
    printf("FAULT: %s\r\n", reason);
    printf("FAULT: outputs disabled, power-cycle or reset to restart\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5U);
    Error_Handler();
}

/* ── Alignment ───────────────────────────────────────────────────────────── */
/* motor_align — enable 3x PWM at neutral, then ramp a fixed-angle sine vector
 *
 * 1. Enable all INL pins while A/B/C are already at the same 50% duty.
 * 2. Ramp a 60° electrical sine vector to ALIGN_LOCK_MODULATION.
 * 3. Hold that vector so the rotor settles on a known electrical axis.
 */
static void motor_align(void)
{
    uint32_t start_ms;

    motor_state = MOTOR_ALIGNING;
    printf("ALIGN: 3x PWM fixed-angle lock | ramp %lu ms | hold %lu ms | theta=60 deg | mod %.1f%%\r\n",
           (unsigned long)ALIGN_RAMP_MS,
           (unsigned long)ALIGN_HOLD_MS,
           (double)(ALIGN_LOCK_MODULATION * 100.0f));

    /* Enter 3x mode while all three phases already have the same 50% duty.
     * That gives zero line-line voltage, so enabling INLx does not step the
     * motor into an arbitrary sector. */
    outputs_enable_sine_mode();
    HAL_Delay(2U);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
        driver_fault_stop("nFAULT when enabling 3x PWM neutral state");

    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < ALIGN_RAMP_MS) {
        float elapsed    = (float)(HAL_GetTick() - start_ms);
        float modulation = ALIGN_LOCK_MODULATION * clampf(elapsed / (float)ALIGN_RAMP_MS, 0.0f, 1.0f);
        apply_sine_pwm(ALIGN_START_ELEC_RAD, modulation);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
            driver_fault_stop("nFAULT during sine alignment ramp");
        HAL_Delay(1U);
    }

    apply_sine_pwm(ALIGN_START_ELEC_RAD, ALIGN_LOCK_MODULATION);
    theta_e_deg_dbg = ALIGN_START_ELEC_RAD * (180.0f / PI_F);
    modulation_dbg  = ALIGN_LOCK_MODULATION * 100.0f;

    HAL_Delay(ALIGN_HOLD_MS);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
        driver_fault_stop("nFAULT at end of sine alignment hold");

    printf("ALIGN: rotor settled in sine mode\r\n");
}

/* ── Main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();
    dwt_init();

    printf("\r\n=== DRV8311H Open-Loop Sinusoidal 3x PWM ===\r\n");
    printf("PWM        : %u Hz center aligned\r\n", PWM_FREQ_HZ);
    printf("Pole pairs : %.0f\r\n", (double)MOTOR_POLE_PAIRS);
    printf("Target     : %.2f rps mech (%.1f rpm)\r\n",
           (double)TARGET_MECH_SPEED_RPS,
           (double)(TARGET_MECH_SPEED_RPS * 60.0f));
    printf("Modulation : %.0f%% -> %.0f%% over %lu ms\r\n",
           (double)(RUN_START_MODULATION * 100.0f),
           (double)(RUN_TARGET_MODULATION * 100.0f),
           (unsigned long)MODULATION_RAMP_MS);
    printf("Speed ramp : 0 -> target over %lu ms\r\n", (unsigned long)SPEED_RAMP_MS);

    outputs_hiz();

    if (!driver_wake_and_check_fault()) {
        printf("ERROR: nFAULT remained low after reset\r\n");
        Error_Handler();
    }
    printf("nFAULT: OK\r\n");

    magnet_ok_dbg = as5600_magnet_detected();
    printf("AS5600: magnet=%s\r\n", magnet_ok_dbg ? "OK" : "NO");
    /* Open-loop does not require the encoder — only used for display */

    /* Pre-load neutral duty, start PWM channels, assert MOE */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_HALF_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_HALF_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_HALF_PERIOD);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    __HAL_TIM_MOE_ENABLE(&htim1);
    printf("BDTR = 0x%08lX\r\n", (unsigned long)TIM1->BDTR);

    HAL_Delay(STARTUP_IDLE_MS);
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
        driver_fault_stop("nFAULT low before alignment");

    /* Lock rotor with a fixed-angle sine vector, then advance the same vector */
    motor_align();

    /* ─── Open-loop sinusoidal run ──────────────────────────────────────── */
    float theta_e    = ALIGN_START_ELEC_RAD;
    float omega_target = ROTATION_DIRECTION
                       * TARGET_MECH_SPEED_RPS * MOTOR_POLE_PAIRS * TWO_PI_F;

    uint32_t run_start_ms  = HAL_GetTick();
    uint32_t last_debug_ms = run_start_ms;
    uint32_t last_ctrl_us  = micros32();

    motor_state = MOTOR_RUNNING;
    printf("RUN: open-loop sinusoidal | omega_target=%.4f rad/s elec (%.4f rad/s mech)\r\n",
           (double)omega_target,
           (double)(omega_target / MOTOR_POLE_PAIRS));

    while (1) {
        uint32_t now_us = micros32();
        uint32_t dt_us  = now_us - last_ctrl_us;

        /* nFAULT is checked every iteration — not gated by control period */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
            driver_fault_stop("nFAULT asserted during run");

        if (dt_us < CONTROL_PERIOD_US)
            continue;

        last_ctrl_us = now_us;   /* timestamp BEFORE any blocking calls */

        float dt         = (float)dt_us * 1.0e-6f;
        float elapsed_ms = (float)(HAL_GetTick() - run_start_ms);

        /* Speed ramp: 0 → omega_target over SPEED_RAMP_MS */
        float speed_ramp = clampf(elapsed_ms / (float)SPEED_RAMP_MS, 0.0f, 1.0f);
        float omega_e    = omega_target * speed_ramp;

        /* Modulation ramp: RUN_START → RUN_TARGET over MODULATION_RAMP_MS */
        float mod_ramp   = clampf(elapsed_ms / (float)MODULATION_RAMP_MS, 0.0f, 1.0f);
        float modulation = RUN_START_MODULATION
                         + (RUN_TARGET_MODULATION - RUN_START_MODULATION) * mod_ramp;

        /* Advance electrical angle and apply sine voltages */
        theta_e = normalize_angle(theta_e + omega_e * dt);
        apply_sine_pwm(theta_e, modulation);

        /* Update debug vars (no I2C here — keeps control loop deterministic) */
        theta_e_deg_dbg = theta_e * (180.0f / PI_F);
        omega_e_dbg     = omega_e;
        modulation_dbg  = modulation * 100.0f;

        /* Encoder display — only at debug rate, I2C read happens here */
        if ((HAL_GetTick() - last_debug_ms) >= DEBUG_PERIOD_MS) {
            uint16_t raw = 0U;
            if (as5600_read_raw(&raw)) {
                encoder_raw_dbg = raw;
                encoder_deg_dbg = ((float)raw / AS5600_COUNTS_PER_REV) * 360.0f;
                magnet_ok_dbg   = as5600_magnet_detected();
            }
            printf("t=%lums | theta_e=%.1f deg | omega_e=%.4f rad/s | mod=%.1f%% | mech=%.2f deg | raw=%u | magnet=%s\r\n",
                   (unsigned long)HAL_GetTick(),
                   (double)theta_e_deg_dbg,
                   (double)omega_e_dbg,
                   (double)modulation_dbg,
                   (double)encoder_deg_dbg,
                   (unsigned)encoder_raw_dbg,
                   magnet_ok_dbg ? "OK" : "NO");
            last_debug_ms = HAL_GetTick();
        }
    }
}

/* ── TIM1 Init ───────────────────────────────────────────────────────────── */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef        sMasterConfig = {0};
    TIM_OC_InitTypeDef             sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBDT = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = PWM_HALF_PERIOD;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->EGR    = TIM_EGR_UG;

    sBDT.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBDT.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBDT.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBDT.DeadTime         = DEAD_TIME_COUNTS;
    sBDT.BreakState       = TIM_BREAK_DISABLE;
    sBDT.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBDT.BreakFilter      = 0U;
    sBDT.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBDT.Break2State      = TIM_BREAK2_DISABLE;
    sBDT.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBDT.Break2Filter     = 0U;
    sBDT.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBDT.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBDT) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

/* ── Clock / GPIO / I2C / UART inits (unchanged) ────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM           = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN           = 85;
    RCC_OscInitStruct.PLL.PLLP           = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ           = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR           = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x40B285C2;
    hi2c1.Init.OwnAddress1      = 0U;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0U;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance            = USART3;
    huart3.Init.BaudRate       = 115200;
    huart3.Init.WordLength     = UART_WORDLENGTH_8B;
    huart3.Init.StopBits       = UART_STOPBITS_1;
    huart3.Init.Parity         = UART_PARITY_NONE;
    huart3.Init.Mode           = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U;

    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert: %s line %u\r\n", file, (unsigned)line);
}
#endif
