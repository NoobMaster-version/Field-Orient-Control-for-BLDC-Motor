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

#define AS5600_ADDR               (0x36U << 1)
#define AS5600_RAW_ANGLE_REG      0x0CU
#define AS5600_STATUS_REG         0x0BU
#define AS5600_COUNTS_PER_REV     4096.0f

#define MOTOR_POLE_PAIRS          6.0f
#define ENCODER_DIRECTION         1.0f
#define ROTATION_DIRECTION        1.0f

#define PWM_FREQ_HZ               20000U
#define PWM_PERIOD                4250U
#define PWM_HALF_PERIOD           (PWM_PERIOD / 2U)
#define DEAD_TIME_COUNTS          20U

#define CONTROL_PERIOD_US         1000U
#define DEBUG_PERIOD_MS           250U

#define ALIGN_SECTOR_CENTER_RAD    (5.0f * PI_F / 3.0f)
#define ALIGN_DUTY_PERCENT        14U
#define ALIGN_RAMP_MS             250U
#define ALIGN_HOLD_MS             300U
#define ALIGN_SAMPLE_COUNT        24U
#define SINE_HANDOFF_MS           300U
#define RUN_SETTLE_MS             900U

#define RUN_START_MODULATION      0.07f
#define RUN_TARGET_MODULATION     0.18f
#define MODULATION_RAMP_MS        3200U
#define TARGET_MECH_SPEED_RPS     0.30f
#define SPEED_RAMP_MS             6500U
#define POSITION_KP               0.95f
#define FEEDFORWARD_LEAD_RAD      0.20f
#define MAX_TORQUE_LEAD_RAD       0.70f

#define I2C_TIMEOUT_MS            10U
#define STARTUP_IDLE_MS           100U

#define TWO_PI_F                  6.28318530718f
#define PI_F                      3.14159265359f
#define TWO_PI_BY_THREE_F         2.09439510239f
#define SQRT3_BY_2_F             0.86602540378f

typedef struct {
    uint16_t raw;
    float mech_angle_rad;
    float mech_angle_deg;
    float mech_unwrapped_rad;
    uint8_t magnet_ok;
    uint8_t valid;
} EncoderSample_t;

static volatile MotorState_t motor_state = MOTOR_IDLE;
static volatile uint16_t encoder_raw_dbg = 0U;
static volatile float encoder_mech_deg_dbg = 0.0f;
static volatile float encoder_mech_unwrapped_dbg = 0.0f;
static volatile float rotor_elec_deg_dbg = 0.0f;
static volatile float stator_elec_deg_dbg = 0.0f;
static volatile float target_mech_deg_dbg = 0.0f;
static volatile float lead_deg_dbg = 0.0f;
static volatile float speed_rpm_dbg = 0.0f;
static volatile float modulation_dbg = 0.0f;
static volatile uint8_t magnet_ok_dbg = 0U;
static volatile uint8_t i2c_fault_dbg = 0U;

static float zero_electrical_offset_rad = 0.0f;
static float mech_angle_prev_rad = 0.0f;
static float mech_angle_unwrapped_rad = 0.0f;
static float target_mech_angle_rad = 0.0f;
static uint8_t encoder_tracking_ready = 0U;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

static uint8_t as5600_read_raw(uint16_t *raw_angle);
static uint8_t as5600_magnet_detected(void);
static uint8_t encoder_sample(EncoderSample_t *sample);
static void encoder_reset_tracking(float mech_angle_rad);
static void outputs_hiz(void);
static void outputs_enable_sine_mode(void);
static void apply_sine_pwm(float stator_angle_rad, float modulation);
static void apply_two_phase_align(uint32_t duty_ccr);
static uint8_t driver_wake_and_check_fault(void);
static void driver_fault_stop(const char *reason);
static void motor_align_and_capture_zero(void);
static uint32_t micros32(void);
static void dwt_init(void);
static float clampf(float x, float lo, float hi);
static float normalize_angle(float angle);
static float wrap_pm_pi(float angle);
static float electrical_angle_from_mech(float mech_angle_rad);
static float radians_to_degrees(float radians);
static float lerp_angle(float a, float b, float t);

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
    return ch;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static float normalize_angle(float angle)
{
    while (angle >= TWO_PI_F) {
        angle -= TWO_PI_F;
    }
    while (angle < 0.0f) {
        angle += TWO_PI_F;
    }
    return angle;
}

static float wrap_pm_pi(float angle)
{
    while (angle > PI_F) {
        angle -= TWO_PI_F;
    }
    while (angle < -PI_F) {
        angle += TWO_PI_F;
    }
    return angle;
}

static float radians_to_degrees(float radians)
{
    return radians * (180.0f / PI_F);
}

static float lerp_angle(float a, float b, float t)
{
    return normalize_angle(a + wrap_pm_pi(b - a) * clampf(t, 0.0f, 1.0f));
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros32(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static uint8_t as5600_read_raw(uint16_t *raw_angle)
{
    uint8_t rx[2] = {0U, 0U};

    if (HAL_I2C_Mem_Read(&hi2c1,
                         AS5600_ADDR,
                         AS5600_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         rx,
                         2U,
                         I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    *raw_angle = (uint16_t)((((uint16_t)rx[0] << 8) | (uint16_t)rx[1]) & 0x0FFFU);
    return 1U;
}

static uint8_t as5600_magnet_detected(void)
{
    uint8_t status = 0U;

    if (HAL_I2C_Mem_Read(&hi2c1,
                         AS5600_ADDR,
                         AS5600_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         &status,
                         1U,
                         I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    return (status & (1U << 5)) ? 1U : 0U;
}

static uint8_t encoder_sample(EncoderSample_t *sample)
{
    uint16_t raw = 0U;
    float mech_angle_rad;

    if (!as5600_read_raw(&raw)) {
        sample->valid = 0U;
        return 0U;
    }

    mech_angle_rad = ((float)raw * TWO_PI_F) / AS5600_COUNTS_PER_REV;

    sample->raw = raw;
    sample->mech_angle_rad = mech_angle_rad;
    sample->mech_angle_deg = radians_to_degrees(mech_angle_rad);
    sample->magnet_ok = as5600_magnet_detected();
    sample->valid = 1U;

    if (!encoder_tracking_ready) {
        encoder_reset_tracking(mech_angle_rad);
    } else {
        float delta = wrap_pm_pi(mech_angle_rad - mech_angle_prev_rad);
        mech_angle_unwrapped_rad += delta;
        mech_angle_prev_rad = mech_angle_rad;
    }

    sample->mech_unwrapped_rad = mech_angle_unwrapped_rad;
    return 1U;
}

static void encoder_reset_tracking(float mech_angle_rad)
{
    mech_angle_prev_rad = mech_angle_rad;
    mech_angle_unwrapped_rad = mech_angle_rad;
    encoder_tracking_ready = 1U;
}

static void outputs_hiz(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);
}

static void outputs_enable_sine_mode(void)
{
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_SET);
}

static void apply_two_phase_align(uint32_t duty_ccr)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

static void apply_sine_pwm(float stator_angle_rad, float modulation)
{
    float angle = normalize_angle(stator_angle_rad);
    float uq = clampf(modulation, 0.0f, 0.95f) * 0.5f;
    float sa = sinf(angle);
    float ca = cosf(angle);
    float ualpha = -sa * uq;
    float ubeta =  ca * uq;
    float ua = ualpha + 0.5f;
    float ub = (-0.5f * ualpha + SQRT3_BY_2_F * ubeta) + 0.5f;
    float uc = (-0.5f * ualpha - SQRT3_BY_2_F * ubeta) + 0.5f;
    uint32_t ccr_a = (uint32_t)clampf(ua * (float)PWM_PERIOD, 1.0f, (float)(PWM_PERIOD - 1U));
    uint32_t ccr_b = (uint32_t)clampf(ub * (float)PWM_PERIOD, 1.0f, (float)(PWM_PERIOD - 1U));
    uint32_t ccr_c = (uint32_t)clampf(uc * (float)PWM_PERIOD, 1.0f, (float)(PWM_PERIOD - 1U));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

static uint8_t driver_wake_and_check_fault(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5U);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
        return 1U;
    }

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
    printf("FAULT: outputs disabled, driver reset required\r\n");

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5U);

    Error_Handler();
}

static float electrical_angle_from_mech(float mech_angle_rad)
{
    return normalize_angle(zero_electrical_offset_rad
                           + (ENCODER_DIRECTION * MOTOR_POLE_PAIRS * mech_angle_rad));
}

static void motor_align_and_capture_zero(void)
{
    uint32_t start_ms = HAL_GetTick();
    float mean_sin = 0.0f;
    float mean_cos = 0.0f;
    uint32_t sample_count = 0U;
    uint32_t align_duty_ccr = (ALIGN_DUTY_PERCENT * PWM_PERIOD) / 100U;

    motor_state = MOTOR_ALIGNING;

    printf("ALIGN: 2-phase lock, ramp %lu ms, hold %lu ms, duty %u%%\r\n",
           (unsigned long)ALIGN_RAMP_MS,
           (unsigned long)ALIGN_HOLD_MS,
           (unsigned)ALIGN_DUTY_PERCENT);

    while ((HAL_GetTick() - start_ms) < ALIGN_RAMP_MS) {
        uint32_t elapsed = HAL_GetTick() - start_ms;
        uint32_t duty = (align_duty_ccr * elapsed) / ALIGN_RAMP_MS;
        apply_two_phase_align(duty);

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {
            driver_fault_stop("nFAULT during 2-phase alignment ramp");
        }
        HAL_Delay(1U);
    }

    apply_two_phase_align(align_duty_ccr);
    HAL_Delay(ALIGN_HOLD_MS);

    for (sample_count = 0U; sample_count < ALIGN_SAMPLE_COUNT; ++sample_count) {
        EncoderSample_t sample;
        if (!encoder_sample(&sample)) {
            driver_fault_stop("AS5600 read failed during alignment capture");
        }
        mean_sin += sinf(sample.mech_angle_rad);
        mean_cos += cosf(sample.mech_angle_rad);
        HAL_Delay(2U);
    }

    {
        float aligned_mech = atan2f(mean_sin, mean_cos);
        if (aligned_mech < 0.0f) {
            aligned_mech += TWO_PI_F;
        }

        zero_electrical_offset_rad = normalize_angle(
            ALIGN_SECTOR_CENTER_RAD - (ENCODER_DIRECTION * MOTOR_POLE_PAIRS * aligned_mech));

        encoder_reset_tracking(aligned_mech);
        target_mech_angle_rad = mech_angle_unwrapped_rad;

        encoder_mech_deg_dbg = radians_to_degrees(aligned_mech);
        target_mech_deg_dbg = radians_to_degrees(target_mech_angle_rad);

        printf("ALIGN: rotor settled\r\n");
        printf("ENCODER: mech=%.2f deg -> electrical zero=%.2f deg\r\n",
               (double)radians_to_degrees(aligned_mech),
               (double)radians_to_degrees(zero_electrical_offset_rad));

        outputs_enable_sine_mode();
        apply_sine_pwm(ALIGN_SECTOR_CENTER_RAD, RUN_START_MODULATION);
        HAL_Delay(SINE_HANDOFF_MS);
    }
}

int main(void)
{
    uint32_t last_control_us;
    uint32_t last_debug_ms;
    uint32_t run_start_ms;
    float speed_estimate_rps = 0.0f;
    EncoderSample_t sample;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();
    dwt_init();

    printf("\r\n=== DRV8311H + AS5600 Sinusoidal 3x PWM ===\r\n");
    printf("PWM          : %u Hz center aligned\r\n", PWM_FREQ_HZ);
    printf("Motor        : %.0f pole pairs\r\n", (double)MOTOR_POLE_PAIRS);
    printf("Run speed    : %.2f rps mech (%.1f rpm)\r\n",
           (double)TARGET_MECH_SPEED_RPS,
           (double)(TARGET_MECH_SPEED_RPS * 60.0f));
    printf("Run mod      : %.0f%% -> %.0f%%\r\n",
           (double)(RUN_START_MODULATION * 100.0f),
           (double)(RUN_TARGET_MODULATION * 100.0f));
    printf("Encoder dir  : %.0f | Rotation dir : %.0f\r\n",
           (double)ENCODER_DIRECTION,
           (double)ROTATION_DIRECTION);

    outputs_hiz();

    if (!driver_wake_and_check_fault()) {
        printf("ERROR: nFAULT remained low after reset\r\n");
        Error_Handler();
    }
    printf("nFAULT: OK\r\n");

    magnet_ok_dbg = as5600_magnet_detected();
    printf("AS5600: magnet=%s\r\n", magnet_ok_dbg ? "OK" : "NO");
    if (!magnet_ok_dbg) {
        printf("ERROR: AS5600 magnet not detected\r\n");
        Error_Handler();
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_HALF_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_HALF_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_HALF_PERIOD);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_MOE_ENABLE(&htim1);
    printf("BDTR = 0x%08lX\r\n", (unsigned long)TIM1->BDTR);

    HAL_Delay(STARTUP_IDLE_MS);
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {
        driver_fault_stop("nFAULT low before alignment");
    }

    motor_align_and_capture_zero();

    if (!encoder_sample(&sample)) {
        driver_fault_stop("AS5600 read failed after alignment");
    }

    encoder_raw_dbg = sample.raw;
    encoder_mech_deg_dbg = sample.mech_angle_deg;
    encoder_mech_unwrapped_dbg = radians_to_degrees(sample.mech_unwrapped_rad);
    magnet_ok_dbg = sample.magnet_ok;
    rotor_elec_deg_dbg = radians_to_degrees(electrical_angle_from_mech(sample.mech_unwrapped_rad));

    run_start_ms = HAL_GetTick();
    last_control_us = micros32();
    last_debug_ms = run_start_ms;
    target_mech_angle_rad = sample.mech_unwrapped_rad;
    motor_state = MOTOR_RUNNING;

    printf("RUN: sinusoidal encoder voltage mode active\r\n");

    while (1) {
        uint32_t now_us = micros32();
        uint32_t dt_us = now_us - last_control_us;

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) {
            driver_fault_stop("nFAULT asserted during run");
        }

        if (dt_us < CONTROL_PERIOD_US) {
            continue;
        }
        last_control_us = now_us;

        if (!encoder_sample(&sample)) {
            i2c_fault_dbg = 1U;
            continue;
        }
        i2c_fault_dbg = 0U;

        {
            float dt = (float)dt_us * 1.0e-6f;
            float elapsed_ms = (float)(HAL_GetTick() - run_start_ms);
            float settle_ramp = clampf(elapsed_ms / (float)RUN_SETTLE_MS, 0.0f, 1.0f);
            float speed_ramp = clampf(elapsed_ms / (float)SPEED_RAMP_MS, 0.0f, 1.0f);
            float mod_ramp = clampf(elapsed_ms / (float)MODULATION_RAMP_MS, 0.0f, 1.0f);
            float speed_cmd = TARGET_MECH_SPEED_RPS * speed_ramp;
            float modulation = RUN_START_MODULATION
                             + (RUN_TARGET_MODULATION - RUN_START_MODULATION) * mod_ramp;
            float previous_mech = encoder_mech_unwrapped_dbg * (PI_F / 180.0f);
            float delta_mech = sample.mech_unwrapped_rad - previous_mech;
            float mech_error;
            float elec_error;
            float rotor_elec;
            float lead_rad;
            float control_theta;
            float stator_theta;

            speed_estimate_rps = 0.90f * speed_estimate_rps
                               + 0.10f * (delta_mech / (TWO_PI_F * dt));

            target_mech_angle_rad += ROTATION_DIRECTION * speed_cmd * TWO_PI_F * dt;
            mech_error = target_mech_angle_rad - sample.mech_unwrapped_rad;
            elec_error = wrap_pm_pi(MOTOR_POLE_PAIRS * mech_error);

            lead_rad = clampf(POSITION_KP * elec_error
                            + (ROTATION_DIRECTION * FEEDFORWARD_LEAD_RAD * speed_ramp),
                              -MAX_TORQUE_LEAD_RAD,
                               MAX_TORQUE_LEAD_RAD);

            rotor_elec = electrical_angle_from_mech(sample.mech_unwrapped_rad);
            control_theta = normalize_angle(rotor_elec + lead_rad);
            stator_theta = lerp_angle(ALIGN_SECTOR_CENTER_RAD, control_theta, settle_ramp);

            outputs_enable_sine_mode();
            apply_sine_pwm(stator_theta, modulation);

            encoder_raw_dbg = sample.raw;
            encoder_mech_deg_dbg = sample.mech_angle_deg;
            encoder_mech_unwrapped_dbg = radians_to_degrees(sample.mech_unwrapped_rad);
            rotor_elec_deg_dbg = radians_to_degrees(rotor_elec);
            stator_elec_deg_dbg = radians_to_degrees(stator_theta);
            target_mech_deg_dbg = radians_to_degrees(target_mech_angle_rad);
            lead_deg_dbg = radians_to_degrees(lead_rad);
            speed_rpm_dbg = speed_estimate_rps * 60.0f;
            modulation_dbg = modulation * 100.0f;
            magnet_ok_dbg = sample.magnet_ok;
        }

        if ((HAL_GetTick() - last_debug_ms) >= DEBUG_PERIOD_MS) {
            printf("t=%lums | mech=%.2f deg | mech_pos=%.2f deg | target=%.2f deg | rotor_e=%.2f deg | stator_e=%.2f deg | lead=%.2f deg | mod=%.1f%% | speed=%.2f rpm | raw=%u | magnet=%s | i2c=%s\r\n",
                   (unsigned long)HAL_GetTick(),
                   (double)encoder_mech_deg_dbg,
                   (double)encoder_mech_unwrapped_dbg,
                   (double)target_mech_deg_dbg,
                   (double)rotor_elec_deg_dbg,
                   (double)stator_elec_deg_dbg,
                   (double)lead_deg_dbg,
                   (double)modulation_dbg,
                   (double)speed_rpm_dbg,
                   (unsigned)encoder_raw_dbg,
                   magnet_ok_dbg ? "OK" : "NO",
                   i2c_fault_dbg ? "ERR" : "OK");
            last_debug_ms = HAL_GetTick();
        }
    }
}

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
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_HALF_PERIOD;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->EGR = TIM_EGR_UG;

    sBDT.OffStateRunMode = TIM_OSSR_ENABLE;
    sBDT.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBDT.LockLevel = TIM_LOCKLEVEL_OFF;
    sBDT.DeadTime = DEAD_TIME_COUNTS;
    sBDT.BreakState = TIM_BREAK_DISABLE;
    sBDT.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBDT.BreakFilter = 0U;
    sBDT.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBDT.Break2State = TIM_BREAK2_DISABLE;
    sBDT.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBDT.Break2Filter = 0U;
    sBDT.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBDT.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBDT) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim1);
}

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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x40B285C2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK) {
        Error_Handler();
    }
}

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
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U;

    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert: %s line %u\r\n", file, (unsigned)line);
}
#endif
