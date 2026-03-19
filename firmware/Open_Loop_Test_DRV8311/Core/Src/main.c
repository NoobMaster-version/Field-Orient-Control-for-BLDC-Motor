/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Open-loop 6-step BLDC startup — DRV8311H 3× PWM mode
  *                   STM32G431CBT6 @ 170 MHz
  *
  * ============================================================
  *  WHAT WAS WRONG IN failed_test.c — and what was fixed here
  * ============================================================
  *
  * BUG 1 — Sinusoidal / SVM drive with no angle feedback
  *   failed_test.c computed sine waves at an arbitrary speed and wrote the
  *   result directly to the three PWM channels. Without a rotor-position
  *   sensor or BEMF estimator the stator field rotates at whatever speed the
  *   MCU chooses; the rotor simply cannot follow from rest and only buzzes.
  *   FIX: replaced with explicit alignment + open-loop 6-step commutation.
  *   The rotor locks to a known electrical position, then the firmware steps
  *   through the six commutation states with a decreasing inter-step delay so
  *   the rotor can accelerate gradually before reaching steady speed.
  *
  * BUG 2 — INLx held permanently HIGH
  *   failed_test.c set PB13/14/15 HIGH and never touched them again. In 3×PWM
  *   mode each commutation step requires one phase to be Hi-Z (floating). Hi-Z
  *   is achieved by setting INLx = 0 *and* INHx = 0 for that phase. With INLx
  *   always HIGH the low-side of the floating phase switches every PWM cycle,
  *   creating spurious currents and preventing clean 6-step operation.
  *   FIX: apply_commutation_step() drives INLx GPIOs per-step, asserting only
  *   the two active phases and clearing the floating phase.
  *
  * BUG 3 — ISR doing heavy floating-point math at 40 kHz
  *   Three sinf() calls + SVM + dead-time compensation inside a 40 kHz ISR
  *   is CPU-intensive and unnecessary for 6-step. The corrected ISR is a
  *   simple tick counter; the main loop calls commutation_tick_handler() at a
  *   safe poll rate and the actual commutation table lookup involves no FP.
  *
  * BUG 4 — No alignment step
  *   failed_test.c started the sine ramp immediately. With no knowledge of
  *   initial rotor position the first electrical cycle could push the rotor in
  *   the wrong direction.
  *   FIX: motor_align() holds step 0 at a low duty for ALIGN_TIME_MS before
  *   any commutation begins.
  *
  * PRESERVED from failed_test.c (these were correct):
  *   - RepetitionCounter = 0   (UEV every overflow)
  *   - NVIC explicitly enabled for TIM1_UP_TIM16_IRQn
  *   - TIM1_UP_TIM16_IRQHandler defined here (remove from stm32g4xx_it.c!)
  *   - HAL_TIM_MOE_ENABLE moved after HAL_TIM_Base_Start_IT
  *   - UIF cleared before enabling IT
  *   - MOE verified with hard check after setting
  *
  * ============================================================
  *  DRV8311H 3× PWM semantics (one line)
  *  INLx = 0 → Hi-Z  |  INLx=1 + INHx=1 → H  |  INLx=1 + INHx=0 → L
  * ============================================================
  *
  * SAFETY NOTES (read before powering up):
  *   - First test with NO motor attached and a bench PSU with current limit.
  *   - Verify VM > 2.7 V, AVDD > 3 V, and nFAULT HIGH before enabling PWM.
  *   - Keep START_DUTY_PERCENT low (≤ 15 %) for initial tests.
  *   - If nFAULT remains LOW after one reset attempt the code halts; do not
  *     short-circuit this check.
  *
  * HARDWARE:
  *   MCU  : STM32G431CBT6
  *   PA8  → TIM1_CH1 → DRV8311 INHA
  *   PA9  → TIM1_CH2 → DRV8311 INHB
  *   PA10 → TIM1_CH3 → DRV8311 INHC
  *   PB13 → INLA  (GPIO output)
  *   PB14 → INLB  (GPIO output)
  *   PB15 → INLC  (GPIO output)
  *   PC13 → nSLEEP (GPIO output, HIGH = awake)
  *   PC14 → nFAULT (GPIO input,  LOW = fault)
  *   SOA/SOB/SOC → ADC channels (see TODO below)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdint.h>

/* ===========================================================================
 * !! TUNING PARAMETERS — review every value before first power-on !!
 * =========================================================================*/

/* PWM carrier ---------------------------------------------------------------*/
#define PWM_FREQ_HZ               20000U    /* switching frequency             */
#define PWM_PERIOD                4250U     /* ARR: 170 MHz / (2 × 4250) = 20 kHz */
#define DEAD_TIME_COUNTS          20U       /* match sBDT.DeadTime below       */

/* Duty levels (0–100 %) — TODO: tune for your motor & supply voltage --------*/
#define MIN_DUTY_PERCENT          18U        /* absolute floor (avoid shoot-through) */
#define START_DUTY_PERCENT        10U       /* duty used during alignment + start   */
#define MAX_DUTY_PERCENT          45U       /* steady-state ceiling                 */

/* Alignment -----------------------------------------------------------------*/
#define ALIGN_TIME_MS             700U      /* TODO: increase if rotor slips       */

/* Commutation ramp ----------------------------------------------------------
 * A2212 2200KV on 2S (7.4V) idles at ~1000 RPM unloaded and spins up fast.
 * With 7 pole pairs: 1000 RPM mech = 7000 RPM elec = 117 Hz elec.
 * That is ~8.5 ms per electrical revolution = 1.4 ms per step at idle.
 *
 * Start slow enough for the rotor to lock on the first few steps, then
 * ramp quickly. COMMUTATION_RAMP_STEP reduces by 2 ms per revolution
 * (every 6 steps) which is aggressive enough to accelerate smoothly.
 *
 * Tuning guide:
 *   - If the motor stalls during ramp: increase START_COMMUTATION_DELAY_MS
 *     or decrease COMMUTATION_RAMP_STEP.
 *   - If it vibrates at the start: increase ALIGN_TIME_MS or
 *     START_DUTY_PERCENT (but watch current).
 *   - If it stalls before reaching MIN: decrease COMMUTATION_RAMP_STEP.
 * --------------------------------------------------------------------------*/
#define START_COMMUTATION_DELAY_MS  25U      /* ~125 Hz elec = ~18 Hz mech (2p=7) */
#define MIN_COMMUTATION_DELAY_MS    6U      /* ~500 Hz elec = ~71 Hz mech        */
#define COMMUTATION_RAMP_STEP_REVS  12U      /* decrease delay every N steps      */
#define COMMUTATION_RAMP_DEC_MS     1U      /* decrease by this many ms each time */

/* Fault retry ---------------------------------------------------------------*/
#define FAULT_MAX_RETRIES         1U        /* do NOT increase past 1              */

/* ===========================================================================
 * Derived constants (do not edit)
 * =========================================================================*/
#define DUTY_TO_CCR(pct)  ((uint32_t)((pct) * PWM_PERIOD / 100U))

/* ===========================================================================
 * TODO: ADC channel assignments for current sense (SOA/SOB/SOC)
 *   #define ADC_CHANNEL_SOA   ADC_CHANNEL_1  // TODO: set correct channel
 *   #define ADC_CHANNEL_SOB   ADC_CHANNEL_2  // TODO: set correct channel
 *   #define ADC_CHANNEL_SOC   ADC_CHANNEL_3  // TODO: set correct channel
 *
 * TODO: CSA gain / ADC scaling
 *   #define CSA_GAIN          40.0f          // DRV8311 GAIN pin = 3V3 → ×40
 *   #define SHUNT_OHMS        0.01f          // TODO: measure actual shunt
 *   // I_phase (A) = (ADC_raw / 4095.0f * 3.3f) / (CSA_GAIN * SHUNT_OHMS)
 * =========================================================================*/

/* ===========================================================================
 * HAL peripheral handles (CubeIDE generates these)
 * =========================================================================*/
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart3;

/* ===========================================================================
 * Motor state
 * =========================================================================*/
typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ALIGNING,
    MOTOR_RAMPING,
    MOTOR_RUNNING,
    MOTOR_FAULT
} MotorState_t;

static volatile MotorState_t motor_state          = MOTOR_IDLE;
static volatile uint8_t      comm_step            = 0;        /* 0–5           */
static volatile uint32_t     commutation_delay_ms = START_COMMUTATION_DELAY_MS;
static volatile uint32_t     last_comm_tick       = 0;
static volatile uint32_t     isr_tick             = 0;        /* for debug     */
static volatile uint8_t      fault_retry_count    = 0;
static volatile uint32_t     step_count           = 0;        /* total steps since ramp start */

/* duty_ccr — module-level so commutation_tick_handler() can ramp it.
 * apply_commutation_step() reads this directly.
 * Initialised in main() before motor_align() is called.                     */
static volatile uint32_t     duty_ccr = 0;

/* ===========================================================================
 * Private function prototypes
 * =========================================================================*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

void motor_align(void);
void apply_commutation_step(uint8_t step);
void commutation_tick_handler(void);
void emergency_stop(void);
static void handle_fault(void);

/* ===========================================================================
 * __io_putchar — routes printf → UART3
 * =========================================================================*/
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ===========================================================================
 * 6-Step commutation table
 *
 * Each row encodes one of the six electrical states for a standard
 * trapezoidal (6-step) BLDC drive.
 *
 * Columns: { INHA_duty, INHB_duty, INHC_duty, INLA, INLB, INLC }
 *
 * Convention (DRV8311 3× PWM):
 *   Active-HIGH phase  : INHx = PWM duty,    INLx = 1  → switches H
 *   Active-LOW  phase  : INHx = 0,           INLx = 1  → pulls L
 *   Floating    phase  : INHx = 0,           INLx = 0  → Hi-Z
 *
 * Winding current direction per step (classic trapezoidal, CCW convention):
 *   Step 0: A→B  (A high, B low,  C float)
 *   Step 1: A→C  (A high, B float,C low )
 *   Step 2: B→C  (A float,B high, C low )
 *   Step 3: B→A  (A low,  B high, C float)
 *   Step 4: C→A  (A low,  B float,C high)
 *   Step 5: C→B  (A float,B low,  C high)
 *
 * TODO: if motor spins in wrong direction swap any two phase wires or
 *       reverse the step sequence (use 5,4,3,2,1,0 ordering).
 * =========================================================================*/
typedef struct {
    uint8_t inh_pwm_a;   /* 1 = PWM, 0 = off */
    uint8_t inh_pwm_b;
    uint8_t inh_pwm_c;
    uint8_t inl_a;       /* GPIO level for INLA */
    uint8_t inl_b;
    uint8_t inl_c;
} CommStep_t;

static const CommStep_t COMM_TABLE[6] = {
    /* step 0: A-high, B-low,   C-float */  { 1, 0, 0,  1, 1, 0 },
    /* step 1: A-high, B-float, C-low   */  { 1, 0, 0,  1, 0, 1 },
    /* step 2: B-high, A-float, C-low   */  { 0, 1, 0,  0, 1, 1 },
    /* step 3: B-high, A-low,   C-float */  { 0, 1, 0,  1, 1, 0 },
    /* step 4: C-high, A-low,   B-float */  { 0, 0, 1,  1, 0, 1 },
    /* step 5: C-high, B-low,   A-float */  { 0, 0, 1,  0, 1, 1 },
};

/* ===========================================================================
 * emergency_stop
 *
 * De-energise all phases immediately:
 *   - Zero all PWM compare values  (INHx → 0)
 *   - Clear all INLx GPIOs          (phases → Hi-Z)
 * Motor will coast to a stop.
 * =========================================================================*/
void emergency_stop(void)
{
    /* Zero PWM output on all channels */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* Drive INLx LOW → all phases Hi-Z */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    motor_state = MOTOR_FAULT;
    printf("!!! EMERGENCY STOP — all phases Hi-Z\r\n");
}

/* ===========================================================================
 * apply_commutation_step
 *
 * Writes a single 6-step commutation state to the timer and GPIO registers.
 * The duty cycle for the active-high phase is read from the current
 * duty_ccr variable (set during align/ramp/run).
 *
 * Parameters:
 *   step  — 0–5, index into COMM_TABLE
 * =========================================================================*/
void apply_commutation_step(uint8_t step)
{
    if (step > 5) return;

    const CommStep_t *s = &COMM_TABLE[step];

    /* duty_ccr is a module-level variable set by main() and updated by
     * commutation_tick_handler() during the ramp. Do NOT use a static
     * local here — that was the bug that locked duty at START_DUTY_PERCENT
     * forever regardless of the ramp progress.                             */
    uint32_t d = duty_ccr;

    /* --- Write PWM compare registers --------------------------------------- */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s->inh_pwm_a ? d : 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, s->inh_pwm_b ? d : 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, s->inh_pwm_c ? d : 0U);

    /* --- Write INLx GPIO pins ---------------------------------------------- */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,
                      s->inl_a ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,
                      s->inl_b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,
                      s->inl_c ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===========================================================================
 * motor_align
 *
 * Locks the rotor to a known electrical position (step 0) by holding the
 * phase energised at START_DUTY_PERCENT for ALIGN_TIME_MS milliseconds.
 *
 * After this call the rotor sits near the step-0 stator axis. Commutation
 * can then begin from step 0 with a predictable initial torque direction.
 *
 * TODO: increase ALIGN_TIME_MS if the rotor does not settle reliably.
 *       Reduce START_DUTY_PERCENT if the motor jumps/vibrates too hard.
 * =========================================================================*/
void motor_align(void)
{
    motor_state = MOTOR_ALIGNING;
    printf("ALIGN: holding step 0 for %u ms at %u%% duty\r\n",
           ALIGN_TIME_MS, START_DUTY_PERCENT);

    apply_commutation_step(0);
    comm_step = 0;

    HAL_Delay(ALIGN_TIME_MS);

    printf("ALIGN: complete\r\n");
}

/* ===========================================================================
 * commutation_tick_handler
 *
 * Call this from the main loop (or a lower-priority timer ISR) to advance
 * the commutation state machine. It uses HAL_GetTick() to implement the
 * inter-step delay without blocking.
 *
 * Behaviour by motor_state:
 *   MOTOR_RAMPING  — advance step, decrease commutation_delay_ms by
 *                    COMMUTATION_RAMP_STEP until MIN_COMMUTATION_DELAY_MS.
 *   MOTOR_RUNNING  — advance step at MIN_COMMUTATION_DELAY_MS (steady).
 *   Other states   — no-op.
 * =========================================================================*/
void commutation_tick_handler(void)
{
    if (motor_state != MOTOR_RAMPING && motor_state != MOTOR_RUNNING)
        return;

    uint32_t now = HAL_GetTick();
    if ((now - last_comm_tick) < commutation_delay_ms)
        return;

    last_comm_tick = now;

    /* Advance commutation step (0–5, wrap) */
    comm_step = (comm_step + 1U) % 6U;
    step_count++;
    apply_commutation_step(comm_step);

    /* Debug print once per electrical revolution (every 6 steps) */
    if (comm_step == 0)
        printf("COMM: revolution | delay=%lums | duty_ccr=%lu\r\n",
               commutation_delay_ms, duty_ccr);

    /* Ramp: every COMMUTATION_RAMP_STEP_REVS steps, reduce the inter-step
     * delay AND increase duty slightly to maintain torque at higher speed.  */
    if (motor_state == MOTOR_RAMPING)
    {
        if ((step_count % COMMUTATION_RAMP_STEP_REVS) == 0)
        {
            /* Reduce commutation delay */
            if (commutation_delay_ms > MIN_COMMUTATION_DELAY_MS + COMMUTATION_RAMP_DEC_MS)
            {
                commutation_delay_ms -= COMMUTATION_RAMP_DEC_MS;

                /* Also ramp duty up toward MAX so torque scales with speed */
                uint32_t max_ccr = DUTY_TO_CCR(MAX_DUTY_PERCENT);
                if (duty_ccr < max_ccr)
                {
                    /* Increase duty by ~1% of PWM_PERIOD per ramp tick     */
                    duty_ccr += (PWM_PERIOD / 100U);
                    if (duty_ccr > max_ccr) duty_ccr = max_ccr;
                }
            }
            else
            {
                commutation_delay_ms = MIN_COMMUTATION_DELAY_MS;
                duty_ccr             = DUTY_TO_CCR(MAX_DUTY_PERCENT);
                motor_state          = MOTOR_RUNNING;
                printf("COMM: ramp complete — steady @ %lu ms/step, duty_ccr=%lu\r\n",
                       commutation_delay_ms, duty_ccr);
            }
        }
    }
}

/* ===========================================================================
 * handle_fault
 *
 * Called from the main loop when nFAULT (PC14) is detected LOW.
 * Attempts one reset cycle by briefly pulling nSLEEP low, then re-checks.
 * Halts if the fault persists after FAULT_MAX_RETRIES attempts.
 * =========================================================================*/
static void handle_fault(void)
{
    emergency_stop();

    if (fault_retry_count >= FAULT_MAX_RETRIES)
    {
        printf("FAULT: retry limit reached — halting. Check wiring / VM.\r\n");
        Error_Handler();
    }

    fault_retry_count++;
    printf("FAULT: nFAULT asserted — reset attempt %u of %u\r\n",
           fault_retry_count, FAULT_MAX_RETRIES);

    /* Pulse nSLEEP low 20–50 µs to reset the DRV8311H fault latch */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1);   /* ≥ 20 µs required; 1 ms is safe */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5);   /* tWAKE settle */

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        printf("FAULT: nFAULT still LOW after reset — halting.\r\n");
        printf("       Check VM, AVDD, motor winding, OCP threshold.\r\n");
        Error_Handler();
    }

    printf("FAULT: cleared — motor halted. Power-cycle to restart.\r\n");
    /* Intentionally do NOT restart the motor automatically.
     * A human should investigate before re-enabling drive.   */
}

/* ===========================================================================
 * TIM1 Update ISR — fires at 40 kHz (center-aligned, RC=0)
 *
 * Kept lean: only increments isr_tick for timing statistics.
 * All commutation logic runs in the main-loop via commutation_tick_handler().
 * =========================================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;
    isr_tick++;
}

/* ===========================================================================
 * TIM1_UP_TIM16_IRQHandler
 *
 * !! IMPORTANT !!
 * On STM32G4, TIM1 update and TIM16 share one IRQ vector.
 * DELETE the matching handler in stm32g4xx_it.c to avoid a linker error.
 * =========================================================================*/
void TIM1_UP_TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/* ===========================================================================
 * main
 * =========================================================================*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();

    printf("\r\n=== DRV8311H BLDC 6-Step Open-Loop Drive ===\r\n");
    printf("MCU   : STM32G431CBT6 @ 170 MHz\r\n");
    printf("PWM   : %u Hz center-aligned | period = %u counts\r\n",
           PWM_FREQ_HZ, PWM_PERIOD);
    printf("Align : %u ms @ %u%% duty\r\n", ALIGN_TIME_MS, START_DUTY_PERCENT);
    printf("Ramp  : %u → %u ms/step, dec every %u steps by %u ms\r\n",
           START_COMMUTATION_DELAY_MS, MIN_COMMUTATION_DELAY_MS,
           COMMUTATION_RAMP_STEP_REVS, COMMUTATION_RAMP_DEC_MS);

    /* -----------------------------------------------------------------------
     * 1. Wake DRV8311H — drive nSLEEP HIGH and wait for charge pump
     * --------------------------------------------------------------------- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(5);   /* tWAKE ~1 ms; 5 ms gives charge-pump margin */

    /* -----------------------------------------------------------------------
     * 2. Check / clear nFAULT before touching PWM
     * --------------------------------------------------------------------- */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        printf("WARNING: nFAULT LOW at startup — attempting clear\r\n");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(5);

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
        {
            printf("ERROR: nFAULT still LOW. Possible causes:\r\n");
            printf("  - VM undervoltage (need > 2.7 V)\r\n");
            printf("  - AVDD cap not charged / missing\r\n");
            printf("  - nFAULT pullup resistor missing\r\n");
            printf("  - OCP: short on motor wires or winding fault\r\n");
            Error_Handler();
        }
    }
    printf("nFAULT: OK\r\n");

    /* -----------------------------------------------------------------------
     * 3. Pre-load 0 duty on all channels (safe: no current until MOE set)
     * --------------------------------------------------------------------- */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* All INLx LOW → all phases Hi-Z at startup */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    /* -----------------------------------------------------------------------
     * 4. Start PWM output channels
     * --------------------------------------------------------------------- */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PA8  → INHA */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* PA9  → INHB */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* PA10 → INHC */

    /* -----------------------------------------------------------------------
     * 5. Clear pending update flag BEFORE enabling interrupt.
     *    TIM_EGR_UG written in MX_TIM1_Init sets UIF; clear it now so the
     *    ISR does not fire spuriously on the very first enable.
     * --------------------------------------------------------------------- */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    /* -----------------------------------------------------------------------
     * 6. Start timer base interrupt (sets UIE and configures NVIC)
     * --------------------------------------------------------------------- */
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
        printf("ERROR: HAL_TIM_Base_Start_IT failed\r\n");
        Error_Handler();
    }

    /* -----------------------------------------------------------------------
     * 7. Assert MOE *after* HAL_TIM_Base_Start_IT (that call can clear MOE)
     * --------------------------------------------------------------------- */
    __HAL_TIM_MOE_ENABLE(&htim1);
    printf("BDTR = 0x%08lX  (bit15 MOE must = 1)\r\n", TIM1->BDTR);

    if (!(TIM1->BDTR & TIM_BDTR_MOE))
    {
        printf("ERROR: MOE bit not set — check LockLevel in BDTR config\r\n");
        Error_Handler();
    }

    /* -----------------------------------------------------------------------
     * 8. Dwell at idle (0% duty, Hi-Z phases) for 200 ms — verify no fault
     * --------------------------------------------------------------------- */
    printf("Holding idle (Hi-Z) for 200 ms...\r\n");
    HAL_Delay(200);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        printf("ERROR: nFAULT at idle — hardware fault. Halting.\r\n");
        Error_Handler();
    }

    /* -----------------------------------------------------------------------
     * 9. Initialise duty and alignment step — lock rotor to step 0
     * --------------------------------------------------------------------- */
    duty_ccr = DUTY_TO_CCR(START_DUTY_PERCENT);  /* must be set before align */
    motor_align();

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        printf("ERROR: nFAULT during alignment — OCP? Reduce START_DUTY_PERCENT.\r\n");
        Error_Handler();
    }

    /* -----------------------------------------------------------------------
     * 10. Begin open-loop commutation ramp
     * --------------------------------------------------------------------- */
    commutation_delay_ms = START_COMMUTATION_DELAY_MS;
    last_comm_tick       = HAL_GetTick();
    motor_state          = MOTOR_RAMPING;
    printf("COMM: starting ramp — %u ms/step → %u ms/step\r\n",
           START_COMMUTATION_DELAY_MS, MIN_COMMUTATION_DELAY_MS);

    /* -----------------------------------------------------------------------
     * TODO: ADC / OCP hook
     *   Insert HAL_ADC_Start_DMA() here once ADC channels are configured for
     *   SOA/SOB/SOC.  Sample in the TIM1 ISR or a DMA complete callback and
     *   compare against an overcurrent threshold derived from:
     *     I_trip (A) = (adc_raw / 4095.0f * 3.3f) / (CSA_GAIN * SHUNT_OHMS)
     *   Call emergency_stop() if I_trip is exceeded.
     * --------------------------------------------------------------------- */

    /* =====================================================================
     * Main loop — commutation tick + fault monitoring + debug print
     * =================================================================== */
    uint32_t last_print  = 0;
    uint8_t  prev_nfault = GPIO_PIN_SET;

    while (1)
    {
        /* ----- Commutation advance ---------------------------------------- */
        commutation_tick_handler();

        /* ----- nFAULT monitoring (edge-triggered) ------------------------- */
        uint8_t nfault = (uint8_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);

        if (nfault != prev_nfault)
        {
            if (nfault == GPIO_PIN_RESET)
            {
                printf("!!! nFAULT ASSERTED\r\n");
                handle_fault();   /* halts or stops motor; see function above */
            }
            else
            {
                printf("nFAULT: de-asserted (cleared externally)\r\n");
            }
            prev_nfault = nfault;
        }

        /* ----- Debug print every 500 ms ----------------------------------- */
        if (HAL_GetTick() - last_print >= 500U)
        {
            const char *state_str;
            switch (motor_state)
            {
                case MOTOR_IDLE:      state_str = "IDLE";      break;
                case MOTOR_ALIGNING:  state_str = "ALIGNING";  break;
                case MOTOR_RAMPING:   state_str = "RAMPING";   break;
                case MOTOR_RUNNING:   state_str = "RUNNING";   break;
                case MOTOR_FAULT:     state_str = "FAULT";     break;
                default:              state_str = "UNKNOWN";   break;
            }

            printf("t=%lums | state=%s | step=%u | delay=%lums | isr_tick=%lu | nFAULT=%s\r\n",
                   HAL_GetTick(),
                   state_str,
                   (unsigned)comm_step,
                   commutation_delay_ms,
                   isr_tick,
                   nfault ? "OK" : "FAULT!");

            last_print = HAL_GetTick();
        }

        /* ----- TODO: BEMF zero-crossing detection hook -------------------- */
        /* Sample the floating phase ADC here (or in a comparator ISR) to
         * detect zero-crossings for sensorless closed-loop commutation.
         * When a zero-crossing is detected update last_comm_tick and call
         * apply_commutation_step() directly instead of waiting for the timer.
         */
    }
}

/* ===========================================================================
 * MX_TIM1_Init — center-aligned PWM @ 20 kHz switching, 40 kHz UEV/ISR
 *
 * RepetitionCounter = 0:  UEV fires every counter overflow (top + bottom).
 * The PWM outputs are enabled only after main() calls HAL_TIM_PWM_Start().
 * MOE is set after HAL_TIM_Base_Start_IT() to prevent it being overwritten.
 * =========================================================================*/
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef        sMasterConfig      = {0};
    TIM_OC_InitTypeDef             sConfigOC          = {0};
    TIM_BreakDeadTimeConfigTypeDef sBDT               = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = PWM_PERIOD;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;   /* KEY: UEV every overflow, not every 2nd */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;            /* start at 0 duty — phases Hi-Z    */
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();

    /* CCR preload — new values latch at UEV, preventing mid-cycle glitches */
    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 |=  TIM_CCMR2_OC3PE;
    TIM1->EGR    =  TIM_EGR_UG;   /* latch preload regs — also sets UIF       */
    /* UIF cleared in main() just before HAL_TIM_Base_Start_IT()              */

    sBDT.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBDT.OffStateIDLEMode = TIM_OSSI_DISABLE;   /* Hi-Z when idle — safe       */
    sBDT.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBDT.DeadTime         = DEAD_TIME_COUNTS;
    sBDT.BreakState       = TIM_BREAK_DISABLE;
    sBDT.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBDT.BreakFilter      = 0;
    sBDT.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBDT.Break2State      = TIM_BREAK2_DISABLE;
    sBDT.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBDT.Break2Filter     = 0;
    sBDT.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBDT.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBDT) != HAL_OK)
        Error_Handler();

    /* Explicitly enable TIM1 update interrupt in NVIC.
     * HAL_TIM_Base_Start_IT() also does this; being explicit documents intent.
     * Priority 0 = highest — keep the ISR lean if changing priority.        */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    HAL_TIM_MspPostInit(&htim1);
}

/* ===========================================================================
 * SystemClock_Config — 170 MHz via HSI + PLL
 * =========================================================================*/
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
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/* ===========================================================================
 * MX_GPIO_Init
 * =========================================================================*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC13 = nSLEEP — start LOW (DRV asleep) until main() wakes it */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC14 = nFAULT — input with internal pullup */
    GPIO_InitStruct.Pin  = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PB13/14/15 = INLA/B/C — GPIO output, start LOW (all phases Hi-Z).
     * Written LOW before HAL_GPIO_Init to guarantee no transient HIGH.
     * apply_commutation_step() will drive them HIGH/LOW per step.           */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ===========================================================================
 * MX_I2C1_Init
 * =========================================================================*/
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x40B285C2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks= I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * MX_USART3_UART_Init — 115200 8N1
 * =========================================================================*/
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
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * Error_Handler — make all phases Hi-Z then halt
 * =========================================================================*/
void Error_Handler(void)
{
    __disable_irq();

    /* Drive INHx to 0 and INLx to 0 → all phases Hi-Z */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    GPIOB->BSRR = (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U; /* reset */

    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert: %s line %u\r\n", file, (unsigned)line);
}
#endif
