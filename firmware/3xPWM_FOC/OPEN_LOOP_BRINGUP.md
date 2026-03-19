# STM32G431 + DRV8311H Bring-Up

## CubeMX setup

- `RCC`: use `HSI` with PLL to `170 MHz`.
- `TIM1`: center-aligned PWM, `Prescaler = 0`, `Period = 1699`, CH1/CH2/CH3 enabled on `PA8/PA9/PA10`, `TRGO = Update`.
- `ADC1`: regular conversion sequence with `PA0 = IN1` and `PA2 = IN3`, external trigger `TIM1 TRGO`, DMA enabled, 12-bit, sample time `47.5 cycles`.
- `ADC2`: single conversion on `PA1 = IN2`, software trigger, 12-bit, sample time `47.5 cycles`.
- `I2C1`: `PA15 = SCL`, `PB7 = SDA`, timing value `0x40B285C2`.
- `USART3`: `PB10 = TX`, `PB11 = RX`, `115200 8N1`.
- `GPIO output`: `PC13 = nSLEEP`, `PB13/PB14/PB15 = INLA/INLB/INLC`.
- `GPIO input`: `PC14 = nFAULT` with pull-up.

## Bring-up sequence

1. Keep `nSLEEP` low during reset.
2. Start TIM1 PWM with all three duties at `50%`.
3. Drive `PB13/PB14/PB15` high if your DRV8311H mode really is `3x PWM`.
4. Calibrate ADC offsets with zero commanded voltage.
5. Raise `nSLEEP`, check `nFAULT`, and stop immediately if a fault is active.
6. Apply a fixed electrical angle for about `1.2 s` to align the rotor.
7. Ramp electrical speed slowly to about `1.2 electrical Hz`.
8. Print phase currents and AS5600 angle over UART at `10 Hz`.

## Required hardware checks

- Do not leave `MODE` floating on the final board. Strap it to the mode you actually want after checking the DRV8311H truth table.
- Confirm your shunt resistor value and update `SHUNT_RESISTOR_OHMS` in [main.c](/home/raze/workspace/stm/3xPWM_FOC/Core/Src/main.c).
- Confirm the DRV current-sense output common-mode behavior matches the code assumption of a midscale zero-current offset.
- If the motor twitches but does not rotate, swap any two motor phases or invert the open-loop angle direction.
- For a `4300 KV` 1S motor, keep startup modulation low. Your current code clamps normal run modulation to `4.5%`.

## What to implement next

1. Encoder validation: verify AS5600 angle is monotonic through one full mechanical revolution.
2. Electrical angle calibration: lock rotor at a known phase vector and measure encoder offset.
3. Closed-loop current sensing: move from slow debug reads to timer-synchronous sampling for all phases.
4. FOC transforms: Clarke, Park, inverse Park, and SVPWM or sinusoidal PWM.
5. Current PI loops: `Id` and `Iq`.
6. Velocity loop: use AS5600 derived speed as the outer loop.
