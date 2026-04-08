# Zephyr Bring-Up for `field_orient_g431`

This directory adds an out-of-tree Zephyr board and a staged bring-up app for your
custom board:

- MCU: `STM32G431CBT6`
- Gate driver: `DRV8311H`
- Encoder: `AS5600` on `I2C1`

## Board mapping

- `USART3` console: `PB10` TX, `PB11` RX
- `I2C1` for `AS5600`: `PA15` SCL, `PB7` SDA
- `TIM1` PWM: `PA8`, `PA9`, `PA10`
- Current sense ADCs: `PA0`, `PA1`, `PA2`
- `DRV8311H` control: `PC13` `nSLEEP`, `PB13/PB14/PB15` low-side control pins
- `DRV8311H` fault: `PC14` `nFAULT`

## Laptop setup

The Zephyr workspace is initialized at:

```bash
/home/raze/zephyrproject
```

Activate the Python environment:

```bash
source /home/raze/zephyrproject/.venv/bin/activate
```

Install Zephyr Python requirements:

```bash
cd /home/raze/zephyrproject/zephyr
pip install -r scripts/requirements.txt
```

Install the Zephyr SDK if it is not already present. A typical local install is:

```bash
cd /home/raze/zephyrproject
west sdk install
```

Export Zephyr into your shell:

```bash
cd /home/raze/zephyrproject/zephyr
west zephyr-export
```

## Build commands

Set:

```bash
export ZEPHYR_BASE=/home/raze/zephyrproject/zephyr
```

## Simplest UART hello-world

If you want the smallest possible flash test first, build the UART-only app:

```bash
west build -b field_orient_g431 \
  /home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app/hello_uart \
  --build-dir /tmp/field_orient_g431-hello-build \
  -- \
  -DBOARD_ROOT=/home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app
```

That app only prints once per second on `USART3` (`PB10` TX, `PB11` RX):

```text
field_orient_g431 hello uart booted
hello from USART3, tick=0
hello from USART3, tick=1
...
```

Build the base console and DRV GPIO bring-up:

```bash
west build -b field_orient_g431 \
  /home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app/app \
  --build-dir /tmp/field_orient_g431-build \
  -- \
  -DBOARD_ROOT=/home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app
```

Build AS5600 bring-up:

```bash
west build -b field_orient_g431 \
  /home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app/app \
  --build-dir /tmp/field_orient_g431-build \
  -- \
  -DBOARD_ROOT=/home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app \
  -DEXTRA_CONF_FILE=i2c_as5600.conf
```

Build PWM test:

```bash
west build -b field_orient_g431 \
  /home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app/app \
  --build-dir /tmp/field_orient_g431-build \
  -- \
  -DBOARD_ROOT=/home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app \
  -DEXTRA_CONF_FILE=pwm_test.conf
```

Build ADC test:

```bash
west build -b field_orient_g431 \
  /home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app/app \
  --build-dir /tmp/field_orient_g431-build \
  -- \
  -DBOARD_ROOT=/home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app \
  -DEXTRA_CONF_FILE=adc_test.conf
```

Build everything enabled:

```bash
west build -b field_orient_g431 \
  /home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app/app \
  --build-dir /tmp/field_orient_g431-build \
  -- \
  -DBOARD_ROOT=/home/raze/workspace/Field-Orient-Control-for-BLDC-Motor/firmware/zephyr-app \
  -DEXTRA_CONF_FILE=full_bringup.conf
```

## Flash and debug

If `STM32CubeProgrammer` is installed:

```bash
west flash --runner stm32cubeprogrammer --build-dir /tmp/field_orient_g431-build
```

If you prefer OpenOCD with an ST-LINK:

```bash
west flash --runner openocd --build-dir /tmp/field_orient_g431-build
```

## Important current limitations

- This is a Phase 1 board bring-up target, not the FOC port yet.
- The PWM test only proves basic Zephyr PWM output. It does not yet configure the
  full center-aligned motor-control timing model from your Cube firmware.
- The ADC test reads channels in software. It does not yet use timer-synchronous
  sampling.
- After this builds and runs, the next step is a dedicated DRV8311 helper plus an
  AS5600 module, then the open-loop validation flow.
