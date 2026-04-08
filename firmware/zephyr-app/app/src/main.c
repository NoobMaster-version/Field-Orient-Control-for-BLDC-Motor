#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define APP_NODE DT_PATH(zephyr_user)
#define AS5600_NODE DT_ALIAS(as5600)

#define DRV_NSLEEP_SPEC GPIO_DT_SPEC_GET(APP_NODE, drv_nsleep_gpios)
#define DRV_NFAULT_SPEC GPIO_DT_SPEC_GET(APP_NODE, drv_nfault_gpios)
#define DRV_INLA_SPEC GPIO_DT_SPEC_GET(APP_NODE, drv_inla_gpios)
#define DRV_INLB_SPEC GPIO_DT_SPEC_GET(APP_NODE, drv_inlb_gpios)
#define DRV_INLC_SPEC GPIO_DT_SPEC_GET(APP_NODE, drv_inlc_gpios)

#if CONFIG_APP_STAGE_PWM_TEST
#define PHASE_PWM_A_SPEC PWM_DT_SPEC_GET(DT_ALIAS(phase_pwm_a))
#define PHASE_PWM_B_SPEC PWM_DT_SPEC_GET(DT_ALIAS(phase_pwm_b))
#define PHASE_PWM_C_SPEC PWM_DT_SPEC_GET(DT_ALIAS(phase_pwm_c))
#endif

#define AS5600_RAW_ANGLE_REG 0x0C
#define AS5600_STATUS_REG    0x0B

static const struct gpio_dt_spec drv_nsleep = DRV_NSLEEP_SPEC;
static const struct gpio_dt_spec drv_nfault = DRV_NFAULT_SPEC;
static const struct gpio_dt_spec drv_inla = DRV_INLA_SPEC;
static const struct gpio_dt_spec drv_inlb = DRV_INLB_SPEC;
static const struct gpio_dt_spec drv_inlc = DRV_INLC_SPEC;

#if CONFIG_APP_STAGE_PWM_TEST
static const struct pwm_dt_spec pwm_a = PHASE_PWM_A_SPEC;
static const struct pwm_dt_spec pwm_b = PHASE_PWM_B_SPEC;
static const struct pwm_dt_spec pwm_c = PHASE_PWM_C_SPEC;
#endif

#if CONFIG_APP_STAGE_I2C_AS5600
static const struct i2c_dt_spec as5600 = I2C_DT_SPEC_GET(AS5600_NODE);
#endif

#if CONFIG_APP_STAGE_ADC_TEST
static const struct device *adc1_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *adc2_dev = DEVICE_DT_GET(DT_NODELABEL(adc2));

static const struct adc_channel_cfg adc1_ch1_cfg = {
	.gain = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 1,
};

static const struct adc_channel_cfg adc1_ch3_cfg = {
	.gain = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 3,
};

static const struct adc_channel_cfg adc2_ch2_cfg = {
	.gain = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 2,
};
#endif

static int configure_drv_gpio(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags)
{
	if (!device_is_ready(spec->port)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE | extra_flags);
}

static int init_drv_pins(void)
{
	int ret;

	ret = configure_drv_gpio(&drv_nsleep, 0);
	if (ret < 0) {
		return ret;
	}

	ret = configure_drv_gpio(&drv_inla, 0);
	if (ret < 0) {
		return ret;
	}

	ret = configure_drv_gpio(&drv_inlb, 0);
	if (ret < 0) {
		return ret;
	}

	ret = configure_drv_gpio(&drv_inlc, 0);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(drv_nfault.port)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&drv_nfault, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static void print_drv_state(void)
{
	int fault = gpio_pin_get_dt(&drv_nfault);

	printk("DRV state: nSLEEP=%d nFAULT=%d INLA=%d INLB=%d INLC=%d\n",
	       gpio_pin_get_dt(&drv_nsleep),
	       fault,
	       gpio_pin_get_dt(&drv_inla),
	       gpio_pin_get_dt(&drv_inlb),
	       gpio_pin_get_dt(&drv_inlc));
}

static int enable_drv_outputs(void)
{
	int ret;

	ret = gpio_pin_set_dt(&drv_inla, 1);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&drv_inlb, 1);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&drv_inlc, 1);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_set_dt(&drv_nsleep, 1);
}

#if CONFIG_APP_STAGE_I2C_AS5600
static int read_as5600(uint16_t *raw_angle, uint8_t *status)
{
	int ret;
	uint8_t raw_buf[2];

	ret = i2c_reg_read_byte_dt(&as5600, AS5600_STATUS_REG, status);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_burst_read_dt(&as5600, AS5600_RAW_ANGLE_REG, raw_buf, sizeof(raw_buf));
	if (ret < 0) {
		return ret;
	}

	*raw_angle = ((((uint16_t)raw_buf[0]) << 8) | raw_buf[1]) & 0x0fff;
	return 0;
}
#endif

#if CONFIG_APP_STAGE_PWM_TEST
static int set_phase_pwms(uint32_t pulse_usec)
{
	int ret;

	ret = pwm_set_pulse_dt(&pwm_a, pulse_usec * 1000U);
	if (ret < 0) {
		return ret;
	}

	ret = pwm_set_pulse_dt(&pwm_b, pulse_usec * 1000U);
	if (ret < 0) {
		return ret;
	}

	return pwm_set_pulse_dt(&pwm_c, pulse_usec * 1000U);
}
#endif

#if CONFIG_APP_STAGE_ADC_TEST
static int setup_adc_channel(const struct device *dev, const struct adc_channel_cfg *cfg)
{
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return adc_channel_setup(dev, cfg);
}

static int read_adc_raw(const struct device *dev, uint8_t channel, int16_t *value)
{
	struct adc_sequence sequence = {
		.channels = BIT(channel),
		.buffer = value,
		.buffer_size = sizeof(*value),
		.resolution = 12,
	};

	return adc_read(dev, &sequence);
}
#endif

int main(void)
{
	int ret;

	printk("field_orient_g431 bring-up start\n");

#if CONFIG_APP_STAGE_DRV_GPIO
	ret = init_drv_pins();
	if (ret < 0) {
		printk("DRV GPIO init failed: %d\n", ret);
		return 0;
	}

	ret = enable_drv_outputs();
	if (ret < 0) {
		printk("DRV GPIO enable failed: %d\n", ret);
		return 0;
	}

	print_drv_state();
#endif

#if CONFIG_APP_STAGE_PWM_TEST
	if (!device_is_ready(pwm_a.dev) || !device_is_ready(pwm_b.dev) || !device_is_ready(pwm_c.dev)) {
		printk("PWM device not ready\n");
		return 0;
	}

	ret = set_phase_pwms((CONFIG_APP_PWM_TEST_PERIOD_USEC * CONFIG_APP_PWM_TEST_DUTY_PERCENT) / 100U);
	if (ret < 0) {
		printk("PWM setup failed: %d\n", ret);
		return 0;
	}

	printk("PWM active: period=%u us duty=%u%%\n",
	       CONFIG_APP_PWM_TEST_PERIOD_USEC,
	       CONFIG_APP_PWM_TEST_DUTY_PERCENT);
#endif

#if CONFIG_APP_STAGE_ADC_TEST
	ret = setup_adc_channel(adc1_dev, &adc1_ch1_cfg);
	if (ret < 0) {
		printk("ADC A setup failed: %d\n", ret);
		return 0;
	}

	ret = setup_adc_channel(adc2_dev, &adc2_ch2_cfg);
	if (ret < 0) {
		printk("ADC B setup failed: %d\n", ret);
		return 0;
	}

	ret = setup_adc_channel(adc1_dev, &adc1_ch3_cfg);
	if (ret < 0) {
		printk("ADC C setup failed: %d\n", ret);
		return 0;
	}
#endif

	while (1) {
#if CONFIG_APP_STAGE_DRV_GPIO
		print_drv_state();
#endif

#if CONFIG_APP_STAGE_I2C_AS5600
		if (!device_is_ready(as5600.bus)) {
			printk("AS5600 I2C bus not ready\n");
		} else {
			uint16_t raw_angle;
			uint8_t status;

			ret = read_as5600(&raw_angle, &status);
			if (ret < 0) {
				printk("AS5600 read failed: %d\n", ret);
			} else {
				printk("AS5600 status=0x%02x raw=%u deg=%u\n",
				       status,
				       raw_angle,
				       (raw_angle * 360U) / 4096U);
			}
		}
#endif

#if CONFIG_APP_STAGE_ADC_TEST
		{
			int16_t raw_a = 0;
			int16_t raw_b = 0;
			int16_t raw_c = 0;

			ret = read_adc_raw(adc1_dev, 1, &raw_a);
			if (ret < 0) {
				printk("ADC A read failed: %d\n", ret);
			}

			ret = read_adc_raw(adc2_dev, 2, &raw_b);
			if (ret < 0) {
				printk("ADC B read failed: %d\n", ret);
			}

			ret = read_adc_raw(adc1_dev, 3, &raw_c);
			if (ret < 0) {
				printk("ADC C read failed: %d\n", ret);
			}

			printk("ADC currents raw: A=%d B=%d C=%d\n", raw_a, raw_b, raw_c);
		}
#endif

		k_msleep(MAX(CONFIG_APP_HEARTBEAT_PERIOD_MS,
			     MAX(CONFIG_APP_AS5600_POLL_PERIOD_MS,
				 CONFIG_APP_ADC_POLL_PERIOD_MS)));
	}

	return 0;
}
