#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	uint32_t counter = 0;

	printk("field_orient_g431 hello uart booted\n");

	while (1) {
		printk("hello from USART3, tick=%u\n", counter++);
		k_msleep(1000);
	}

	return 0;
}
