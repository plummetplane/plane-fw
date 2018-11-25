#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "ppm-decode/ppm-readval.h"

static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

static void gpio_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int main(void) {
	clock_setup();
	gpio_setup();
	ppm_readval_init();

	gpio_set(GPIOC, GPIO13);

	long long int i;

	/* blink the onboard led to indicate it is working */
	while(1) {
		gpio_toggle(GPIOC, GPIO13);
		for (i = 0; i < 7200000; i++)
			__asm__("nop");
	}
	return 0;
}
