#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "ppm-decode/ppm-readval.h"
#include "drivers/pwm/pwm.h"
#include "drivers/serial/spi.h"
#include "drivers/serial/usart.h"

void usart_callback(uint8_t data);

static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

static void gpio_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

static void pwm_setup(void) {
	pwm_tim_setup(TIM3, 72, 20000);
	pwm_pin_setup(TIM3, TIM_OC1);
	pwm_set_duty(TIM3, TIM_OC1, 1500);
}

static void usart_setup(void) {
	usart_init(USART1, 115200);
	usart_set_callback(USART1, &usart_callback);
}

static void spi_setup(void) {
	spi_init();
}

void usart_callback(uint8_t data) {
	usart_drv_write(USART1, data);
}

int main(void) {
	clock_setup();
	gpio_setup();
	pwm_setup();
	ppm_readval_init();
	usart_setup();
	spi_setup();

	gpio_set(GPIOC, GPIO13);

	long long int i;

	/* blink the onboard led to indicate it is working */
	while(1) {
		gpio_toggle(GPIOC, GPIO13);
		for (i = 0; i < 720000; i++)
			__asm__("nop");
		pwm_set_duty(TIM3, TIM_OC1, channel_ppm[2]);
		//usart_drv_str_write(USART1, "it works!");
	}
	return 0;
}
