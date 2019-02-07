#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "ppm-decode/ppm-readval.h"
#include "drivers/misc/bmp280-i2c.h"
#include "drivers/pwm/pwm.h"
#include "drivers/serial/i2c.h"
#include "drivers/serial/spi.h"
#include "drivers/serial/usart.h"
#include "drivers/wireless/nrf24l01.h"

void usart_callback(uint8_t data);

void i2c_callback(uint8_t *buf, uint32_t size);

static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

static void gpio_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

static void pwm_setup(void) {
	pwm_tim_setup(TIM4, 72, 20000);
	pwm_pin_setup(TIM4, TIM_OC4);
	pwm_set_duty(TIM4, TIM_OC4, 1500);
}

static void usart_setup(void) {
	usart_init(USART1, 115200);
	usart_set_callback(USART1, &usart_callback);
}

static void i2c_setup(void) {
	i2c_init_slave(0, 0x27);
	i2c_set_callback(0, &i2c_callback);
}

static void nrf24l01_setup(void) {
	nrf24l01_init(0, GPIOB, GPIO0, NRF24L01_MODE_TX);
}

static void bmp280_i2c_setup(void) {
	bmp280_i2c_init(0);
}

void usart_callback(uint8_t data) {
	usart_drv_write(USART1, data);
}

void i2c_callback(uint8_t *buf, uint32_t size) {
	for (unsigned int i = 0; i < size; i++) {
		buf[i]++;
	}
}

int main(void) {
	clock_setup();
	gpio_setup();
	pwm_setup();
	ppm_readval_init();
	usart_setup();
	i2c_setup();
	nrf24l01_setup();
	bmp280_i2c_setup();

	gpio_set(GPIOC, GPIO13);

	long long int i;

	/* blink the onboard led to indicate it is working */
	while(1) {
		gpio_toggle(GPIOC, GPIO13);
		for (i = 0; i < 720000; i++)
			__asm__("nop");
		pwm_set_duty(TIM4, TIM_OC4, channel_ppm[2]);
		//usart_drv_str_write(USART1, "it works!");
	}
	return 0;
}
