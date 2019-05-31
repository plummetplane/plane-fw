#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "drivers/misc/bmp280-i2c.h"
#include "drivers/pwm/pwm.h"
#include "drivers/wireless/nrf24l01.h"

#include "command/cmds.h"

/*
 * TODO: telemetry request handler
 */

#define BMP_I2C 0
#define BMP_SAMPLE_COUNT 32

uint32_t pres_hist[32];
uint8_t pres_pos = 0;
uint64_t pres_sum = 0;

void nrf_recv_callback(nrf24l01_payload payload);

static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

static void tim_setup(void) {
	rcc_periph_clock_enable(RCC_TIM3);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	rcc_periph_reset_pulse(RST_TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM3, (rcc_apb1_frequency * 2));
	timer_disable_preload(TIM3);
	timer_continuous_mode(TIM3);
	timer_set_period(TIM3, 0xff);
	timer_set_oc_value(TIM3, TIM_OC1, 65535);
	timer_enable_counter(TIM3);
	timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

static void pwm_setup(void) {
	pwm_tim_setup(TIM4, 72, 20000);
	pwm_pin_setup(TIM4, TIM_OC4);
	pwm_pin_setup(TIM4, TIM_OC3);
	pwm_set_duty(TIM4, TIM_OC4, 1500);
	pwm_set_duty(TIM4, TIM_OC3, 1500);
}

static void nrf24l01_setup(void) {
	nrf24l01_init(0, GPIOB, GPIO0, NRF24L01_MODE_RX);
	nrf24l01_rxdr_callback(&nrf_recv_callback);

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

	nvic_enable_irq(NVIC_EXTI0_IRQ);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1);

	exti_select_source(EXTI0, GPIOB);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);
}

static void bmp_setup(void) {
	bmp280_i2c_init(BMP_I2C);

	for (uint32_t i = 0; i < BMP_SAMPLE_COUNT; i++) {
		pres_hist[i] = bmp280_i2c_pres(0);
		pres_sum += pres_hist[i];
	}
}

static void telemetry_transmit(void) {
	nrf24l01_mode(NRF24L01_MODE_TX);

	/* bit-shift - faster then division */
	uint32_t pres = pres_sum >> 5;

	nrf24l01_payload telemetry;

	for (uint8_t i = 0; i < 4; i++) {
		telemetry.data[i] = (uint8_t)((pres >> ((3 - i) * 8)) & 0xff);
	}

	telemetry.size = 4;

	nrf24l01_transmit(&telemetry);
}

void servo_pwm_duty(uint8_t *ppm) {
	pwm_set_duty(TIM4, TIM_OC4, ppm[0]);
	pwm_set_duty(TIM4, TIM_OC3, ppm[1]);
}

void nrf_recv_callback(nrf24l01_payload payload) {
	switch (payload.data[0]) {
		case CMD_SERVO_CONTROL:
			servo_pwm_duty(&payload.data[1]);
			break;
		case CMD_REQ_TELEMETRY:
			telemetry_transmit();
			break;
	}
}

void tim3_isr(void) {
	if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
		timer_clear_flag(TIM3, TIM_SR_CC1IF);

		pres_sum -= pres_hist[pres_pos];
		pres_hist[pres_pos] = bmp280_i2c_pres(BMP_I2C);
		pres_sum += pres_hist[pres_pos];

		pres_pos++;

		if (pres_pos >= BMP_SAMPLE_COUNT)
			pres_pos = 0;
	}
}

void exti0_isr(void) {
	nrf24l01_irq();
	exti_reset_request(EXTI0);
}

int main(void) {
	clock_setup();
	nrf24l01_setup();

	while(1)
		__asm("wfi");

	return 0;
}
