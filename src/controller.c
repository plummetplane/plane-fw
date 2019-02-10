#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "ppm-decode/ppm-readval.h"
#include "drivers/wireless/nrf24l01.h"

#include "command/cmds.h"

uint32_t trans_fail = 0;

void nrf_tx_fail(void);

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
	timer_set_period(TIM3, 0xffff);
	timer_set_oc_value(TIM3, TIM_OC1, 65535);
	timer_enable_counter(TIM3);
	timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

static void nrf24l01_setup(void) {
	/* init nrf module interface */
	nrf24l01_init(0, GPIOB, GPIO0, NRF24L01_MODE_TX);

	nrf24l01_maxrt_callback(&nrf_tx_fail);

	/* irq pin */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

	nvic_enable_irq(NVIC_EXTI0_IRQ);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1);

	exti_select_source(EXTI0, GPIOB);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);
}

void tim3_isr(void) {
	if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
		timer_clear_flag(TIM3, TIM_SR_CC1IF);

		nrf24l01_payload payload = {
			.size = 7,
			.data = {
				CMD_SERVO_CONTROL,
				channel_ppm[0],
				channel_ppm[1],
				channel_ppm[2],
				channel_ppm[3],
				channel_ppm[4],
				channel_ppm[5]
			}
		};

		nrf24l01_transmit(&payload);
	}
}

void exti0_isr(void) {
	nrf24l01_irq();
	exti_reset_request(EXTI0);
}

void nrf_tx_fail(void) {
	trans_fail++;
}

void nrf24l01_read_reg(uint8_t reg, uint8_t *buf);
void nrf24l01_write_reg(uint8_t reg, uint8_t *buf);

int main(void) {
	clock_setup();
	nrf24l01_setup();
	tim_setup();

	while(1)
		__asm("wfi");

	return 0;
}
