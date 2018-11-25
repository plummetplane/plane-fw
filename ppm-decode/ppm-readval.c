#include <stdint.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "ppm-readval.h"

uint32_t channel_ppm[7] = { 0 };
uint8_t channel = 0;

static void tim_init(void) {
	/* enable TIM2 clock */
	rcc_periph_clock_enable(RCC_TIM2);

	/* gpio config */
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

	/* timer config */
	rcc_periph_reset_pulse(RST_TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM2, 0xFFFF);
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 1000000));

	/* setup input capture */
	timer_ic_set_polarity(TIM2, TIM_IC2, TIM_IC_RISING);
	timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI1);
	timer_ic_set_prescaler(TIM2, TIM_IC2, TIM_IC_PSC_OFF);
	timer_ic_set_filter(TIM2, TIM_IC2, TIM_IC_OFF);

	/* setup interrupt */
	nvic_set_priority(NVIC_TIM2_IRQ, 2);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	
	timer_enable_irq(TIM2, (TIM_DIER_CC2IE | TIM_DIER_UIE));
	timer_ic_enable(TIM2, TIM_IC2);

	timer_enable_counter(TIM2);
}

void tim2_isr(void) {
	if ((TIM2_SR & TIM_SR_CC2IF) != 0) {
		/* save the time period between pulses
		 * and reset counter to 0
		 *
		 * important thing to do is to align the channels
		 */
		timer_clear_flag(TIM2, TIM_SR_CC2IF);
		channel_ppm[channel] = timer_get_counter(TIM2);
		timer_set_counter(TIM2, 0);
		if (channel_ppm[channel] > 8000 || channel >= 7) {
			channel = 0;
		} else {
			channel++;
		}
	} else if ((TIM2_SR & TIM_SR_UIF) != 0) {
		timer_clear_flag(TIM2, TIM_SR_UIF);
	}
}

int ppm_readval_init(void) {
	tim_init();
	return 0;
}
