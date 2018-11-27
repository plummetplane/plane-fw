#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "pwm.h"

/* setup GPIO */
void gpio_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);

	gpio_set_mode(GPIO_BANK_TIM3_CH1, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM3_CH1);

	timer_disable_oc_output(TIM3, TIM_OC1);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_value(TIM3, TIM_OC1, 1500);
	timer_enable_oc_output(TIM3, TIM_OC1);
}

/* setup timer for generating pwm signal */
void timer_setup(void) {
	rcc_periph_clock_enable(RCC_TIM3);

	rcc_periph_reset_pulse(RST_TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM3, 72);
	timer_continuous_mode(TIM3);
	timer_set_period(TIM3, 20000);
	timer_enable_preload(TIM3);
	timer_enable_counter(TIM3);
}

void pwm_set_duty_cycle(uint32_t val) {
	timer_set_oc_value(TIM3, TIM_OC1, val);
}

int pwm_init(void) {
	timer_setup();
	gpio_setup();
	return 0;
}
