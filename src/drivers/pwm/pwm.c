#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "pwm.h"

/* setup GPIO */
int gpio_setup(uint32_t tim, enum tim_oc_id tim_channel) {
	uint32_t rcc_gpio_bank = 0, gpio_bank = 0, gpio = 0;
	switch (tim) {
		case TIM1:
			switch (tim_channel) {
				case TIM_OC1:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM1_CH1;
					gpio = GPIO_TIM1_CH1;
					break;
				case TIM_OC2:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM1_CH2;
					gpio = GPIO_TIM1_CH2;
					break;
				case TIM_OC3:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM1_CH3;
					gpio = GPIO_TIM1_CH3;
					break;
				case TIM_OC4:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM1_CH4;
					gpio = GPIO_TIM1_CH4;
					break;
			}
			break;
		case TIM2:
			switch (tim_channel) {
				case TIM_OC1:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM2_CH1_ETR;
					gpio = GPIO_TIM2_CH1_ETR;
					break;
				case TIM_OC2:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM2_CH2;
					gpio = GPIO_TIM2_CH2;
					break;
				case TIM_OC3:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM2_CH3;
					gpio = GPIO_TIM2_CH3;
					break;
				case TIM_OC4:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM2_CH4;
					gpio = GPIO_TIM2_CH4;
					break;
			}
			break;
		case TIM3:
			switch (tim_channel) {
				case TIM_OC1:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM3_CH1;
					gpio = GPIO_TIM3_CH1;
					break;
				case TIM_OC2:
					rcc_gpio_bank = RCC_GPIOA;
					gpio_bank = GPIO_BANK_TIM3_CH2;
					gpio = GPIO_TIM3_CH2;
					break;
				case TIM_OC3:
					rcc_gpio_bank = RCC_GPIOB;
					gpio_bank = GPIO_BANK_TIM3_CH3;
					gpio = GPIO_TIM3_CH3;
					break;
				case TIM_OC4:
					rcc_gpio_bank = RCC_GPIOB;
					gpio_bank = GPIO_BANK_TIM3_CH4;
					gpio = GPIO_TIM3_CH4;
					break;
			}
			break;
		case TIM4:
			switch (tim_channel) {
				case TIM_OC1:
					rcc_gpio_bank = RCC_GPIOB;
					gpio_bank = GPIO_BANK_TIM4_CH1;
					gpio = GPIO_TIM4_CH1;
					break;
				case TIM_OC2:
					rcc_gpio_bank = RCC_GPIOB;
					gpio_bank = GPIO_BANK_TIM4_CH2;
					gpio = GPIO_TIM4_CH2;
					break;
				case TIM_OC3:
					rcc_gpio_bank = RCC_GPIOB;
					gpio_bank = GPIO_BANK_TIM4_CH3;
					gpio = GPIO_TIM4_CH3;
					break;
				case TIM_OC4:
					rcc_gpio_bank = RCC_GPIOB;
					gpio_bank = GPIO_BANK_TIM4_CH4;
					gpio = GPIO_TIM4_CH4;
					break;
			}
			break;
	}

	if ((rcc_gpio_bank | gpio_bank | gpio) == 0)
		return -1;

	rcc_periph_clock_enable(rcc_gpio_bank);
	rcc_periph_clock_enable(RCC_AFIO);

	gpio_set_mode(gpio_bank, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, gpio);

	timer_disable_oc_output(tim, tim_channel);
	timer_set_oc_mode(tim, tim_channel, TIM_OCM_PWM1);
	timer_set_oc_value(tim, tim_channel, 0);
	timer_enable_oc_output(tim, tim_channel);

	return 0;
}

/* setup timer for generating pwm signal */
int tim_setup(uint32_t tim, uint32_t prescaler, uint32_t period) {
	enum rcc_periph_clken rcc_tim;
	enum rcc_periph_rst rst_tim;
	switch (tim) {
		case TIM1:
			rcc_tim = RCC_TIM1;
			rst_tim = RST_TIM1;
			break;
		case TIM2:
			rcc_tim = RCC_TIM2;
			rst_tim = RST_TIM2;
			break;
		case TIM3:
			rcc_tim = RCC_TIM3;
			rst_tim = RST_TIM3;
			break;
		case TIM4:
			rcc_tim = RCC_TIM4;
			rst_tim = RST_TIM4;
			break;
		default:
			return -1;
			break;
	}

	rcc_periph_clock_enable(rcc_tim);

	rcc_periph_reset_pulse(rst_tim);
	timer_set_mode(tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(tim, prescaler);
	timer_continuous_mode(tim);
	timer_set_period(tim, period);
	timer_enable_preload(tim);
	timer_enable_counter(tim);

	return 0;
}

void pwm_set_duty(uint32_t tim, enum tim_oc_id tim_channel, uint32_t val) {
	timer_set_oc_value(tim, tim_channel, val);
}

int pwm_tim_setup(uint32_t tim, uint32_t prescaler, uint32_t period) {
	return tim_setup(tim, prescaler, period);
}

int pwm_pin_setup(uint32_t tim, enum tim_oc_id tim_channel) {
	return gpio_setup(tim, tim_channel);
}

int pwm_init(void) {
	return 0;
}
