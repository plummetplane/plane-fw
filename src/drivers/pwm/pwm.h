#ifndef __PWM_H__
#define __PWM_H__

#include <stdint.h>

#include <libopencm3/stm32/timer.h>

int pwm_init(void);

int pwm_tim_setup(uint32_t tim, uint32_t prescaler, uint32_t period);

int pwm_pin_setup(uint32_t tim, enum tim_oc_id tim_channel);

void pwm_set_duty(uint32_t tim, enum tim_oc_id tim_channel, uint32_t val);

#endif
