#ifndef __PWM_H__
#define __PWM_H__

#include <stdint.h>

int pwm_init(void);

void pwm_set_duty_cycle(uint32_t val);

#endif
