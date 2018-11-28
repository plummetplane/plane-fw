#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>
#include <libopencm3/stm32/usart.h>

int usart_init(uint32_t usart, uint32_t baud);

void usart_drv_str_write(uint32_t usart, char *data);
char usart_drv_recv(uint32_t usart);

#endif
