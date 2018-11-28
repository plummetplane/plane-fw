#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>
#include <libopencm3/stm32/usart.h>

#define BUFFER_SIZE 64

int usart_init(uint32_t usart, uint32_t baud);

void usart_drv_write(uint32_t usart, uint8_t data);
void usart_drv_str_write(uint32_t usart, uint8_t *data);
uint8_t usart_drv_recv(uint32_t usart);

int usart_set_callback(uint32_t usart, void *callback_func);

#endif
