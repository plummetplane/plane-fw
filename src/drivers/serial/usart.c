#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "usart.h"

uint8_t recv_byte = '\0';

int usart_setup(uint32_t usart, uint32_t baud) {
	uint32_t rcc_usart, nvic_usart_irq, rcc_gpio_bank_usart, gpio_bank_usart;
	uint16_t gpio_usart_rx, gpio_usart_tx;
	switch (usart) {
		case USART1:
			rcc_usart = RCC_USART1;
			nvic_usart_irq = NVIC_USART1_IRQ;
			rcc_gpio_bank_usart = RCC_GPIOA;
			gpio_bank_usart = GPIO_BANK_USART1_RX;
			gpio_usart_rx = GPIO_USART1_RX;
			gpio_usart_tx = GPIO_USART1_TX;
			break;
		case USART2:
			rcc_usart = RCC_USART2;
			nvic_usart_irq = NVIC_USART2_IRQ;
			rcc_gpio_bank_usart = RCC_GPIOA;
			gpio_bank_usart = GPIO_BANK_USART2_RX;
			gpio_usart_rx = GPIO_USART2_RX;
			gpio_usart_tx = GPIO_USART2_TX;
			break;
		case USART3:
			rcc_usart = RCC_USART3;
			nvic_usart_irq = NVIC_USART3_IRQ;
			rcc_gpio_bank_usart = RCC_GPIOB;
			gpio_bank_usart = GPIO_BANK_USART3_RX;
			gpio_usart_rx = GPIO_USART3_RX;
			gpio_usart_tx = GPIO_USART3_TX;
			break;
		default:
			return -1;
			break;
	}

	/* gpio setup */
	rcc_periph_clock_enable(rcc_gpio_bank_usart);
	rcc_periph_clock_enable(RCC_AFIO);

	/* enable USART RX/TX */
	gpio_set_mode(gpio_bank_usart, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, gpio_usart_tx);
	gpio_set_mode(gpio_bank_usart, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, gpio_usart_rx);

	/* enable usart clock */
	rcc_periph_clock_enable(rcc_usart);

	/* enable USART interrupt */
	nvic_enable_irq(nvic_usart_irq);

	/* setup UART */
	usart_set_baudrate(usart, baud);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
	usart_set_mode(usart, USART_MODE_TX_RX);

	/* enable USART receive interrupt. */
	USART_CR1(usart) |= USART_CR1_RXNEIE;

	/* enable the USART */
	usart_enable(usart);

	return 0;
}

void usart1_isr(void) {
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) && ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
		recv_byte = usart_drv_recv(USART1);
	}
}

void usart2_isr(void) {
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) && ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
		recv_byte = usart_drv_recv(USART2);
	}
}

void usart3_isr(void) {
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) && ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
		recv_byte = usart_drv_recv(USART3);
	}
}

void usart_drv_str_write(uint32_t usart, char *data) {
	/* usart write string */
	uint32_t pos = 0;
	while (data[pos] != '\0') {
		usart_send_blocking(usart, data[pos]);
		pos++;
	}
}

void usart_drv_write(uint32_t usart, uint8_t data) {
	usart_send_blocking(usart, data);
}

char usart_drv_recv(uint32_t usart) {
	return usart_recv(usart);
}

int usart_init(uint32_t usart, uint32_t baud) {
	return usart_setup(usart, baud);
}
