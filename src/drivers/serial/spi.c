#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "spi.h"

void spi_setup(void) {
	/* setup GPIO pins for SPI */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 | GPIO5 | GPIO7);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);

	/* SPI clock */
	rcc_periph_clock_enable(RCC_SPI1);

	/* reset SPI */
	spi_reset(SPI1);

	/* setup spi in master mode
	 * clock baud rate: 1/64 of periph clock
	 * clock polarity: idle high
	 * clock phase: data valid on 2nd pulse
	gpio_setup();
	 * data frame format: 8-bit
	 * frame format: MSB first
	 */
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1,
			SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

	/* set nss management to software */
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	/* enable SPI */
	spi_enable(SPI1);
}

void spi_select(void) {
	gpio_clear(GPIOA, GPIO4);
}

void spi_deselect(void) {
	gpio_set(GPIOA, GPIO4);
}

void spi_release(void) {
	spi_deselect();
	spi_drv_read(SPI1);
}

uint16_t spi_drv_read(uint32_t spi) {
	return spi_read(spi);
}

void spi_drv_write(uint32_t spi, uint16_t data) {
	spi_send(spi, data);
}

uint16_t spi_drv_xfer(uint32_t spi, uint16_t data) {
	return spi_xfer(spi, data);
}

int spi_init(void) {
	spi_setup();
	return 0;
}
