#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "spi.h"

struct spi_gpio {
	uint32_t rcc_spi;
	uint32_t spi;
	uint32_t rcc_gpioport;
	uint32_t gpioport;
	uint16_t miso;
	uint16_t mosi;
	uint16_t sck;
	uint16_t nss;
};

struct spi_gpio spi_gpios[] =
{
	{.rcc_spi = RCC_SPI1, .spi = SPI1, .rcc_gpioport = RCC_GPIOA, .gpioport = GPIOA, .miso = GPIO_SPI1_MISO,
		.mosi = GPIO_SPI1_MOSI, .sck = GPIO_SPI1_SCK, .nss = GPIO_SPI1_NSS},
	{.rcc_spi = RCC_SPI2, .spi = SPI2, .rcc_gpioport = RCC_GPIOB, .gpioport = GPIOB, .miso = GPIO_SPI2_MISO,
		.mosi = GPIO_SPI2_MOSI, .sck = GPIO_SPI2_SCK, .nss = GPIO_SPI2_NSS},
	{.rcc_spi = RCC_SPI3, .spi = SPI3, .rcc_gpioport = RCC_GPIOB, .gpioport = GPIOB, .miso = GPIO_SPI3_MISO,
		.mosi = GPIO_SPI3_MOSI, .sck = GPIO_SPI3_SCK, .nss = GPIO_SPI3_NSS}
};

/*
 * Prepare SPI for use in master mode
 *
 * br - baud rate		SPI_CR1_BAUDRATE_FPCLK_DIV_256
 * cpol - clock polarity	SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE
 * cpha - clock phase		SPI_CR1_CPHA_CLK_TRANSITION_1
 * dff - data frame format	SPI_CR1_DFF_8BIT
 * lsbfirs - lsb/msb first	SPI_CR1_MSBFIRST
 */
void spi_setup(uint8_t spi, uint32_t br, uint32_t cpol, uint32_t cpha, uint32_t dff, uint32_t lsbfirst) {
	/* setup GPIO pins for SPI */
	rcc_periph_clock_enable(spi_gpios[spi].rcc_gpioport);
	rcc_periph_clock_enable(RCC_AFIO);

	gpio_set_mode(spi_gpios[spi].gpioport, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, spi_gpios[spi].sck | spi_gpios[spi].mosi);
	gpio_set_mode(spi_gpios[spi].gpioport, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, spi_gpios[spi].miso);
	gpio_set_mode(spi_gpios[spi].gpioport, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, spi_gpios[spi].nss);

	/* SPI clock */
	rcc_periph_clock_enable(spi_gpios[spi].rcc_spi);

	/* reset SPI */
	spi_reset(spi_gpios[spi].spi);

	/* setup spi in master mode */
	spi_init_master(spi_gpios[spi].spi, br, cpol, cpha, dff, lsbfirst);

	/* set nss management to software */
	spi_enable_software_slave_management(spi_gpios[spi].spi);
	spi_set_nss_high(spi_gpios[spi].spi);

	/* enable SPI */
	spi_enable(spi_gpios[spi].spi);
}

void spi_select(uint8_t spi) {
	gpio_clear(spi_gpios[spi].gpioport, spi_gpios[spi].nss);
}

void spi_deselect(uint8_t spi) {
	gpio_set(spi_gpios[spi].gpioport, spi_gpios[spi].nss);
}

void spi_release(uint8_t spi) {
	spi_deselect(spi);
	spi_drv_read(spi_gpios[spi].spi);
}

uint16_t spi_drv_read(uint8_t spi) {
	return spi_read(spi_gpios[spi].spi);
}

void spi_drv_write(uint8_t spi, uint16_t data) {
	spi_send(spi_gpios[spi].spi, data);
}

uint16_t spi_drv_xfer(uint8_t spi, uint16_t data) {
	return spi_xfer(spi_gpios[spi].spi, data);
}

int spi_init(uint8_t spi, uint32_t br, uint32_t cpol, uint32_t cpha, uint32_t dff, uint32_t lsbfirst) {
	spi_setup(spi, br, cpol, cpha, dff, lsbfirst);
	return 0;
}
