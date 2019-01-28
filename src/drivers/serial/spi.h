#ifndef __SPI_H__
#define __SPI_H__

#include <libopencm3/stm32/spi.h>

int spi_init(uint8_t spi, uint32_t br, uint32_t cpol, uint32_t cpha, uint32_t dff, uint32_t lsbfirst);
void spi_select(uint8_t spi);
void spi_deselect(uint8_t spi);
void spi_release(uint8_t spi);
uint16_t spi_drv_read(uint8_t spi);
void spi_drv_write(uint8_t spi, uint16_t data);
uint16_t spi_drv_xfer(uint8_t spi, uint16_t data);

#endif
