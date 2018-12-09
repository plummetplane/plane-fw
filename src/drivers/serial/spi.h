#ifndef __SPI_H__
#define __SPI_H__

#include <libopencm3/stm32/spi.h>

int spi_init(void);

uint16_t spi_drv_read(uint32_t spi);

void spi_drv_write(uint32_t spi, uint16_t data);

uint16_t spi_drv_xfer(uint32_t spi, uint16_t data);

#endif
