#ifndef __I2C_H__
#define __I2C_H__

#define I2C_BUF_SIZE 64

#include <libopencm3/stm32/i2c.h>

int i2c_init_slave(uint8_t i2c, uint8_t address);

int i2c_init_master(uint8_t i2c);

void i2c_set_callback(uint8_t i2c, void (*callback)(uint8_t *buf, uint32_t buf_size));

void i2c_transfer(uint8_t i2c, uint8_t addr, uint8_t *w, uint32_t wn, uint8_t *r, uint32_t rn);

#endif
