#ifndef __I2C_H__
#define __I2C_H__

#define I2C_BUF_SIZE 64

#include <libopencm3/stm32/i2c.h>

extern volatile uint8_t i2c1_data[I2C_BUF_SIZE];
extern volatile uint8_t i2c2_data[I2C_BUF_SIZE];

int i2c_init_slave(uint32_t i2c, uint8_t address);

int i2c_init_master(uint32_t i2c);

void i2c_set_callback(uint32_t i2c, void (*callback)(uint8_t *buf, uint32_t buf_size));

#endif
