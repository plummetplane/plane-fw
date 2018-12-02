#ifndef __I2C_H__
#define __I2C_H__

#define I2C_BUF_SIZE 64

extern volatile uint8_t i2c1_data[I2C_BUF_SIZE];

int i2c_init(void);

#endif
