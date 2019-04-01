#ifndef __BMP180_I2C_H__
#define __BMP180_I2C_H__

void bmp180_i2c_init(uint8_t i2c);

int32_t bmp180_i2c_temp(uint8_t i2c);

uint32_t bmp180_i2c_pres(uint8_t i2c);

void bmp180_i2c_get(uint8_t i2c, int32_t *temp, uint32_t *pres);

#endif /* __BMP180_I2C_H__ */
