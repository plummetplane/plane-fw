#ifndef __BMP280_I2C_H__
#define __BMP280_I2C_H__

#define BMP280_DATA_SIZE 6 /* bytes */

void bmp280_i2c_init(uint8_t i2c);

void bmp280_i2c_reset(uint8_t i2c);

int32_t bmp280_i2c_temp(uint8_t i2c);

uint32_t bmp280_i2c_pres(uint8_t i2c);

void bmp280_i2c_get(uint8_t i2c, int32_t *temp, uint32_t *pres);

#endif /* __BMP280_I2C_H__ */
