#include <stdint.h>

#include "../serial/i2c.h"

#include "bmp180-i2c.h"

/* device i2c address */
#define BMP180_I2C_ADDR 0xee // FIXME: or 0x77?? (datasheet p. 20)

/* registers */
#define BMP180_REG_ID 0xd0
#define BMP180_REG_SOFT 0xe0
#define BMP180_REG_CTRL_MEAS 0xf4
#define BMP180_REG_OUT_MSB 0xf6
#define BMP180_REG_OUT_LSB 0xf7
#define BMP180_REG_OUT_XLSB 0xf8
#define BMP180_REG_DATA BMP180_REG_OUT_MSB
#define BMP180_SIZE_DATA 3 /* bytes */

#define BMP180_CTRL_TEMP 0x2e
#define BMP180_CTRL_PRES_OSS_0 0x34
#define BMP180_CTRL_PRES_OSS_1 0x74
#define BMP180_CTRL_PRES_OSS_2 0xb4
#define BMP180_CTRL_PRES_OSS_3 0xf4

void bmp180_i2c_read_reg(uint8_t i2c, uint8_t reg, uint8_t count, uint8_t *buf);
void bmp180_i2c_write_reg(uint8_t i2c, uint8_t *buf, uint8_t count);

/*
 * Reading/writing registers
 */
void bmp180_i2c_read_reg(uint8_t i2c, uint8_t reg, uint8_t count, uint8_t *buf) {
	i2c_transfer(i2c, BMP180_I2C_ADDR, &reg, 1, buf, count);
}

void bmp180_i2c_write_reg(uint8_t i2c, uint8_t *buf, uint8_t count) {
	i2c_transfer(i2c, BMP180_I2C_ADDR, buf, count, 0, 0);
}

int32_t bmp180_i2c_temp(uint8_t i2c) {
	return 0;
}

uint32_t bmp180_i2c_pres(uint8_t i2c) {
	return 0;
}

void bmp180_i2c_get(uint8_t i2c, int32_t *temp, uint32_t *pres) {

}

void bmp180_i2c_init(uint8_t i2c) {
	i2c_init_master(i2c);
	//bmp180_i2c_reset();
}

