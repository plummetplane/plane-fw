#include <stdint.h>

#include "../serial/i2c.h"
#include "bmp280-common.h"

#include "bmp280-i2c.h"

#define BMP280_I2C_ADDR 0x76

#define BMP280_REG_CALIB 0x88
#define BMP280_REG_ID 0xd0
#define BMP280_REG_RESET 0xe0
#define BMP280_REG_STATUS 0xf3
#define BMP280_REG_CTRL_MEAS 0xf4
#define BMP280_REG_CONFIG 0xf5
#define BMP280_REG_PRESS_MSB 0xf7
#define BMP280_REG_PRESS_LSB 0xf8
#define BMP280_REG_PRESS_XLSB 0xf9
#define BMP280_REG_TEMP_MSB 0xfa
#define BMP280_REG_TEMP_LSB 0xfb
#define BMP280_REG_TEMP_XLSB 0xfc
#define BMP280_REG_DATA BMP280_REG_PRESS_MSB

#define BMP280_CALIB_SIZE 26

void bmp280_i2c_read_reg(uint8_t i2c, uint8_t reg, uint8_t count, uint8_t *reg_buf);

void bmp280_i2c_setup(uint8_t i2c);

/* Only the I2C interface needs to be configured, but also reset the bmp */
void bmp280_i2c_init(uint8_t i2c) {
	i2c_init_master(i2c);
	bmp280_i2c_setup(i2c);
}

/* setup the sesor for a reading */
void bmp280_i2c_setup(uint8_t i2c) {
	const uint8_t setup_seq[2] = {
		BMP280_REG_CTRL_MEAS,
		//0x93
		0x33
	};

	i2c_transfer(i2c, BMP280_I2C_ADDR, setup_seq, 2, 0, 0);

	for (uint16_t i = 0xffff; i > 0; i--)
		__asm("nop");
}

/* issue reset command */
void bmp280_i2c_reset(uint8_t i2c) {
	const uint8_t reset_seq[2] = {
		BMP280_REG_RESET,
		0xb6
	};

	i2c_transfer(i2c, BMP280_I2C_ADDR, reset_seq, 2, 0, 0);
}

void bmp280_i2c_read_reg(uint8_t i2c, uint8_t reg, uint8_t count, uint8_t *reg_buf) {
	i2c_transfer(i2c, BMP280_I2C_ADDR, &reg, 1, reg_buf, count);
}

int32_t bmp280_i2c_temp(uint8_t i2c) {
	uint8_t data_regs[BMP280_DATA_SIZE];
	uint8_t cal_regs[BMP280_CALIB_SIZE];
	bmp280_i2c_read_reg(i2c, BMP280_REG_DATA, BMP280_DATA_SIZE, data_regs);
	bmp280_i2c_read_reg(i2c, BMP280_REG_CALIB, BMP280_CALIB_SIZE, cal_regs);

	return bmp280_comp_temp(&data_regs[3], cal_regs);
}

uint32_t bmp280_i2c_pres(uint8_t i2c) {
	uint8_t data_regs[BMP280_DATA_SIZE];
	uint8_t cal_regs[BMP280_CALIB_SIZE];
	bmp280_i2c_read_reg(i2c, BMP280_REG_DATA, BMP280_DATA_SIZE, data_regs);
	bmp280_i2c_read_reg(i2c, BMP280_REG_CALIB, BMP280_CALIB_SIZE, cal_regs);

	return bmp280_comp_pres(&data_regs[0], &data_regs[3], cal_regs);
}

void bmp280_i2c_get(uint8_t i2c, int32_t *temp, uint32_t *pres) {
	uint8_t data_regs[BMP280_DATA_SIZE];
	uint8_t cal_regs[BMP280_CALIB_SIZE];
	bmp280_i2c_read_reg(i2c, BMP280_REG_DATA, BMP280_DATA_SIZE, data_regs);
	bmp280_i2c_read_reg(i2c, BMP280_REG_CALIB, BMP280_CALIB_SIZE, cal_regs);

	bmp280_comp(&data_regs[0], &data_regs[3], cal_regs, temp, pres);
}
