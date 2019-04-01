#include <stdint.h>

#include "bmp180-common.h"

int32_t bmp180_t_fine(uint8_t *data, uint8_t *cal);
int32_t bmp180_t_temp(int32_t t_fine);

/*
 * Compensate temperature value by the procedure described in the datasheet
 */
int32_t bmp180_t_fine(uint8_t *data, uint8_t *cal) {
	/* assemble uncompensated temp */
	int32_t utemp = (data[0] << 8) + data[1];

	int32_t x, t_fine;

	x = ((utemp - (cal[10] << 8 + cal[11])) * (cal[8] << 8 + cal[9])) >> 15;
	t_fine = ((cal[18] << 8 + cal[19]) << 11) / (x + cal[20] << 8 + cal[21]);
	t_fine += x;

	return t_fine;
}

int32_t bmp180_t_temp(int32_t t_fine) {
	return (t_fine + 8) << 4;
}

/*
 * Compensate raw pressure value
 */
int32_t bmp180_pres(uint8_t *pres, uint8_t *cal, int32_t t_fine, uint8_t oss) {
	int32_t upres = (pres[0] << 16 + pres[1] << 8 + pres[3]) >> (8 - oss);

	int32_t comp_pres;

	int32_t x1, x2, x3, b3, b4, b7;
	t_fine -= 4000;
	x1 = ((cal[14] << 8 + cal[15]) * (t_fine * t_fine >> 12)) >> 11;
	x2 = (cal[2] << 8 + cal[3]) * t_fine >> 11;
	x3 = x1 + x2;

	b3 = ((((long)(cal[0] << 8 + cal[1]) * 4 + x3) << oss) + 2) / 4;
	x1 = (cal[4] << 8 + cal[5]) * t_fine >> 13;
	x2 = ((cal[12] << 8 + cal[13]) * (t_fine * t_fine >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;

	b4 = (cal[6] << 8 + cal[7]) * (unsigned long)(x3 + 32768) >> 15;
	b7 = ((unsigned long)upres - b3) * (50000 >> oss);

	if (b7 < 0x80000000) {
		comp_pres = (b7 * 2) / b4;
	} else {
		comp_pres = (b7 / b4) * 2;
	}

	x1 = (comp_pres >> 8) * (comp_pres >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * comp_pres) >> 16;
	comp_pres += (x1 + x2 + 3791) >> 4;

	return comp_pres;
}

int32_t bmp180_comp_temp(uint8_t *temp, uint8_t *cal) {
	int32_t t_fine = bmp180_t_fine(temp, cal);
	return bmp180_t_temp(t_fine);
}

uint32_t bmp180_comp_pres(uint8_t *temp, uint8_t *pres, uint8_t *cal, uint8_t *ctrl_meas) {
	return 0;
}

void bmp180_comp(uint8_t *temp, uint8_t *pres, uint8_t *cal, uint8_t *ctrl_meas, int32_t *comp_temp, int32_t *comp_pres) {
	int32_t t_fine = bmp180_t_fine(temp, cal);
	*comp_temp = bmp180_t_temp(t_fine);
	*comp_pres = bmp180_pres(pres, cal, t_fine, (*ctrl_meas >> 6));
}
