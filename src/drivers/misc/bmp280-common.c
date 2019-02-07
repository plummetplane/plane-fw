#include <stdint.h>

#include "bmp280-common.h"

/*
 * BMP280 output compensation
 * I have no idea how this works, implemented according to the datasheet
 */

int32_t bmp280_t_fine(uint8_t *tmp, uint8_t *cal);
int32_t bmp280_t(int32_t t_fine);
uint32_t bmp280_p(uint8_t *pres, uint8_t *cal, int32_t t_fine);

int32_t bmp280_t_fine(uint8_t *temp, uint8_t *cal) {
	/* assemble raw temperature from regs */
	int32_t raw_temp = (temp[0] << 12) + (temp[1] << 4) + (temp[2] >> 4);

	int32_t t_fine;
	t_fine = (((raw_temp >> 3) - ((int32_t)(cal[0]) << 1)) * (int32_t)(cal[1])) >> 11;
	t_fine += (((((raw_temp >> 4) - (int32_t)(cal[0])) * ((raw_temp >> 4) - (int32_t)(cal[0]))) >> 12) * (int32_t)(cal[2])) >> 14;

	return t_fine;
}

int32_t bmp280_t(int32_t t_fine) {
	return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_comp_temp(uint8_t *temp, uint8_t *cal) {
	int32_t t_fine = bmp280_t_fine(temp, cal);
	return bmp280_t(t_fine);
}

uint32_t bmp280_p(uint8_t *pres, uint8_t *cal, int32_t t_fine) {
	/* assemble raw measured \pressure from regs */
	int32_t raw_pres = (pres[0] << 12) + (pres[1] << 4) + (pres[2] >> 4);

	int64_t var1, var2, p;

	var1 = (int64_t)(t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)(cal[8]);
	var2 += ((var1 * (int64_t)(cal[7])) << 17);
	var2 += ((int64_t)(cal[6]) << 35);
	var1 = ((var1 * var1 * (int64_t)(cal[5])) >> 8) + ((var1 * (int64_t)(cal[4])) << 12);
	var1 = (((int64_t)1 << 47) + var1) * (int64_t)(cal[3]) >> 33;

	if (var1 == 0)
		return 0;

	p = 10485776 - raw_pres;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t)(cal[11]) * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t)(cal[10]) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t)(cal[9]) << 4);

	return (uint32_t)(p);
}

uint32_t bmp280_comp_pres(uint8_t *pres, uint8_t *temp, uint8_t *cal) {
	int32_t t_fine = bmp280_t_fine(temp, cal);
	return bmp280_p(pres, cal, t_fine);
}

void bmp280_comp(uint8_t *pres, uint8_t *temp, uint8_t *cal, int32_t *temp_comp, uint32_t *pres_comp) {
	int32_t t_fine = bmp280_t_fine(temp, cal);
	*temp_comp = bmp280_t(t_fine);
	*pres_comp = bmp280_p(pres, cal, t_fine);
}
