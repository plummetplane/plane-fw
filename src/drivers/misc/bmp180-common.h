#ifndef __BMP180_COMMON_H__
#define __BMP180_COMMON_H__

int32_t bmp180_comp_temp(uint8_t *temp, uint8_t *cal);

uint32_t bmp180_comp_pres(uint8_t *temp, uint8_t *pres, uint8_t *cal, uint8_t *ctrl_meas);

void bmp180_comp(uint8_t *temp, uint8_t *pres, uint8_t *cal, uint8_t *ctrl_meas, int32_t *temp_comp, int32_t *pres_comp);

#endif /* __BMP180_COMMON_H__ */
