#ifndef __BMP280_COMMON_H__
#define __BMP280_COMMON_H__

/*
 * compensate temperature value
 * resolution is 0.01 deg C, the returned value is *100
 */
int32_t bmp280_comp_temp(uint8_t *temp, uint8_t *cal);

/*
 * compensate raw pressure value
 * the returned value is *256 Pa
 */
uint32_t bmp280_comp_pres(uint8_t *pres, uint8_t *temp, uint8_t *cal);

/*
 * compensate both temperature and pressure
 * the temp has to be calculated for pressure anyway, so why not calculate both at the same time?
 */
void bmp280_comp(uint8_t *pres, uint8_t *temp, uint8_t *cal, int32_t *temp_comp, uint32_t *pres_comp);

#endif /* __BMP280_COMMON_H__ */
