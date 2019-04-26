#include <stdint.h>

#include "../drivers/misc/bmp280-i2c.h"

uint32_t temp = 0;
uint32_t pres;

float pf;

int main(void) {
	bmp280_i2c_init(0);

	for (uint32_t i = 0xfffff; i > 0; i--)
		__asm("nop");

	bmp280_i2c_get(0, &temp, &pres);

	pf = (float)pres;

	pres /= 256;

	while(1)
		__asm("nop");
}
