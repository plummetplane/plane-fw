#include <stdint.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#include "i2c.h"

volatile uint8_t i2c1_data[I2C_BUF_SIZE] = {0};
volatile uint16_t i2c1_pos = 0xffff;

void i2c_setup(void) {
	/* enable clocks */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

	/* setup gpio */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA);

	nvic_enable_irq(NVIC_I2C1_EV_IRQ);

	/* reset i2c */
	i2c_reset(I2C1);

	/* disable i2c before configuration */
	i2c_peripheral_disable(I2C1);

	/* set clock -- TODO: verify */
	//i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);

	/* 400KHz - I2C Fast Mode */
	// i2c_set_fast_mode(I2C1);

	/*
	 * for 100khz the datasheet uggests 0xb4 for ccr value
	 * 0x1e for fast mode (400khz)
	 */
	//i2c_set_ccr(I2C1, 0xb4);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 1000ns/28ns = 35;
	 * Incremented by 1 -> 36.
	 */
	//i2c_set_trise(I2C1, 0x24);

	/* set slave address, to be able to address this device */
	i2c_set_own_7bit_slave_address(I2C1, 0x27);

	/* enable interrupt */
	i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

	/* enable the interface */
	i2c_peripheral_enable(I2C1);

	/* enable sending acknowledgement packets */
	i2c_enable_ack(I2C1);
}

void i2c1_ev_isr(void) {
	if (I2C_SR1(I2C1) & I2C_SR1_ADDR) {
		i2c1_pos = 0xffff;

		/* the ADDR I2C flag is cleared by reading the SR2 */
		(void) I2C_SR2(I2C1);
	} else if (I2C_SR1(I2C1) & I2C_SR1_RxNE) {
		/* receiving data */
		if (i2c1_pos == 0xffff) {
			/*
			 * first byte is the register address
			 * if it is a valid address, set it as a buffer position otherwise fallback to address 0x00
			 */
			uint8_t data = i2c_get_data(I2C1);
			i2c1_pos = (data < I2C_BUF_SIZE)?data:0x00;
		} else {
			if (i2c1_pos < I2C_BUF_SIZE) {
				/* write received data to the buffer */
				i2c1_data[i2c1_pos++] = i2c_get_data(I2C1);

			} else {
				/*
				 * if we are at the end of the buffer send NACK instead of ACK to
				 * notify the master there is no space lesft to write to
				 */
				/* this is necessary or else the execution locks up */
				(void) i2c_get_data(I2C1);
				/*
				 * at this point NACK should be sent instead of ACK, however,
				 * there is no way, that i know of, to accomplish this
				 */
			}
		}
	} else if ((I2C_SR1(I2C1) & I2C_SR1_TxE) && !(I2C_SR1(I2C1) & I2C_SR1_BTF)) {
		if (i2c1_pos == 0xffff) {
			/*
			 * first byte is the register address
			 * if it is a valid address, set it as a buffer position otherwise fallback to address 0x00
			 */
			uint8_t data = i2c_get_data(I2C1);
			i2c1_pos = (data < I2C_BUF_SIZE)?data:0x00;
		}

		if (i2c1_pos < I2C_BUF_SIZE) {
			i2c_send_data(I2C1, i2c1_data[i2c1_pos++]);
		} else {
			/* similiar scenario to that above (except now we are transmitting data) */
			i2c_send_data(I2C1, 0xff);
		}
	} else if (I2C_SR1(I2C1) & I2C_SR1_STOPF) {
		/* callback */
		i2c_peripheral_enable(I2C1);
	} else if (I2C_SR1(I2C1) & I2C_SR1_AF) {
		I2C_SR1(I2C1) &= ~I2C_SR1_AF;
	}
}

int i2c_init(void) {
	i2c_setup();
}
