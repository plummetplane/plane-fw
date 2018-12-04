#include <stdint.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#include "i2c.h"

volatile uint8_t i2c1_data[I2C_BUF_SIZE] = {0}, i2c2_data[I2C_BUF_SIZE] = {0};
volatile uint16_t i2c1_pos = 0xffff, i2c2_pos = 0xffff;

void (*i2c1_callback)(uint8_t *buf, uint32_t buf_size) = 0,
     (*i2c2_callback)(uint8_t *buf, uint32_t buf_size) = 0;

int i2c_setup(uint32_t i2c, uint8_t address) {
	uint32_t rcc_i2c, nvic_i2c_ev_irq, rcc_gpio_port, gpio_scl, gpio_sda;
	switch (i2c) {
		case I2C1:
			rcc_i2c = RCC_I2C1;
			nvic_i2c_ev_irq = NVIC_I2C1_EV_IRQ;
			gpio_scl = GPIO_I2C1_SCL;
			gpio_sda = GPIO_I2C1_SDA;
			break;
//		case I2C2:
//			rcc_i2c = RCC_I2C2;
//			nvic_i2c_ev_irq = NVIC_I2C2_EV_IRQ;
//			gpio_scl = GPIO_I2C2_SCL;
//			gpio_sda = GPIO_I2C2_SDA;
//			break;
		default:
			return -1;
			break;
	}

	/* enable clocks */
	rcc_periph_clock_enable(rcc_i2c);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

	/* setup gpio */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, gpio_scl | gpio_sda);

	/* reset i2c */
	i2c_reset(i2c);

	/* disable i2c before configuration */
	i2c_peripheral_disable(i2c);

	/* set clock -- TODO: verify */
	//i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
	i2c_set_speed(i2c, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);

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

	if (address != 0x00) {
		nvic_enable_irq(nvic_i2c_ev_irq);

		/* set slave address, to be able to address this device */
		i2c_set_own_7bit_slave_address(i2c, address);

		/* enable interrupt */
		i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
	}

	/* enable the interface */
	i2c_peripheral_enable(i2c);

	if (address != 0x00) {
		/* enable sending ACK */
		i2c_enable_ack(I2C1);
	}

	return 0;
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
		i2c_peripheral_enable(I2C1);

		/*
		 * callback is triggered after both, reading and writing
		 * also after receiving address for reading (just before reading)
		 */
		if (i2c1_callback != 0) {
			i2c1_callback(i2c1_data, I2C_BUF_SIZE);
		}
	} else if (I2C_SR1(I2C1) & I2C_SR1_AF) {
		I2C_SR1(I2C1) &= ~I2C_SR1_AF;
	}
}

void i2c_set_callback(uint32_t i2c, void (*callback)(uint8_t *buf, uint32_t buf_size)) {
	void (**i2c_callback)(uint8_t *buf, uint32_t buf_size) = 0;
	switch (i2c) {
		case I2C1:
			i2c_callback = &i2c1_callback;
			break;
//		case I2C2:
//			i2c_callback = &i2c2_callback;
//			bbreak;
		default:
			return;
			break;
	}

	*i2c_callback = callback;
}

/* only 7-bit address range for now */
int i2c_init_slave(uint32_t i2c, uint8_t address)  {
	if (address < 0x07 || address > 0x78)
		return -1;
	return i2c_setup(i2c, address);
}

/* initialize i2c interface without assigning salve address */
int i2c_init_master(uint32_t i2c) {
	return i2c_setup(i2c, 0x00);
}
