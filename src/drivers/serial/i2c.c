#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#include "i2c.h"

struct i2c_stm_regs {
	uint32_t rcc_i2c;
	uint32_t i2c;
	uint32_t nvic_i2c_ev_irq;
	uint32_t rcc_gpioport;
	uint32_t gpioport;
	uint16_t gpio_scl;
	uint16_t gpio_sda;
};

struct i2c_buf {
	uint8_t data[I2C_BUF_SIZE];
	uint16_t pos;
};

static struct i2c_stm_regs i2c_dev[] =
{
	{.rcc_i2c = RCC_I2C1, .i2c = I2C1, .nvic_i2c_ev_irq = NVIC_I2C1_EV_IRQ, .rcc_gpioport = RCC_GPIOB, .gpioport = GPIOB,
		.gpio_scl = GPIO_I2C1_SCL, .gpio_sda = GPIO_I2C1_SDA},
	{.rcc_i2c = RCC_I2C2, .i2c = I2C2, .nvic_i2c_ev_irq = NVIC_I2C2_EV_IRQ, .rcc_gpioport = RCC_GPIOB, .gpioport = GPIOB,
		.gpio_scl = GPIO_I2C2_SCL, .gpio_sda = GPIO_I2C2_SDA}
};

struct i2c_buf i2c_bufs[2];

void (*i2c1_callback)(uint8_t *buf, uint32_t buf_size) = 0,
     (*i2c2_callback)(uint8_t *buf, uint32_t buf_size) = 0;

void i2c_clear_buf(uint8_t i2c) {
	memset(i2c_bufs[i2c].data, 0, sizeof(i2c_bufs[i2c].data));
	i2c_bufs[i2c].pos = 0xffff;
}

int i2c_setup(uint8_t i2c, uint8_t address) {
	/* enable clocks */
	rcc_periph_clock_enable(i2c_dev[i2c].rcc_i2c);
	rcc_periph_clock_enable(i2c_dev[i2c].rcc_gpioport);
	rcc_periph_clock_enable(RCC_AFIO);

	/* setup gpio */
	gpio_set_mode(i2c_dev[i2c].gpioport, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, i2c_dev[i2c].gpio_scl | i2c_dev[i2c].gpio_sda);

	/* reset i2c */
	i2c_reset(i2c_dev[i2c].i2c);

	/* disable i2c before configuration */
	i2c_peripheral_disable(i2c_dev[i2c].i2c);

	/* set clock -- TODO: verify */
	//i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
	i2c_set_speed(i2c_dev[i2c].i2c, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);

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
		nvic_enable_irq(i2c_dev[i2c].nvic_i2c_ev_irq);

		/* set slave address, to be able to address this device */
		i2c_set_own_7bit_slave_address(i2c_dev[i2c].i2c, address);

		/* enable interrupt */
		i2c_enable_interrupt(i2c_dev[i2c].i2c, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
	}

	/* enable the interface */
	i2c_peripheral_enable(i2c_dev[i2c].i2c);

	if (address != 0x00) {
		/* enable sending ACK */
		i2c_enable_ack(i2c_dev[i2c].i2c);
	}

	return 0;
}

void i2c_ev_isr(uint8_t i2c) {
	if (I2C_SR1(i2c_dev[i2c].i2c) & I2C_SR1_ADDR) {
		i2c_bufs[i2c].pos = 0xffff;

		/* the ADDR I2C flag is cleared by reading the SR2 */
		(void) I2C_SR2(i2c_dev[i2c].i2c);
	} else if (I2C_SR1(i2c_dev[i2c].i2c) & I2C_SR1_RxNE) {
		/* receiving data */
		if (i2c_bufs[i2c].pos == 0xffff) {
			/*
			 * first byte is the register address
			 * if it is a valid address, set it as a buffer position otherwise fallback to address 0x00
			 */
			uint8_t data = i2c_get_data(i2c_dev[i2c].i2c);
			i2c_bufs[i2c].pos = (data < I2C_BUF_SIZE)?data:0x00;
		} else {
			if (i2c_bufs[i2c].pos < I2C_BUF_SIZE) {
				/* write received data to the buffer */
				i2c_bufs[i2c].data[i2c_bufs[i2c].pos++] = i2c_get_data(i2c_dev[i2c].i2c);

			} else {
				/*
				 * if we are at the end of the buffer send NACK instead of ACK to
				 * notify the master there is no space lesft to write to
				 */
				/* this is necessary or else the execution locks up */
				(void) i2c_get_data(i2c_dev[i2c].i2c);
				/*
				 * at this point NACK should be sent instead of ACK, however,
				 * there is no way, that i know of, to accomplish this
				 */
			}
		}
	} else if ((I2C_SR1(i2c_dev[i2c].i2c) & I2C_SR1_TxE) && !(I2C_SR1(i2c_dev[i2c].i2c) & I2C_SR1_BTF)) {
		if (i2c_bufs[i2c].pos == 0xffff) {
			/*
			 * first byte is the register address
			 * if it is a valid address, set it as a buffer position otherwise fallback to address 0x00
			 */
			uint8_t data = i2c_get_data(i2c_dev[i2c].i2c);
			i2c_bufs[i2c].pos = (data < I2C_BUF_SIZE)?data:0x00;
		}

		if (i2c_bufs[i2c].pos < I2C_BUF_SIZE) {
			i2c_send_data(i2c_dev[i2c].i2c, i2c_bufs[i2c].data[i2c_bufs[i2c].pos++]);
		} else {
			/* similiar scenario to that above (except now we are transmitting data) */
			i2c_send_data(i2c_dev[i2c].i2c, 0xff);
		}
	} else if (I2C_SR1(i2c_dev[i2c].i2c) & I2C_SR1_STOPF) {
		i2c_peripheral_enable(i2c_dev[i2c].i2c);

		/*
		 * callback is triggered after both, reading and writing
		 * also after receiving address for reading (just before reading)
		 */
		void (*i2c_callback)(uint8_t *buf, uint32_t buf_size) = 0;
		switch (i2c) {
			case 0:
				i2c_callback = i2c1_callback;
				break;
			case 1:
				i2c_callback = i2c2_callback;
				break;
		}
		if (i2c_callback != 0) {
			i2c_callback(i2c_bufs[i2c].data, I2C_BUF_SIZE);
		}
	} else if (I2C_SR1(i2c_dev[i2c].i2c) & I2C_SR1_AF) {
		I2C_SR1(i2c_dev[i2c].i2c) &= ~I2C_SR1_AF;
	}
}

void i2c1_ev_isr(void) {
	i2c_ev_isr(0);
}

void i2c2_ev_isr(void) {
	i2c_ev_isr(1);
}

void i2c_set_callback(uint8_t i2c, void (*callback)(uint8_t *buf, uint32_t buf_size)) {
	void (**i2c_callback)(uint8_t *buf, uint32_t buf_size) = 0;
	switch (i2c) {
		case 0:
			i2c_callback = &i2c1_callback;
			break;
		case 1:
			i2c_callback = &i2c2_callback;
			break;
		default:
			return;
			break;
	}

	*i2c_callback = callback;
}

void i2c_transfer(uint8_t i2c, uint8_t addr, uint8_t *w, uint32_t wn, uint8_t *r, uint32_t rn) {
	i2c_transfer7(i2c_dev[i2c].i2c, addr, w, wn, r, rn);
}

/* only 7-bit address range for now */
int i2c_init_slave(uint8_t i2c, uint8_t address)  {
	/* verify whether the address is valid */
	if (address < 0x07 || address > 0x78)
		return -1;

	i2c_clear_buf(i2c);
	return i2c_setup(i2c, address);
}

/* initialize i2c interface without assigning salve address */
int i2c_init_master(uint8_t i2c) {
	i2c_clear_buf(i2c);
	return i2c_setup(i2c, 0x00);
}
