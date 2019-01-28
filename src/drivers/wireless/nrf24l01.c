#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "../serial/spi.h"
#include "nrf24l01.h"

#define NRF24L01_REG_BUF_MAX 5

/*
 * SPI commands
 */
#define NRF24L01_CMD_R_REGISTER 0x00
#define NRF24L01_CMD_W_REGISTER 0x20
#define NRF24L01_CMD_R_RX_PAYLOAD 0x61
#define NRF24L01_CMD_W_TX_PAYLOAD 0xa0
#define NRF24L01_CMD_FLUSH_TX 0xe1
#define NRF24L01_CMD_FLUSH_RX 0xe2
#define NRF24L01_CMD_REUSE_TX_PL 0xe3
#define NRF24L01_CMD_R_RX_PL_WID 0x60
#define NRF24L01_CMD_W_ACK_PAYLOAD 0xa8
#define NRF24L01_CMD_W_TX_PAYLOAD_NOACK 0xb0
#define NRF24L01_CMD_NOP 0xff

/*
 * Registers
 */
#define NRF24L01_REG_CONFIG 0x00
#define NRF24L01_REG_EN_AA 0x01
#define NRF24L01_REG_EN_RXADDR 0x02
#define NRF24L01_REG_SETUP_AW 0x03
#define NRF24L01_REG_SETUP_RETR 0x04
#define NRF24L01_REG_RF_CH 0x05
#define NRF24L01_REG_RF_SETUP 0x06
#define NRF24L01_REG_STATUS 0x07
#define NRF24L01_REG_OBSERVE_TX 0x08
#define NRF24L01_REG_RPD 0x09
#define NRF24L01_REG_RX_ADDR_P0 0x0a
#define NRF24L01_REG_RX_ADDR_P1 0x0b
#define NRF24L01_REG_RX_ADDR_P2 0x0c
#define NRF24L01_REG_RX_ADDR_P3 0x0d
#define NRF24L01_REG_RX_ADDR_P4 0x0e
#define NRF24L01_REG_RX_ADDR_P5 0x0f
#define NRF24L01_REG_TX_ADDR 0x10
#define NRF24L01_REG_RX_PW_P0 0x11
#define NRF24L01_REG_RX_PW_P1 0x12
#define NRF24L01_REG_RX_PW_P2 0x13
#define NRF24L01_REG_RX_PW_P3 0x14
#define NRF24L01_REG_RX_PW_P4 0x15
#define NRF24L01_REG_RX_PW_P5 0x16
#define NRF24L01_REG_FIFO_STATUS 0x17
#define NRF24L01_REG_DYNDP 0x1c
#define NRF24L01_REG_FEATURE 0x1d

/*
 * Config register
 */
#define NRF24L01_REG_CONFIG_MASK_RX_DR 0x06
#define NRF24L01_REG_CONFIG_MASK_TX_DS 0x05
#define NRF24L01_REG_CONFIG_MASK_MAX_RT 0x04
#define NRF24L01_REG_CONFIG_EN_CRC 0x03
#define NRF24L01_REG_CONFIG_CRCO 0x02
#define NRF24L01_REG_CONFIG_PWR_UP 0x01
#define NRF24L01_REG_CONFIG_PRIM_RX 0x00

/*
 * Status register
 */
#define NRF24L01_REG_STATUS_RX_DR 0x06
#define NRF24L01_REG_STATUS_TX_DS 0x05
#define NRF24L01_REG_STATUS_MAX_RT 0x04
#define NRF24L01_REG_STATUS_RX_P_NO 0x01
#define NRF24L01_REG_STATUS_TX_FULL 0x00

typedef struct {
	uint8_t size;
	uint8_t reg_size[];
} nrf24l01_regs;

uint32_t spi;
uint32_t en_gpioport;
uint16_t en_gpios;
void (*rxdr_callback)(nrf24l01_payload payload) = 0;
void (*txds_callback)(void) = 0;
void (*maxrt_callback)(void) = 0;

/*
 * Registers
 */
const nrf24l01_regs nrf24l01_reg_def = {
	.size = 30,
	.reg_size =
	{
		/*
		 * CONFIG
		 * 0 PRIM_RX
		 * 1 PWR_UP
		 * 2 CRCO
		 * 3 EN_CRC
		 * 4 MASK_MAX_RT
		 * 5 MASK_TX_DS
		 * 6 MASK_RX_DR
		 * 7 RESERVED
		 */
		1,
		/*
		 * EN_AA
		 * 0 ENAA_P0
		 * 1 ENAA_P1
		 * 2 ENAA_P2
		 * 3 ENAA_P3
		 * 4 ENAA_P4
		 * 5 ENAA_P5
		 * 7:6 RESERVED
		 */
		1,
		1,	/* EN_RXADDR */
		1,	/* SETUP_AW */
		1,	/* SETUP_RETR */
		1,	/* RF_CH */
		1,	/* RF_SETUP */
		1,	/* STATUS */
		1,	/* OBSERVE_TX */
		1,	/* CD */
		5,	/* RX_ADDR_P0 */
		5,	/* RX_ADDR_P1 */
		5,	/* RX_ADDR_P2 */
		5,	/* RX_ADDR_P3 */
		5,	/* RX_ADDR_P4 */
		5,	/* RX_ADDR_P5 */
		5,	/* RX_ADDR_P6 */
		5,	/* TX_ADDR */
		1,	/* RX_PW_P0 */
		1,	/* RX_PW_P1 */
		1,	/* RX_PW_P2 */
		1,	/* RX_PW_P3 */
		1,	/* RX_PW_P4 */
		1,	/* RX_PW_P5 */
		1,	/* FIFO_STATUS */
		0,	/* NA */
		0,	/* NA */
		0,	/* NA */
		0,	/* NA */
		1,	/* DYNDP */
		1,	/* FEATURE */
	}
};

void nrf24l01_read_reg(uint8_t reg, uint8_t *buf);

void nrf24l01_write_reg(uint8_t reg, uint8_t *buf);

void nrf24l01_config(uint8_t field, uint8_t value);

uint8_t nrf24l01_read_status(uint8_t field);

void nrf24l01_clear_status(uint8_t field);

void nrf24l01_init(uint32_t spi_dev, uint32_t gpioport, uint16_t gpios, uint8_t mode) {
	spi_init();

	spi = spi_dev;
	en_gpioport = gpioport;
	en_gpios = gpios;

	rcc_periph_clock_enable(en_gpioport);

	gpio_set_mode(en_gpioport, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, en_gpios);

	nrf24l01_mode(mode);

	nrf24l01_enable_power();
}

/*
 * Reading/writing registers
 */
void nrf24l01_read_reg(uint8_t reg, uint8_t *buf) {
	spi_select();

	spi_drv_xfer(spi, (NRF24L01_CMD_R_REGISTER | reg));

	for (uint8_t i = 0; i < nrf24l01_reg_def.reg_size[reg]; i++) {
		buf[i] = spi_drv_xfer(spi, NRF24L01_CMD_NOP);
	}

	spi_deselect();
}

void nrf24l01_write_reg(uint8_t reg, uint8_t *buf) {
	spi_select();

	spi_drv_xfer(spi, (NRF24L01_CMD_W_REGISTER | reg));

	for (uint8_t i = 0; i < nrf24l01_reg_def.reg_size[reg]; i++) {
		spi_drv_xfer(spi, buf[i]);
	}

	spi_deselect();

}

/*
 * Configuration
 */
void nrf24l01_config(uint8_t field, uint8_t value) {
	uint8_t config_reg;

	nrf24l01_read_reg(NRF24L01_REG_CONFIG, &config_reg);

	config_reg &= ~(0x01 << field);
	config_reg |= (value << field);

	nrf24l01_write_reg(NRF24L01_REG_CONFIG, &config_reg);
}

void nrf24l01_enable_crc(void) {
	nrf24l01_config(NRF24L01_REG_CONFIG_EN_CRC, 1);
}

void nrf24l01_disable_crc(void) {
	nrf24l01_config(NRF24L01_REG_CONFIG_EN_CRC, 0);
}

void nrf24l01_enable_power(void) {
	nrf24l01_config(NRF24L01_REG_CONFIG_PWR_UP, 1);
}

void nrf24l01_disable_power(void) {
	nrf24l01_config(NRF24L01_REG_CONFIG_PWR_UP, 0);
}

void nrf24l01_mode(uint8_t mode) {
	nrf24l01_config(NRF24L01_REG_CONFIG_PRIM_RX, mode);

	if (mode == NRF24L01_MODE_RX) {
		gpio_set(en_gpioport, en_gpios);
	} else {
		gpio_clear(en_gpioport, en_gpios);
	}
}

/*
 * Read status
 */
uint8_t nrf24l01_read_status(uint8_t field) {
	uint8_t status;

	nrf24l01_read_reg(NRF24L01_REG_STATUS, &status);

	/* return only the value of the requested field */
	return ((status & (1 << field)) >> field);
}

void nrf24l01_clear_status(uint8_t field) {
	uint8_t reg = (1 << field);

	nrf24l01_write_reg(NRF24L01_REG_STATUS, &reg);
}

/*
 * Send/Receive data
 */
int nrf24l01_transmit(nrf24l01_payload *payload) {
	if (nrf24l01_read_status(NRF24L01_REG_STATUS_TX_FULL) == 1) {
		spi_select();
		spi_drv_xfer(spi, NRF24L01_CMD_FLUSH_TX);
		spi_deselect();
	}

	spi_select();

	spi_drv_xfer(spi, NRF24L01_CMD_W_TX_PAYLOAD);

	for (uint8_t i = 0; i < payload->size; i++) {
		spi_drv_xfer(spi, payload->data[i]);
	}

	spi_deselect();

	/* create >10us pulse on EN pin */
	gpio_set(en_gpioport, en_gpios);
	for (uint16_t i = 720; i > 0; i--)
		__asm("nop");
	gpio_clear(en_gpioport, en_gpios);

	return 0;
}

int nrf24l01_receive(nrf24l01_payload *payload) {
	/* Read number of received bytes in pipe 0 */
	uint8_t rx_pw;

	nrf24l01_read_reg(NRF24L01_REG_RX_PW_P0, &rx_pw);

	/* read received bytes */
	spi_select();

	spi_drv_xfer(spi, NRF24L01_CMD_R_RX_PAYLOAD);

	for (uint8_t i = 0; i < rx_pw; i++) {
		spi_drv_xfer(spi, payload->data[i]);
	}

	payload->size = rx_pw;

	spi_deselect();

	return 0;
}

/*
 * Setup callback functions
 */
void nrf24l01_rxdr_callback(void (*callback_func)(nrf24l01_payload payload)) {
	rxdr_callback = callback_func;
}

void nrf24l01_txds_callback(void (*callback_func)(void)) {
	txds_callback = callback_func;
}

void nrf24l01_maxrt_callback(void (*callback_func)(void)) {
	maxrt_callback = callback_func;
}

/*
 * IRQ handler
 */
void nrf24l01_irq(void) {
	uint8_t status;
	nrf24l01_read_reg(NRF24L01_REG_STATUS, &status);

	if ((status & (1 << NRF24L01_REG_STATUS_RX_DR)) != 0) {
		/* Received data */
		if (rxdr_callback != 0) {
			nrf24l01_payload rxdata;
			nrf24l01_receive(&rxdata);
			rxdr_callback(rxdata);
		} else {
			spi_select();
			spi_drv_xfer(spi, NRF24L01_CMD_FLUSH_RX);
			spi_deselect();
		}

		nrf24l01_clear_status(NRF24L01_REG_STATUS_RX_DR);
	} else if ((status & (1 << NRF24L01_REG_STATUS_TX_DS)) != 0) {
		/* Successful transmission (received ACK) */
		if (txds_callback != 0)
			txds_callback();

		nrf24l01_clear_status(NRF24L01_REG_STATUS_TX_DS);
	} else if ((status & (1 << NRF24L01_REG_STATUS_MAX_RT)) != 0) {
		/* Maximum retransmit count reached (probably an unsucessful transmission) */
		if (maxrt_callback != 0)
			maxrt_callback();

		nrf24l01_clear_status(NRF24L01_REG_STATUS_MAX_RT);
	}
}
