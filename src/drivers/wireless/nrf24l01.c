#include <stdint.h>

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
#define NRF24L01_REG_ENAA 0x01
#define NRF24L01_REG_ENRXADDR 0x02
#define NRF24L01_REG_SETUPAW 0x03
#define NRF24L01_REG_SETUPRETR 0x04
#define NRF24L01_REG_RFCH 0x05
#define NRF24L01_REG_RFSETUP 0x06
#define NRF24L01_REG_STATUS 0x07
#define NRF24L01_REG_OBSERVETX 0x08
#define NRF24L01_REG_CD 0x09
#define NRF24L01_REG_RXADDRP0 0x0a
#define NRF24L01_REG_RXADDRP1 0x0b
#define NRF24L01_REG_RXADDRP2 0x0c
#define NRF24L01_REG_RXADDRP3 0x0d
#define NRF24L01_REG_RXADDRP4 0x0e
#define NRF24L01_REG_RXADDRP5 0x0f
#define NRF24L01_REG_TXADDR 0x10
#define NRF24L01_REG_RXPWP0 0x11
#define NRF24L01_REG_RXPWP1 0x12
#define NRF24L01_REG_RXPWP2 0x13
#define NRF24L01_REG_RXPWP3 0x14
#define NRF24L01_REG_RXPWP4 0x15
#define NRF24L01_REG_RXPWP5 0x16
#define NRF24L01_REG_FIFOSTATUS 0x17
#define NRF24L01_REG_DYNDP 0x1c
#define NRF24L01_REG_FEATURE 0x1d

typedef struct {
	uint8_t size;
	uint8_t reg_size[];
} nrf24l01_regs;

/*
 * Register buffer
 */
typedef struct {
	uint8_t size;
	uint8_t data[NRF24L01_REG_BUF_MAX];
} nrf24l01_reg_buf;

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

void nrf24l01_read_reg(uint8_t reg, nrf24l01_reg_buf *buf);

void nrf24l01_write_reg(uint8_t reg, nrf24l01_reg_buf *buf);

void nrf24l01_init(uint32_t spi) {
	spi_init();

	spi_select();
	spi_drv_read(SPI1);

	spi_deselect();
}

void nrf24l01_read_reg(uint8_t reg, nrf24l01_reg_buf *buf) {
	spi_select();

	spi_drv_xfer(SPI1, (NRF24L01_CMD_R_REGISTER | reg));

	for (uint8_t i = 0; i < nrf24l01_reg_def.reg_size[reg]; i++) {
		buf->data[i] = spi_drv_xfer(SPI1, NRF24L01_CMD_NOP);
	}

	spi_deselect();
}

void nrf24l01_write_reg(uint8_t reg, nrf24l01_reg_buf *buf) {
	spi_select();

	spi_drv_xfer(SPI1, (NRF24L01_CMD_W_REGISTER | reg));

	for (uint8_t i = 0; i < nrf24l01_reg_def.reg_size[reg]; i++) {
		spi_drv_xfer(SPI1, buf->data[i]);
	}

	spi_deselect();
}
