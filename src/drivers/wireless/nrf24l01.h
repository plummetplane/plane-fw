#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#define NRF24L01_MODE_TX 0x00
#define NRF24L01_MODE_RX 0x01

#define NRF24L01_PAYLOAD_MAX 32

typedef struct {
	uint8_t size;
	uint8_t data[NRF24L01_PAYLOAD_MAX];
} nrf24l01_payload;

/*
 * Initialize SPI bus and configure the device
 * spi - SPI bus the device is connected to
 * gpioport. gpios - EN PIN connection
 * mode - rx/tx
 */
void nrf24l01_init(uint32_t spi, uint32_t gpioport, uint16_t gpios, uint8_t mode);

/*
 * Enable CRC
 */
void nrf24l01_enable_crc(void);

/*
 * Disable CRC
 */
void nrf24l01_disable_crc(void);

/*
 * Power up
 */
void nrf24l01_enable_power(void);

/*
 * Power down
 */
void nrf24l01_disable_power(void);

/*
 * Set mode (RX/TX)
 * (ref.: NRF24L01_MODE_TX, NRF24L01_MODE_RX)
 */
void nrf24l01_mode(uint8_t mode);

/*
 * Transmit/receive data
 */
int nrf24l01_transmit(nrf24l01_payload *payload);
int nrf24l01_receive(nrf24l01_payload *payload);

/*
 * setup callback functions for the irq handler
 */
void nrf24l01_rxdr_callback(void (*callback_func)(nrf24l01_payload payload));
void nrf24l01_txds_callback(void (*callback_func)(void));
void nrf24l01_maxrt_callback(void (*callback_func)(void));

/*
 * IRQ handler
 * EXTI must be configured and call this function
 *	can be setup to trigger an action on receive/transmit/failed transmit events
 *	(see callback funcs)
 */
void nrf24l01_irq(void);

#endif /* __NRF24L01_H__ */
