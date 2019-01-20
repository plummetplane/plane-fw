#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#define NRF24L01_PAYLOAD_MAX 32

typedef struct {
	uint8_t size;
	uint8_t data[NRF24L01_PAYLOAD_MAX];
} nrf24l01_payload;

void nrf24l01_init(uint32_t spi);

int nrf24l01_send(nrf24l01_payload *payload);

#endif /* __NRF24L01_H__ */
