#ifndef __SSD1306_H__
#define __SSD1306_H__

void ssd1306_init(uint8_t i2c, uint8_t addr, uint8_t mux_rat);

void ssd1306_transmit_buffer(uint8_t i2c, uint8_t addr, uint8_t *buf, uint16_t size);

void ssd1306_set_col_addr(uint8_t i2c, uint8_t addr, uint8_t start, uint8_t end);

void ssd1306_set_page_addr(uint8_t i2c, uint8_t addr, uint8_t start, uint8_t end);

#endif /* __SSD1306_H__ */
