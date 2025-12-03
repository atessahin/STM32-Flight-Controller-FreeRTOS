#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx.h"
#include <string.h>
#include <stdint.h>

// I2C Adresi
#define SSD1306_I2C_ADDR        0x3C

// Ekran Boyutları
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64

// Renkler
typedef enum {
	Black = 0x00,
	White = 0x01
} SSD1306_COLOR;

// Font Yapısı
typedef struct {
	const uint8_t width;
	uint8_t height;
	const uint16_t *data;
} FontDef;

// Fonksiyonlar
void ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);


extern FontDef Font_7x10;

#endif
