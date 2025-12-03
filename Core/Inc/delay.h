#ifndef DELAY_H
#define DELAY_H

#include "stm32f4xx.h"
#include <stdint.h>

extern volatile uint32_t uwTick;

void delayInit(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t millis(void);

#endif
