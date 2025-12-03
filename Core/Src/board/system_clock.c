#include "stm32f4xx.h"
void systemClockConfig()
{
	RCC->CR |=(1<<16);
	while(!(RCC->CR & (1<<17)));

	FLASH->ACR|=(3<<1)|(1<<8)|(1<<9)|(1<<10);

	RCC->APB1ENR|=(1<<28);
	PWR->CR |=(3U<<14);


	RCC->CR &=~(1<<24);
	while(RCC->CR & (1<<25))
		;

	//PLL: HSE/M * N/P = 8MHz/8 * 168/2 = 84mhz
	RCC->PLLCFGR = (8 << 0) |       // PLLM = 8
                   (168 << 6) |     // PLLN = 168
                   (0 << 16) |      // PLLP = 2
                   (1 << 22);       // PLLSRC = HSE


	RCC->CR |= (1 << 24);           // PLL ON
	while(!(RCC->CR & (1 << 25)));  // Wait until PLL is ready


	RCC->CFGR&=~(15U<<4);           // AHB prescaler = 1, 84MHz
	RCC->CFGR|=(4U<<10);            // APB1 prescaler = 2, 84MHz/2 = 42MHz (max)
	RCC->CFGR&=~(7U<<13);           // APB2 prescaler = 1, 84MHz


	// Select PLL as system clock
	RCC->CFGR |= (2 << 0);          // SW = PLL
	while((RCC->CFGR & (3 << 2)) != (2 << 2))  // Wait until PLL is system clock
		;

	SystemCoreClockUpdate();
}
