#include "stm32f4xx.h"
#include "system_clock.h"

void gpioConfig();
void i2cConfig();

void boardInit()
{
	systemClockConfig();
	gpioConfig();
}


void gpioConfig()
{
	RCC->AHB1ENR |= (1<<1); // Enable GPIOB
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA

	// Configure PB6 SCL and PB7 SDA for Alternate Function
	GPIOB->MODER &= ~((3<<12) | (3<<14)); // PB6, PB7 clear MODE bits
	GPIOB->MODER |=  ((2<<12) | (2<<14)); // Set to Alternate Function Mode

	// Set output type to Open-Drain
	GPIOB->OTYPER |= (1<<6) | (1<<7);

	// Configure Pull-up resistors
	GPIOB->PUPDR &= ~((3<<12) | (3<<14));
	GPIOB->PUPDR |=  (1<<12) | (1<<14); // Set to Pull-up

	// Select Alternate Function 4 AF4 for I2C1
	GPIOB->AFR[0] &= ~((0xF<<24) | (0xF<<28)); // Clear AF bits
	GPIOB->AFR[0] |=  ((4<<24) | (4<<28)); // Set AF4 for PB6/PB7
}

void i2cConfig()
{
    RCC->APB1ENR |= (1<<21); // Enable I2C1 clock

    // Software reset
    I2C1->CR1 |= (1<<15);
    I2C1->CR1 &= ~(1<<15);

    I2C1->CR1 &= ~(1<<0);  // Disable Peripheral

    // Set timing registers for 100kHz Standard Mode
    I2C1->CR2 = 42;    // PCLK1 frequency in MHz
    I2C1->CCR = 210;   // Clock Control Register
    I2C1->TRISE = 43;  // Rise Time Register

    I2C1->CR1 |= (1<<10);  // Enable Acknowledge
    I2C1->CR1 |= (1<<0);   // Enable Peripheral

    for(volatile int i=0;i<50000;i++);
}
