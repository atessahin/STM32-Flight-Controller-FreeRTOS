#include "i2c_bus.h"
#include "stm32f4xx.h"


#define I2C_TIMEOUT 50000

// RETURN: 0 = Success, 1 = Error
uint8_t i2cWrite(uint8_t deviceAddr, uint8_t registerAddr, uint8_t *data, uint16_t length)
{
    volatile uint32_t timeout;

    // Check Bus Busy status
    timeout = I2C_TIMEOUT;
    while(I2C1->SR2 & (1<<1))
    {
        if(--timeout == 0) return 1; //Bus stayed busy
    }

    // Send Start Bit
    I2C1->CR1 |= (1<<8);

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (1<<0))) // Wait for SB
    {
        if(--timeout == 0) return 1; //SB timeout
    }

    // Send Slave Address
    I2C1->DR = deviceAddr << 1;

    timeout = I2C_TIMEOUT;
    // Wait for ADDR or AF
    while( !((I2C1->SR1 & (1<<1)) || (I2C1->SR1 & (1<<10))) )
    {
        if(--timeout == 0) return 1; // ADDR/AF timeout
    }

    // NACK Check
    if(I2C1->SR1 & (1<<10))
    {
         I2C1->SR1 &= ~(1<<10); // Clear AF flag
         I2C1->CR1 |= (1<<9);   // Send STOP
         return 1;              // ERROR: NACK detected
    }

    volatile uint32_t temp = I2C1->SR1;
    temp = I2C1->SR2; // Clear ADDR flag by reading SR1 then SR2

    //Send Register Address
    I2C1->DR = registerAddr;

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (1<<7))) // Wait for TXE
    {
        if(--timeout == 0) return 1; // TXE timeout
    }

    // Send Data Payload
    for (uint16_t i= 0; i < length; i++)
    {
        I2C1->DR = data[i];

        timeout = I2C_TIMEOUT;
        while(!(I2C1->SR1 & (1<<7))) // Wait for TXE
        {
            if(--timeout == 0) return 1;
        }
    }

    // Wait for BTF (Byte Transfer Finished)
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (1<<2)))
    {
        if(--timeout == 0) return 1;
    }

   // Send Stop Bit
    I2C1->CR1 |= (1<<9);

    return 0;
}

// RETURN: 0 = Success, 1 = Error (Timeout or NACK)
uint8_t i2cRead(uint8_t deviceAddr, uint8_t registerAddr, uint8_t length, uint8_t *buffer)
{
    volatile uint32_t timeout;

    //Check Bus Busy status
    timeout = I2C_TIMEOUT;
    while(I2C1->SR2 & (1<<1))
    {
        if(--timeout == 0) return 1;
    }

    // Send Start Bit
    I2C1->CR1 |= (1<<8);

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (1<<0))) // Wait for SB
    {
        if(--timeout == 0) return 1;
    }

    //Send Slave Address
    I2C1->DR = deviceAddr << 1;

    timeout = I2C_TIMEOUT;
    while( !((I2C1->SR1 & (1<<1)) || (I2C1->SR1 & (1<<10))) ) // Wait for ADDR or AF
    {
        if(--timeout == 0) return 1;
    }

    if(I2C1->SR1 & (1<<10)) // NACK Check
    {
        I2C1->SR1 &= ~(1<<10);
        I2C1->CR1 |= (1<<9);
        return 1;
    }

    volatile uint32_t temp = I2C1->SR1;
    temp = I2C1->SR2; // Clear ADDR flag

    //Send Register Address
    I2C1->DR = registerAddr;

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (1<<7))) // TXE
    {
        if(--timeout == 0) return 1;
    }

    // 5. Send Restart
    I2C1->CR1 |= (1<<8);

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & (1<<0))) // Wait for SB
    {
        if(--timeout == 0) return 1;
    }

    //Send Slave Address
    I2C1->DR = (deviceAddr << 1) | 1;

    timeout = I2C_TIMEOUT;
    while( !((I2C1->SR1 & (1<<1)) || (I2C1->SR1 & (1<<10))) ) // Wait for ADDR or AF
    {
        if(--timeout == 0) return 1;
    }

    if(I2C1->SR1 & (1<<10)) // NACK Check
    {
        I2C1->SR1 &= ~(1<<10);
        I2C1->CR1 |= (1<<9);
        return 1;
    }

    temp = I2C1->SR1;
    temp = I2C1->SR2; // Clear ADDR flag

    //Data Read Loop
    for (int i= 0; i < length; i++)
    {
        // Handle last byte read differently (NACK + STOP)
        if(i == length - 1)
        {
            I2C1->CR1 &= ~(1<<10); // Clear ACK bit (send NACK on last byte)
            I2C1->CR1 |= (1<<9);   // Send STOP after receiving last byte
        }
        else
        {
            I2C1->CR1 |= (1<<10);  // Send ACK
        }

        timeout = I2C_TIMEOUT;
        while(!(I2C1->SR1 & (1<<6))) // Wait for RxNE (Receive Buffer Not Empty)
        {
            if(--timeout == 0) return 1; //RxNE timeout
        }
        buffer[i] = I2C1->DR; // Read the data byte
    }

    return 0;
}


void i2c_software_reset(void)
{

    // Reset  1
    I2C1->CR1 |= (1<<15);

    // Reset  0
    I2C1->CR1 &= ~(1<<15);


    I2C1->CR1 &= ~(1<<0);  // PE=0 (Disable)
    I2C1->CR1 |= (1<<0);   // PE=1 (Enable)
}

