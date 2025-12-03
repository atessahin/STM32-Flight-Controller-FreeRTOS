#ifndef I2C_BUS_H
#define I2C_BUS_H
#include <stdint.h>

// 0 = Başarılı, 1 = Hata
uint8_t i2cWrite(uint8_t deviceAddr, uint8_t registerAddr, uint8_t *data, uint16_t length);
uint8_t i2cRead(uint8_t deviceAddr, uint8_t registerAddr, uint8_t length, uint8_t *buffer);
void i2c_software_reset(void);
#endif
