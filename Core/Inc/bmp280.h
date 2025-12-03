#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "delay.h"

#define BMP280_ADDR 0x76

#define CHIP_ID      0xD0
#define RESET        0xE0
#define STATUS       0xF3

#define CTRL_MEAS    0xF4
#define CONFIG       0xF5

#define PRESS_MSB    0xF7
#define PRESS_LSB    0xF8
#define PRESS_XLSB   0xF9

#define TEMP_MSB     0xFA
#define TEMP_LSB     0xFB
#define TEMP_XLSB    0xFC

#define CALIB_START  0x88
#define CALIB_END    0xA1

// Calibration struct
typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} calib_data;

// Function prototypes
void bmp280_init(void);
void bmp280_read_calibration(void);

int32_t bmp280_read_raw_temp(void);
int32_t bmp280_read_raw_press(void);

int32_t calculate_T(int32_t rawTemp);
int32_t calculate_P(int32_t rawPress);

float read_altitude(void);
void bmp280_calibrate_basic_altitude(void);

#endif // BMP280_H
