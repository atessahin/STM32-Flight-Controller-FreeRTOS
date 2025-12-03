#ifndef FLIGHT_CORE_H
#define FLIGHT_CORE_H

#include <stdint.h>
#include <stdio.h>
#include "delay.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define GYRO_SENSITIVITY 131.0f // ±250dps modu için MPU6050
#define ALPHA 0.98f             // Filtre katsayısı (Jiroskop Güveni)
#define ACCEL_LPF 0.15f

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float altitude;
    uint32_t timestamp;
} RawData_t;

typedef struct
{
    float pitch;
    float roll;
    float altitude;
} Attitude_t;

typedef struct
{
    float pitch_angle;
    float roll_angle;
    float gyro_x_rate;
    float gyro_y_rate;
    uint32_t prev_time;
} FusionState_t;

float calculate_dt(uint32_t currentTime);
void update_attitude(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, float altitude, float delta_T);

#endif
