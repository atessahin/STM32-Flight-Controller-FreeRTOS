#include "flight_core.h"

float dt;
static FusionState_t fusion_state;
Attitude_t current_attitude;


static float smooth_ax = 0, smooth_ay = 0, smooth_az = 0;

// Calculates time difference between measurements and updates state
float calculate_dt(uint32_t currentTime)
{
    float diff_ms = currentTime - fusion_state.prev_time; // Time elapsed in milliseconds
    dt = diff_ms / 1000.0; // Convert to seconds
    fusion_state.prev_time = currentTime;
    return dt;
}

// Runs the Complementary Filter and updates global attitude structure
void update_attitude(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, float altitude, float delta_T)
{
    // 1. LOW PASS FILTER (Smoothes out high-frequency noise from Accelerometer)
    if(smooth_ax == 0) {
        smooth_ax = ax; smooth_ay = ay; smooth_az = az;
    } else {
        smooth_ax = smooth_ax * (1.0f - ACCEL_LPF) + ax * ACCEL_LPF;
        smooth_ay = smooth_ay * (1.0f - ACCEL_LPF) + ay * ACCEL_LPF;
        smooth_az = smooth_az * (1.0f - ACCEL_LPF) + az * ACCEL_LPF;
    }

    // 2. GYROSCOPE CALCULATION (Convert raw data to degrees/second)
    float gyro_x_vel = gx / GYRO_SENSITIVITY;
    float gyro_y_vel = gy / GYRO_SENSITIVITY;

    // 3. PREDICTION (Predict current angle using previous angle + gyro change)
    float prediction_pitch = fusion_state.pitch_angle + (gyro_x_vel * delta_T);
    float prediction_roll  = fusion_state.roll_angle  + (gyro_y_vel * delta_T);

    // 4. ACCELEROMETER ANGLE CALCULATION
    // Accel Roll: Uses Y (side) and Z (up/down)
    float accel_roll_deg  = atan2(smooth_ay, sqrt(smooth_ax*smooth_ax + smooth_az*smooth_az)) * RAD_TO_DEG;
    // Accel Pitch: Uses X (forward/back) and Z (up/down)
    float accel_pitch_deg = atan2(-smooth_ax, sqrt(smooth_ay*smooth_ay + smooth_az*smooth_az)) * RAD_TO_DEG;

    // 5. SENSOR FUSION (Complementary Filter implementation)
    // Pitch: Weight Gyro Prediction (high frequency) + Accel Reference (low frequency)
    fusion_state.pitch_angle = (ALPHA * prediction_pitch) + ((1.0f - ALPHA) * accel_pitch_deg);
    // Roll: Weight Gyro Prediction (high frequency) + Accel Reference (low frequency)
    fusion_state.roll_angle  = (ALPHA * prediction_roll)  + ((1.0f - ALPHA) * accel_roll_deg);

    // Update global output structure
    current_attitude.pitch = fusion_state.pitch_angle;
    current_attitude.roll  = fusion_state.roll_angle;
    current_attitude.altitude = altitude;
}
