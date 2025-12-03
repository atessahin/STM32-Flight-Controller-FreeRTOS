#include "i2c_bus.h"
#include "mpu6050.h"
#include "delay.h"


static int16_t gyro_x_offset =0;
static int16_t gyro_y_offset =0;
static int16_t gyro_z_offset =0;

// Wrapper function to write a single register value
void MPU_WriteReg(uint8_t reg, uint8_t value,uint8_t length) {

    i2cWrite(MPU6050_ADDR, reg, &value, length);
}

// Read and return WHO_AM_I register value (Device ID check)
uint8_t mpu6050_whoami(void)
{
	uint8_t whoAmIBuffer;
	i2cRead(MPU6050_ADDR, WHO_AM_I, 1, &whoAmIBuffer);
	return whoAmIBuffer;
}

// MPU6050 Initialization sequence
void mpu6050_init(void)
{
	MPU_WriteReg(PWR_MGMT_1, 0x03,1); // Set clock source (PLL with Z axis gyro reference)
	MPU_WriteReg(CONFIGM, 0x03,1); // Set DLPF (Digital Low Pass Filter) to 44Hz
	MPU_WriteReg(ACCEL_CONFIG, 0x00,1); // Set Accel scale to +/- 2g
	MPU_WriteReg(GYRO_CONFIG, 0x00,1); // Set Gyro scale to +/- 250 deg/s
}


void mpu6050_calibrate_gyro()
{
	int32_t x_sum=0;
	int32_t y_sum=0;
	int32_t z_sum=0;
	int16_t gx,gy,gz;

	for (int i = 0; i < 200;  i++) // Sample 200 times
	{
		 uint8_t calibrateBuff2[6];
		 // Read 6 bytes starting from GYRO_XOUT_H
		 i2cRead(MPU6050_ADDR,GYRO_XOUT_H, 6, calibrateBuff2);

		 // Convert high/low bytes to 16-bit signed integer
		 gx = (int16_t)((calibrateBuff2[0] << 8) | calibrateBuff2[1]);
		 gy = (int16_t)((calibrateBuff2[2] << 8) | calibrateBuff2[3]);
		 gz = (int16_t)((calibrateBuff2[4] << 8) | calibrateBuff2[5]);

		 x_sum+=gx;
		 y_sum+=gy;
		 z_sum+=gz;

		 delay_ms(2);
	}

    // Calculate the average offset
	gyro_x_offset=x_sum/200;
	gyro_y_offset=y_sum/200;
	gyro_z_offset=z_sum/200;

}


void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t collabBuff[6];

    // Read 6 bytes starting from ACCEL_XOUT_H
    i2cRead(MPU6050_ADDR, ACCEL_XOUT_H, 6, collabBuff);

    // Convert high/low bytes and store via pointer
    *ax = (int16_t)((collabBuff[0] << 8) | collabBuff[1]);
    *ay = (int16_t)((collabBuff[2] << 8) | collabBuff[3]);
    *az = (int16_t)((collabBuff[4] << 8) | collabBuff[5]);
}

void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
	 uint8_t collabBuff2[6];
	 int16_t gx_raw,gy_raw,gz_raw;


	 i2cRead(MPU6050_ADDR,GYRO_XOUT_H, 6, collabBuff2);

	 // Convert high/low bytes
	 gx_raw = (int16_t)((collabBuff2[0] << 8) | collabBuff2[1]);
	 gy_raw = (int16_t)((collabBuff2[2] << 8) | collabBuff2[3]);
	 gz_raw = (int16_t)((collabBuff2[4] << 8) | collabBuff2[5]);

	 // Apply calibration offset
	 *gx = gx_raw - gyro_x_offset;
	 *gy = gy_raw - gyro_y_offset;
	 *gz = gz_raw - gyro_z_offset;
}
