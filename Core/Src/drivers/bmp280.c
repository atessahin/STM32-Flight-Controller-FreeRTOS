#include "i2c_bus.h"
#include "bmp280.h"


static int32_t t_fine;
static float reference_pressure = 1013.25; // Sea Level Pressure
calib_data bmp280_calib;

// Wrapper function to write a register
void Bmp280_WriteReg(uint8_t reg, uint8_t value, uint8_t length) {

    i2cWrite(BMP280_ADDR, reg, &value, length);
}

// BMP280 Initialization sequence
void bmp280_init()
{
	uint8_t whoAmIBuffer;
	i2cRead(BMP280_ADDR, CHIP_ID, 1, &whoAmIBuffer);

	if(whoAmIBuffer != 0x60) // Check if device is BME280
	{
	    return;
	}

	Bmp280_WriteReg(RESET, 0xB6,1); // Software Reset

	delay_ms(5); // Wait for sensor to reboot


	bmp280_read_calibration(); // Read factory calibration data

	uint8_t ctrl_meas_adjustment;

    // Read current CTRL_MEAS register value
	i2cRead(BMP280_ADDR, CTRL_MEAS, 1, &ctrl_meas_adjustment);

    // Set configuration: Temp Oversampling x1, Press Oversampling x4, Mode: Normal (3)
	ctrl_meas_adjustment = (1 << 5) | (4 << 2) | (3);

	Bmp280_WriteReg(CTRL_MEAS, ctrl_meas_adjustment,1);

    // Set config register
    uint8_t config_adjustment =(0 << 5) | (2 << 2);

    Bmp280_WriteReg(CONFIG, config_adjustment,1);

}

// Read the 26 bytes of non-volatile calibration data from the sensor
void bmp280_read_calibration(void)
{
    uint8_t buffer[26];    // 0x88–0xA1 calibration registers

    // Read 26 bytes in a block
    i2cRead(BMP280_ADDR, CALIB_START, 26, buffer);


    bmp280_calib.dig_T1 = (uint16_t)((buffer[1] << 8) | buffer[0]);
    bmp280_calib.dig_T2 = (int16_t)((buffer[3] << 8) | buffer[2]);
    bmp280_calib.dig_T3 = (int16_t)((buffer[5] << 8) | buffer[4]);


    bmp280_calib.dig_P1 = (uint16_t)((buffer[7] << 8) | buffer[6]);
    bmp280_calib.dig_P2 = (int16_t)((buffer[9] << 8) | buffer[8]);
    bmp280_calib.dig_P3 = (int16_t)((buffer[11] << 8) | buffer[10]);
    bmp280_calib.dig_P4 = (int16_t)((buffer[13] << 8) | buffer[12]);
    bmp280_calib.dig_P5 = (int16_t)((buffer[15] << 8) | buffer[14]);
    bmp280_calib.dig_P6 = (int16_t)((buffer[17] << 8) | buffer[16]);
    bmp280_calib.dig_P7 = (int16_t)((buffer[19] << 8) | buffer[18]);
    bmp280_calib.dig_P8 = (int16_t)((buffer[21] << 8) | buffer[20]);
    bmp280_calib.dig_P9 = (int16_t)((buffer[23] << 8) | buffer[22]);
}

// Read raw 20-bit temperature data
int32_t bmp280_read_raw_temp()
{
    uint8_t buffer[3];
    int32_t T_raw;

    // Read 3 bytes starting from TEMP_MSB
    i2cRead(BMP280_ADDR, TEMP_MSB, 3, buffer);

    // Combine 20-bit data (MSB[7:0] << 12 | LSB[7:0] << 4 | XLSB[7:4] >> 4)
    T_raw = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);

    return T_raw;
}

// Read raw 20-bit pressure data
int32_t bmp280_read_raw_press()
{
    uint8_t buffer[3];
    int32_t P_raw;

    // Read 3 bytes starting from PRESS_MSB
    i2cRead(BMP280_ADDR, PRESS_MSB, 3, buffer);

    // Combine 20-bit data
    P_raw = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);

    return P_raw;
}

// Calculate compensated temperature and set t_fine
int32_t calculate_T(int32_t rawTemp)
{
    int32_t var1, var2, T;

    // Bosch compensation algorithm (Fixed point arithmetic)
    var1 = ((((rawTemp >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) *
             (int32_t)bmp280_calib.dig_T2) >> 11;

    var2 = (((((rawTemp >> 4) - (int32_t)bmp280_calib.dig_T1) *
             ((rawTemp >> 4) - (int32_t)bmp280_calib.dig_T1)) >> 12) *
             (int32_t)bmp280_calib.dig_T3) >> 14;

    t_fine = var1 + var2; // Store global compensation value

    T = (t_fine * 5 + 128) >> 8;

    return T;
}

// Calculate compensated pressure (Requires t_fine to be set by calculate_T)
int32_t calculate_P(int32_t rawPress)
{
    int64_t var1, var2, p;

    // Bosch compensation algorithm
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = ((((int64_t)1 << 47) + var1)) *
           ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;
    }

    p = 1048576 - rawPress;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)bmp280_calib.dig_P9) *
          (p >> 13) * (p >> 13)) >> 25;

    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);

    return (int32_t)p;
}

// Simple averaging for setting the ground reference pressure
void bmp280_calibrate_basic_altitude()
{
	delay_ms(500); // Wait for stabilization
	float sum_pressure=0;
	int sample_count=50;

	for (int i = 0;i < sample_count; i++) // Loop 50 times
	{
		int32_t raw_t = bmp280_read_raw_temp(); // Read temperature
		calculate_T(raw_t);

		int32_t raw_p= bmp280_read_raw_press(); // Read pressure
		int32_t real_p=calculate_P(raw_p);
		float p_hpa = (float)real_p / 256.0f / 100.0f;// Convert (Pa * 256) -> hPa

		sum_pressure+=p_hpa; // Sum the pressure readings

		delay_ms(10); // Wait between readings
	}

	reference_pressure=sum_pressure/sample_count; // Set global baseline pressure
}

// Read current altitude based on sea level pressure
float read_altitude()
{

	   int32_t raw_temp = bmp280_read_raw_temp();
	   calculate_T(raw_temp); // Set t_fine based on temp

	   int32_t raw_press = bmp280_read_raw_press();
       int32_t pressure = calculate_P(raw_press); // Get compensated pressure

    float pressure_hPa = (float)pressure / 256.0f / 100.0f; // Convert to hPa


    float altitude = 44330.0f * (1.0f - powf(pressure_hPa / reference_pressure, 0.1903f));

   return altitude;
}
