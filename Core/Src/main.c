#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"


#include "board_init.h"
#include "delay.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "flight_core.h"
#include "ssd1306.h"

#include <stdint.h>
#include <stdio.h>


SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t mpuInterruptSem;
QueueHandle_t Raw_Data_Queue;
QueueHandle_t Display_Queue;


TaskHandle_t sensor;
TaskHandle_t fusion;
TaskHandle_t display;


extern Attitude_t current_attitude;


void SensorTask(void *argument);
void FusionTask(void *argument);
void DisplayTask(void *argument);


int main(void)
{

    boardInit();
    delayInit();

    i2cConfig();
    delay_ms(50);


    ssd1306_Init();
    ssd1306_Fill(Black);

    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("INITIALIZING...", Font_7x10, White);
    ssd1306_SetCursor(10, 30);
    ssd1306_WriteString("DO NOT MOVE!", Font_7x10, White);
    ssd1306_UpdateScreen();

    delay_ms(1000);



    mpu6050_init();
    delay_ms(100);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 20);
    ssd1306_WriteString("CALIBRATING...", Font_7x10, White);
    ssd1306_UpdateScreen();

    mpu6050_calibrate_gyro();



    bmp280_init();
    bmp280_calibrate_basic_altitude();



    ssd1306_Fill(Black);
    ssd1306_SetCursor(5, 10);
    ssd1306_WriteString("SYSTEM READY!", Font_7x10, White);
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("READY FOR TAKEOFF!", Font_7x10, White);
    ssd1306_UpdateScreen();
    delay_ms(1000);



    i2cMutex = xSemaphoreCreateMutex();
    Raw_Data_Queue = xQueueCreate(5, sizeof(RawData_t));
    Display_Queue  = xQueueCreate(1, sizeof(Attitude_t));

    xTaskCreate(SensorTask, "ReadSensors",  512, NULL, 3, &sensor); // High
    xTaskCreate(FusionTask, "SensorFusion", 512, NULL, 2, &fusion); // Medium
    xTaskCreate(DisplayTask,"OLED_Screen",  1024, NULL, 1, &display); // Low


    vTaskStartScheduler();

    while(1) { }
}

void SensorTask(void *argument)
{
    RawData_t localData;
    uint32_t now;
    uint8_t int_status = 0;
    int baro_divider = 0;
    float raw_altitude = 0.0f;
    static float filtered_altitude = 0.0f;
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // Task runs every 4ms (250Hz)
    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelay(pdMS_TO_TICKS(100)); // Initial task delay

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait for the next cycle

        if(xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) // Lock the I2C bus
        {
            // Read MPU6050 Data Ready Status
            if(i2cRead(MPU6050_ADDR, INT_STATUS, 1, &int_status) == 0) // Check I2C success (0)
            {
                if(int_status & 0x01)
                {
                    now = millis();
                    localData.timestamp = now;


                    mpu6050_read_accel(&localData.ax, &localData.ay, &localData.az); // Read Accel
                    mpu6050_read_gyro(&localData.gx, &localData.gy, &localData.gz); // Read Gyro


                    localData.altitude = read_altitude(); // Read raw altitude

                    baro_divider++;

                    if(baro_divider >= 10)
                    {
                        baro_divider = 0;


                        raw_altitude = read_altitude();

                        if(filtered_altitude == 0.0f)
                        {
                            filtered_altitude = raw_altitude; // Initialize filter
                        }
                        else
                        {
                            filtered_altitude = (filtered_altitude * 0.90f) + (raw_altitude * (1.0f - 0.90f));
                        }
                    }

                    localData.altitude = filtered_altitude; // Use filtered altitude
                    xQueueSend(Raw_Data_Queue, &localData, 0); // Send data to FusionTask
                }
            }
            else
            	i2c_software_reset();


            xSemaphoreGive(i2cMutex);
        }
    }
}
void FusionTask(void *argument)
{
    RawData_t localData;
    float dt;
    vTaskDelay(pdMS_TO_TICKS(100));
    while(1)
    {
        // Wait for sensor data indefinitely
        if(xQueueReceive(Raw_Data_Queue, &localData, portMAX_DELAY) == pdTRUE)
        {
            dt = calculate_dt(localData.timestamp);

            update_attitude(localData.ax, localData.ay, localData.az,
                            localData.gx, localData.gy,
                            localData.altitude, dt);

            xQueueSend(Display_Queue, &current_attitude, portMAX_DELAY); // Send result to DisplayTask
        }
    }
}
void DisplayTask(void *argument)
{
    Attitude_t localData;

    char buf_pitch[16];
    char buf_roll[16];
    char buf_alt[16];

    int refresh_counter = 0;
    vTaskDelay(pdMS_TO_TICKS(100)); // Initial task delay
    while(1)
    {
        // Wait for filtered data
        if(xQueueReceive(Display_Queue, &localData, portMAX_DELAY) == pdTRUE)
        {
            refresh_counter++;

            if(refresh_counter >= 10) // Update display slower
            {
                refresh_counter = 0;


                sprintf(buf_pitch, "P: %5.1f", localData.pitch);
                sprintf(buf_roll,  "R: %5.1f", localData.roll);
                sprintf(buf_alt,   "A: %4.1fm", localData.altitude);

                xSemaphoreTake(i2cMutex, portMAX_DELAY);

                // Write to OLED buffer
                ssd1306_Fill(Black);

                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString(buf_pitch, Font_7x10, White);

                ssd1306_SetCursor(0, 15);
                ssd1306_WriteString(buf_roll, Font_7x10, White);

                ssd1306_SetCursor(0, 45);
                ssd1306_WriteString(buf_alt, Font_7x10, White);

                ssd1306_UpdateScreen();

                xSemaphoreGive(i2cMutex);
            }
        }
    }
}

void Error_Handler(void)
{
    __disable_irq(); // Disable all interrupts
    while(1); // Infinite loop on error
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while(1); // Infinite loop on assertion failure
}
#endif

