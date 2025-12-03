#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

// SystemCoreClock is defined externally
extern uint32_t SystemCoreClock;

// DWT initialization
void delayInit(void)
{
    // Enable DWT tracing feature
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0; // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable cycle counter
}

// Millisecond delay function
void delay_ms(uint32_t ms)
{
    // If FreeRTOS Scheduler is running, use vTaskDelay
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        // If Scheduler has not started
        delay_us(ms * 1000);
    }
}

// Microsecond delay function (DWT busy-wait)
void delay_us(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT;
    // Calculate required cycles: us * (Cycles per second / 1,000,000)
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - startTick) < delayTicks)
    {
        // Blocking wait loop
    }
}

// Returns FreeRTOS time in milliseconds
uint32_t millis(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    else
    {
        return 0; // Return 0 if RTOS is not running
    }
}
