#include "application/init/init.h"
#include "application/blinky/blinky.h"
#include "drivers/adc/adc.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"
#include "stm32h7xx_hal.h"
// Add these includes for hardware handles
#include "FreeRTOS.h"
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1
#include "i2c.h" // For hi2c2, hi2c4
#include "task.h"
#include "usart.h" // For huart4, huart8

// Initialize task handles to NULL
TaskHandle_t blinky_task_handle = NULL;

// Task priorities
const uint32_t blinky_task_priority = configMAX_PRIORITIES - 1;

// Main initialization function
w_status_t system_init(void) {
    // hotfix: allow time for .... stuff ?? ... before init.
    // without this, the uart DMA change made proc freeze upon power cycle.
    // probably because movella triggers before its ready
    vTaskDelay(500);

    // Create FreeRTOS tasks
    xTaskCreate(blinky_task, "blinky", 256, NULL, blinky_task_priority, &blinky_task_handle);
    return W_SUCCESS;
}
