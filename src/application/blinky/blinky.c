/*
 * blinky.c
 * Demo task
 */

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "blinky.h"

// FreeRTOS task functions always have the function signature  `void taskname(void *argument)`
void blinky_task(void *args) {
    (void)args;
    // This loop repeats for infinity and must never exit! (tasks never end)
    for (;;) {
        // Write 0 to the LED pin (turn it on)
        // HAL functions are provided by STM32 to easily interact with hardware
        // LEDs are wired to the STM32's GPIO pins - this one specifically is wired to GPIO_E pin 10
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);

        // FreeRTOS function that puts this task on pause for 500 ticks (0.5 sec)
        // Other tasks will run while this task is paused
        vTaskDelay(500);

        // Write 1 to the LED pin (turn it off)
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);

        // Pause task for 500 ticks (0.5 sec)
        vTaskDelay(500);
    }
}
