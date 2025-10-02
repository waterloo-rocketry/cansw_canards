#ifndef INIT_H
#define INIT_H

#include "FreeRTOS.h"
#include "main.h" // For HAL handle declarations
#include "rocketlib/include/common.h"
#include "task.h"

// Maximum number of initialization retries before giving up
#define MAX_INIT_RETRIES 1

// Delay between initialization retries in milliseconds
#define INIT_RETRY_DELAY_MS 1000

// Task handles - defined in init.c
extern TaskHandle_t blinky_task_handle;

// Task priorities
extern const uint32_t blinky_task_priority;

// Main initialization function that sets up everything
w_status_t system_init(void);

#endif // INIT_H