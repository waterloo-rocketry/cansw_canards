#include "application/init/init.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/ekf.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
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
TaskHandle_t log_task_handle = NULL;
TaskHandle_t estimator_task_handle = NULL;
TaskHandle_t can_handler_handle_tx = NULL;
TaskHandle_t can_handler_handle_rx = NULL;
TaskHandle_t health_checks_task_handle = NULL;
TaskHandle_t controller_task_handle = NULL;
TaskHandle_t flight_phase_task_handle = NULL;
TaskHandle_t imu_handler_task_handle = NULL;
TaskHandle_t movella_task_handle = NULL;

// Task priorities
// flight phase must have highest priority to preempt everything else
const uint32_t flight_phase_task_priority = configMAX_PRIORITIES - 1;
// prioritize not missing injectorvalveopen msg
// TODO: could dynamically reduce this priority after flight starts?
const uint32_t can_handler_rx_priority = 45;
// in general, prioritize consumers (estimator) over producers (imus) to avoid congestion
const uint32_t can_handler_tx_priority = 40;
const uint32_t controller_task_priority = 30;
const uint32_t estimator_task_priority = 25;
const uint32_t imu_handler_task_priority = 20;
const uint32_t movella_task_priority = 20;
const uint32_t log_task_priority = 15;
// should be lowest prio above default task
const uint32_t health_checks_task_priority = 10;

// Initialize a function with retry logic
w_status_t init_with_retry(w_status_t (*init_fn)(void)) {
    w_status_t status;
    uint32_t retry_count = 0;

    while (retry_count < MAX_INIT_RETRIES) {
        status = init_fn();

        if (W_SUCCESS == status) {
            return W_SUCCESS;
        }

        retry_count++;
        if (retry_count < MAX_INIT_RETRIES) {
            HAL_Delay(INIT_RETRY_DELAY_MS);
        }
    }

    return W_FAILURE;
}

// Initialize a function with retry logic and parameter
w_status_t init_with_retry_param(w_status_t (*init_fn)(void *), void *param) {
    w_status_t status;
    uint32_t retry_count = 0;

    while (retry_count < MAX_INIT_RETRIES) {
        status = init_fn(param);

        if (status == W_SUCCESS) {
            return W_SUCCESS;
        }

        retry_count++;
        if (retry_count < MAX_INIT_RETRIES) {
            HAL_Delay(INIT_RETRY_DELAY_MS);
        }
    }

    return W_FAILURE;
}

// Main initialization function
w_status_t system_init(void) {
    // hotfix: allow time for .... stuff ?? ... before init.
    // without this, the uart DMA change made proc freeze upon power cycle.
    // probably because movella triggers before its ready
    vTaskDelay(500);

    // INIT NON-CRITICAL MODULES; try to do logger first
    w_status_t non_crit_status = sd_card_init();
    non_crit_status |= log_init();
    if (non_crit_status != W_SUCCESS) {
        // Log non-critical initialization failure
        log_text(10, "init", "Non-crit init fail 0x%lx", non_crit_status);
    }

    w_status_t status = W_SUCCESS;

    // INIT REQUIRED MODULES
    status |= gpio_init();
    status |= i2c_init(I2C_BUS_2, &hi2c2, 0);
    status |= i2c_init(I2C_BUS_4, &hi2c4, 0);
    status |= uart_init(UART_DEBUG_SERIAL, &huart4, 100);
    status |= uart_init(UART_MOVELLA, &huart8, 100);
    status |= adc_init(&hadc1);
    status |= estimator_init();
    status |= health_check_init();
    status |= init_with_retry(altimu_init);
    status |= init_with_retry(movella_init);
    status |= init_with_retry(flight_phase_init);
    status |= init_with_retry(imu_handler_init);
    status |= init_with_retry_param((w_status_t(*)(void *))can_handler_init, &hfdcan1);
    status |= init_with_retry(controller_init);
    status |= init_with_retry(ekf_init);

    // cannot continue if any of the above fail
    if (status != W_SUCCESS) {
        // Log critical initialization failure - specific modules should have logged details
        log_text(10, "init", "crit init fail (status: 0x%lx).", status);
        return status;
    }

    // Create FreeRTOS tasks
    BaseType_t task_status = pdTRUE;

    task_status &= xTaskCreate(
        flight_phase_task,
        "flight phase",
        256,
        NULL,
        flight_phase_task_priority,
        &flight_phase_task_handle
    );

    task_status &= xTaskCreate(
        health_check_task,
        "health",
        512,
        NULL,
        health_checks_task_priority,
        &health_checks_task_handle
    );

    task_status &= xTaskCreate(
        imu_handler_task,
        "imu handler",
        512,
        NULL,
        imu_handler_task_priority,
        &imu_handler_task_handle
    );

    task_status &= xTaskCreate(
        can_handler_task_rx,
        "can handler rx",
        256,
        NULL,
        can_handler_rx_priority,
        &can_handler_handle_rx
    );

    task_status &= xTaskCreate(
        can_handler_task_tx,
        "can handler tx",
        256,
        NULL,
        can_handler_tx_priority,
        &can_handler_handle_tx
    );

    task_status &= xTaskCreate(
        movella_task, "movella", 2560, NULL, movella_task_priority, &movella_task_handle
    );

    task_status &= xTaskCreate(log_task, "logger", 512, NULL, log_task_priority, &log_task_handle);

    task_status &= xTaskCreate(
        controller_task, "controller", 512, NULL, controller_task_priority, &controller_task_handle
    );

    task_status &= xTaskCreate(
        estimator_task, "estimator", 8192, NULL, estimator_task_priority, &estimator_task_handle
    );

    if (task_status != pdTRUE) {
        // Log critical task creation failure
        log_text(10, "SystemInit", "CRITICAL: Failed to create one or more FreeRTOS tasks.");
        return W_OVERFLOW;
    }
    log_text(10, "SystemInit", "All tasks created successfully.");
    return W_SUCCESS;
}
