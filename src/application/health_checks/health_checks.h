#ifndef HEALTH_CHECKS_H
#define HEALTH_CHECKS_H

#include "rocketlib/include/common.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initializes the health check module and watchdog registry
 *
 * Resets all watchdog task entries and initializes the watchdog counter.
 * Must be called once at system startup before other health check functions.
 *
 * @return W_SUCCESS if initialization successful
 */
w_status_t health_check_init(void);

/**
 * @brief Resets the watchdog timer for the calling task
 *
 * Must be called periodically by registered tasks to prevent watchdog timeout.
 * Calling frequency depends on the timeout value set during registration.
 * Should be called at least once within the timeout period.
 * This is typically done in the main loop of the dedicated task.
 *
 * @return W_SUCCESS if watchdog successfully kicked, W_FAILURE if task not registered
 */
w_status_t watchdog_kick(void);

/**
 * @brief Registers a task for watchdog monitoring
 *
 * Adds a task to the watchdog registry with specified timeout value.
 * Each task should call this once during initialization before using watchdog_kick.
 *
 * @param[in] task_handle Handle of the task to register (typically from xTaskGetCurrentTaskHandle)
 * @param[in] timeout_ticks Maximum ticks between watchdog kicks before timeout is triggered
 *
 * @return W_SUCCESS if registration successful, W_FAILURE on invalid params or registry full
 */
w_status_t watchdog_register_task(TaskHandle_t task_handle, uint32_t timeout_ticks);

/**
 * @brief Task function for health check background processing
 *
 * Runs continuously, performing system health checks at 1Hz intervals.
 * Monitors all registered tasks for watchdog timeouts and checks system current.
 * Should be created as a FreeRTOS task during system initialization.
 *
 * @param void* argument Pointer to task argument (unused)
 *
 * @return None (this is a task function that never returns)
 */
void health_check_task(void *argument);

#endif
