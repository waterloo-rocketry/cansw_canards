#ifndef HEALTH_CHECKS_H
#define HEALTH_CHECKS_H

#include "FreeRTOS.h"
#include "message_types.h"
#include "rocketlib/include/common.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Health check result from a module
 *
 * Standard structure returned by all module health check functions
 */
typedef struct {
	can_canards_health_severity_t severity;
	can_canards_module_id_t module_id;
	uint32_t error_bitfield;
} health_status_t;

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
 * @brief Handles a fatal system error by sending a CAN message.
 *
 * This function attempts to send a CAN message indicating the error and then
 * enters a safe, non-recoverable state (infinite loop with interrupts disabled).
 * It is designed to be called in critical failure scenarios where normal error
 * logging (e.g., to SD card) or task execution may not be possible.
 *
 * It uses the canlib library to send a DEBUG_RAW message with a coarse timestamp
 * and the first few characters of the error message.
 *
 * @param errorMsg A descriptive string for the error (only the first ~6 chars will be sent).
 */
void proc_handle_fatal_error(const char *errorMsg);

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
