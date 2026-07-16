#ifndef HEALTH_CHECKS_H
#define HEALTH_CHECKS_H

#include "FreeRTOS.h"
#include "rocketlib/include/common.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Health severity levels
 */
typedef enum {
	HEALTH_OK = 0, /**< No issues */
	HEALTH_ERROR, /**< Something is wrong, but can still fly safely */
	HEALTH_FATAL /**< Unrecoverable failure, unsafe flight */
} health_severity_t;

typedef enum {
	MODULE_ADC = 0,
	MODULE_ADXL380 = 1,
	MODULE_ADXRS649 = 2,
	MODULE_AK45 = 3,
	MODULE_CAN_HANDLER = 4,
	MODULE_CONTROLLER = 5,
	MODULE_FLIGHT_PHASE = 6,
	MODULE_FSM = 7,
	MODULE_GPIO = 8,
	MODULE_I2C = 9,
	MODULE_IIS2MDC = 10,
	MODULE_LOGGER = 11,
	MODULE_LSM6DSV32X = 12,
	MODULE_MOVELLA = 13,
	MODULE_MS5611 = 14,
	MODULE_NAVIGATOR = 15,
	MODULE_POWER_HANDLER = 16,
	MODULE_SD_CARD = 17,
	MODULE_SENSOR_HANDLER = 18,
	MODULE_TELEMETRY = 19,
	MODULE_TIMER = 20,
	MODULE_UART = 21,
	MODULE_COUNT = 22, // number of modules
	MODULE_MAX = 31
} module_id_t;

// Any module can add more error codes
typedef enum {
	ERR_NONE = 0,
	ERR_AD_ACCEL = 1,
	ERR_AD_GYRO = 2,
	ERR_AK45_FAILED_CALIBRATION = 3,
	ERR_AK45_NOT_CALIBRATED = 4,
	ERR_BAT1_FAULT = 5,
	ERR_BAT2_FAULT = 6,
	ERR_BOARD_BARO = 7,
	ERR_BOARD_IMU = 8,
	ERR_BOARD_MAG = 9,
	ERR_CRC_FAILED = 10,
	ERR_CRITICAL = 11,
	ERR_DROPPED_RX = 12,
	ERR_DROPPED_TX = 13,
	ERR_ERROR_STATE = 14,
	ERR_EVENT_SEND_FAILED = 15,
	ERR_FUNC_REGISTER_FAILED = 16,
	ERR_GPIO_FAIL = 17,
	ERR_HANDLE_INVALID = 18,
	ERR_INVALID_EVENT = 19,
	ERR_INVALID_PARAM = 20,
	ERR_LOW_POWER_MODE_WITH_EXT_5V_ON = 21,
	ERR_MOTOR = 22,
	ERR_MTI = 23,
	ERR_NOT_INIT = 24,
	ERR_NOT_RUN = 25,
	ERR_NULL_CTX = 26,
	ERR_OVERFLOW = 27,
	ERR_PERIPHERAL_COMM_FAIL = 28,
	ERR_TIMEOUT = 29,
	ERR_TX_FAILURE = 30,
	ERR_TX_OVERDUE = 31,
	ERR_UNKNOWN_STATE = 32,
	ERR_MAX = 31
} module_error_code_t;

/**
 * @brief Health check result from a module
 *
 * Standard structure returned by all module health check functions
 */
typedef struct {
	health_severity_t severity;
	module_id_t module_id;
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
