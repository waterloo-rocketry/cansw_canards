#ifndef AK45_DRIVER_H
#define AK45_DRIVER_H

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

#include "application/health_checks/health_checks.h"

// Servo fault codes
typedef enum {
	AK45_FAULT_NONE = 0,
	AK45_FAULT_OVERVOLTAGE = 1,
	AK45_FAULT_UNDERVOLTAGE = 2,
	AK45_FAULT_DRV_FAULT = 3,
	AK45_FAULT_ABS_OVERCURRENT = 4,
	AK45_FAULT_OVERTEMP_FET = 5,
	AK45_FAULT_OVERTEMP_MOTOR = 6
} ak45_fault_code_t;

/**
 * @brief Decoded feedback from the servo
 */
typedef struct {
	float32_t position_deg;
	float32_t speed_erpm; // RPM counted electrically
	float32_t current_a;
	int8_t temperature_c;
	ak45_fault_code_t fault_code;
	uint32_t timestamp_ms; // Time of last received feedback
} ak45_feedback_t;

/**
 * Status variables describing the health of the AK45 driver module
 */
typedef struct {
	bool is_init;
	bool failed_calibration; // TODO: implement calibration step and add this check
	uint32_t rx_errors; // FDCAN RX-callback failures
	uint32_t tx_errors; // FDCAN transmit FIFO add failures
	uint32_t invalid_args; // nullptr/ out of range arguments to functions
	uint32_t out_of_memory; // failure to allocate memory for queue creation
	uint32_t not_initialized; // calls to get_feedback when driver is not initialized
	uint32_t feedback_queue_empty; // calls for latest feedback when queue empty
	uint32_t reinit_attempts; // count of init attempts after first successful init
	uint32_t fdcan_stop_fails; // count of failures to stop FDCAN bus
	uint32_t init_fdcan_timeout; // count of timeouts to receive response from the motor during init
	uint32_t init_fdcan_notification_fails; // Count of failures to activate FDCAN notification
											// during init
	uint32_t init_fdcan_start_fails; // count of failures to start FDCAN bus
	uint32_t init_fdcan_filter_cfg_fails; // count of failures to configure FDCAN filter during init
} ak45_health_t;

/**
 * @brief Initialize the servo driver
 *
 * Sets up internal queues and configures the FDCAN RX filter for the servo's extended ID.
 *
 * @param[in] hfdcan Pointer to FDCAN handle
 * @param[in] can_init_timeout_ms timeout in ms for waiting for AK45's CAN to initialize
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t ak45_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms);

/**
 * @brief Send a position command to the servo
 *
 * @param[in] angle_deg  Target position in degrees
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t ak45_send_position_cmd(float32_t angle_deg);

/**
 * @brief Send a disable command to the servo
 *
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t ak45_send_disable_cmd(void);

/**
 * @brief Get the latest motor feedback
 *
 * @param[out] fb Pointer to store data
 * @return W_SUCCESS if feedback is available, W_FAILURE otherwise
 */
w_status_t ak45_get_latest_feedback(ak45_feedback_t *fb);

/**
 * @brief Get FDCAN transmit error count
 *
 * @return Number of transmission failures
 */
uint32_t ak45_get_tx_errors(void);

/**
 * @brief Get and report the AK45 motor driver status for the health check system
 *
 * Follows the module_get_status naming convention used by health_checks.
 *
 * @return CAN board status bitfield
 */
health_status_t ak45_get_status(void);

#endif // AK45_DRIVER_H
