#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// Motor CAN driver ID configurable on the servo, default 1
#define MOTOR_DRIVER_ID 0x45

// Servo fault codes
typedef enum {
	MOTOR_FAULT_NONE = 0,
	MOTOR_FAULT_OVERVOLTAGE = 1,
	MOTOR_FAULT_UNDERVOLTAGE = 2,
	MOTOR_FAULT_DRV_FAULT = 3,
	MOTOR_FAULT_ABS_OVERCURRENT = 4,
	MOTOR_FAULT_OVERTEMP_FET = 5,
	MOTOR_FAULT_OVERTEMP_MOTOR = 6
} motor_fault_code_t;

/**
 * @brief Decoded feedback from the servo
 */
typedef struct {
	float position_deg;
	float speed_erpm;
	float current_a;
	int8_t temperature_c;
	motor_fault_code_t fault_code;
	uint32_t timestamp_ms; // Time of last received feedback
} motor_feedback_t;

/**
 * @brief Initialize the servo driver
 *
 * Sets up internal queues and configures the FDCAN RX filter for the servo's extended ID.
 *
 * @param[in] hfdcan Pointer to FDCAN handle
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_driver_init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Send a position command to the servo
 *
 * @param[in] angle_deg  Target position in degrees
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_send_position_cmd(float angle_deg);

/**
 * @brief Send a disable command to the servo
 *
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_send_disable_cmd(void);

/**
 * @brief Check if a fault code is fatal
 *
 * @param[in] fault  Fault code
 * @return true if fatal fault
 */
bool motor_is_fatal_fault(motor_fault_code_t fault);

/**
 * @brief Get the latest motor feedback
 *
 * @param[out] fb Pointer to store data
 * @return W_SUCCESS if feedback is available, W_FAILURE otherwise
 */
w_status_t motor_get_latest_feedback(motor_feedback_t *fb);

/**
 * @brief Get FDCAN transmit error count
 *
 * @return Number of transmission failures
 */
uint32_t motor_get_tx_errors(void);

#endif // MOTOR_DRIVER_H
