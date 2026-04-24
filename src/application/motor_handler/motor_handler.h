#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include "FreeRTOS.h"
#include "drivers/ak45-10/ak45-10.h"
#include "stm32h7xx_hal.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// Feedback timeout
#define MOTOR_FEEDBACK_TIMEOUT_MS 500

// Controller timeout
#define CONTROLLER_TIMEOUT_MS = 100

/**
 * @brief Error tracking for motor handler module
 */
typedef struct {
	bool is_init;
	uint32_t feedback_timeouts;
	uint32_t controller_timeouts;
	uint32_t fault_count;
	ak45_fault_code_t last_fault; // Most recent fault code
} motor_handler_error_data_t;

/**
 * @brief Initialize motor handler module
 *
 * @param[in] hfdcan Pointer to FDCAN handle
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_handler_init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Motor handler task
 *
 * Checks flight phase (whether actuation is allowed)
 * Gets the commanded angle from the controller module
 * Sends position commands to the servo
 * Reads feedback and checks for fatal faults
 */
void motor_handler_task(void *argument);

/**
 * @brief Set the commanded motor angle, called by the controller module
 *
 * Pushes a new angle command into the motor handler.
 * The motor_handler_task will transmit this command to the servo
 * on its next iteration.
 *
 * @param[in] angle_deg  Desired angle in degrees
 * @return W_SUCCESS on success, W_INVALID_PARAM if motor handler is not init
 */
w_status_t motor_handler_set_angle_cmd(float angle_deg);

/**
 * @brief Get the latest motor feedback
 *
 * @param[out] fb Pointer to store data
 * @return W_SUCCESS if feedback is available, W_FAILURE otherwise
 */
w_status_t motor_handler_get_latest_feedback(ak45_feedback_t *fb);

/**
 * @brief Report motor handler health status
 *
 * Logs error statistics and latest feedback data.
 *
 * @return error bitfield
 */
uint32_t motor_handler_get_status(void);

#endif // MOTOR_HANDLER_H