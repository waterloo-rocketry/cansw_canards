#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include "FreeRTOS.h"
#include "drivers/motor_driver/motor_driver.h"
#include "stm32h7xx_hal.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// Feedback timeout
#define MOTOR_FEEDBACK_TIMEOUT_MS 500

/**
 * @brief Error tracking for motor handler module
 */
typedef struct {
	bool is_init;
	uint32_t feedback_timeouts;
	uint32_t fault_count;
	motor_fault_code_t last_fault; // Most recent fault code
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
 * @brief Get the latest motor feedback
 *
 * @param[out] fb Pointer to store data
 * @return W_SUCCESS if feedback is available, W_FAILURE otherwise
 */
w_status_t motor_handler_get_latest_feedback(motor_feedback_t *fb);

/**
 * @brief Report motor handler health status
 *
 * Logs error statistics and latest feedback data.
 *
 * @return error bitfield
 */
uint32_t motor_handler_get_status(void);

#endif // MOTOR_HANDLER_H