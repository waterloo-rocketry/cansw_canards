#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "application/health_checks/health_checks.h"
#include "rocketlib.h"

// motor 1 3 is an opposite pair
// motor 2 4 is an opposite pair
enum motor_id {
	motor_one,
	motor_two,
	motor_three,
	motor_four
};

typedef struct {
	motor_id motor_one;
	motor_id motor_two;
	motor_id motor_three;
	motor_id motor_four;
} motors;

/**
 * @brief Initializes the motor controller.
 * @return W_OK if successful, W_ERROR otherwise.
 */
w_status_t motor_init(void);

/**
 * @brief Sets the speed of the specified motor.
 * @param motor The ID of the motor to set.
 * @param timestamp The timestamp for the motor speed command.
 * @return W_OK if successful, W_ERROR otherwise.
 */
w_status_t get_motor_angle(motor_id motor, uint32_t *timestamp, float *angle);

/**
 * @brief The main task for controlling the motors.
 * @return W_OK if successful, W_ERROR otherwise.
 */
w_status_t motor_task(void *argument);

/**
 * @brief Gets the health status of the specified motor.
 * @param motor_health_status The health status to get.
 * @return The health status if successful, HEALTH_ERROR otherwise.
 */
health_status_t get_motor_health(uint32_t motor_health_status);

#endif // MOTOR_CONTROLLER_H
