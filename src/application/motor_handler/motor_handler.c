#include "application/motor_handler/motor_handler.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define MOTOR_TASK_PERIOD_MS 10 // 100 Hz command rate (poll slower when not actuating?)
#define LOG_WAIT_MS 10

static motor_handler_error_data_t motor_error_stats = {0};
static motor_feedback_t latest_feedback = {0};

w_status_t motor_handler_init(FDCAN_HandleTypeDef *hfdcan) {
	if (NULL == hfdcan) {
		return W_INVALID_PARAM;
	}

	// todo: init motor driver

	return W_SUCCESS;
}

void motor_handler_task(void *argument) {
	// todo: check flight phase, send commands, read feedback,
	// check for fatal faults, log errors, check for timeouts

	// Kick watchdog
	watchdog_kick();
}

w_status_t motor_handler_get_latest_feedback(motor_feedback_t *fb) {
	if (NULL == fb) {
		return W_INVALID_PARAM;
	}

	*fb = latest_feedback;
	return W_SUCCESS;
}

uint32_t motor_handler_get_status(void) {
	uint32_t status_bitfield = 0;

	return status_bitfield;
}
