#include "application/motor_handler/motor_handler.h"
#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define LOG_WAIT_MS 10

// Feedback timeout
#define MOTOR_FEEDBACK_TIMEOUT_MS 500

// Controller timeout
#define CONTROLLER_TIMEOUT_MS 100

static motor_handler_error_data_t motor_error_stats = {0};
static ak45_feedback_t latest_feedback = {0};

static QueueHandle_t angle_cmd_queue = NULL; // placeholder angle command queue

w_status_t motor_handler_init(FDCAN_HandleTypeDef *hfdcan) {
	if (NULL == hfdcan) {
		return W_INVALID_PARAM;
	}

	// todo: init ak45 driver, create angle command queue

	return W_SUCCESS;
}

w_status_t motor_handler_set_angle_cmd(float angle_deg) {
	if (NULL == angle_cmd_queue) {
		return W_INVALID_PARAM;
	}

	// todo: update timestamp so the task knows when the command arrived,
	// overwrite the current command in the angle cmd queue

	return W_SUCCESS;
}

w_status_t motor_handler_get_latest_feedback(ak45_feedback_t *fb) {
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
