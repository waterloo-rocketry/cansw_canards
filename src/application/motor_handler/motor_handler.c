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
	if (motor_driver_init(hfdcan) != W_SUCCESS) {
		log_text(LOG_WAIT_MS, "motor", "Motor driver init failure");
		return W_FAILURE;
	}

	motor_error_stats.is_init = true;
	log_text(LOG_WAIT_MS, "motor", "Motor control modules init success");
	return W_SUCCESS;
}

void motor_handler_task(void *argument) {
	(void)argument;

	TickType_t last_wake_time = xTaskGetTickCount();
	uint32_t last_feedback_ms = 0;
	timer_get_ms(&last_feedback_ms);

	while (true) {
		flight_phase_state_t current_phase = flight_phase_get_state();
		motor_feedback_t fb = {0};
		bool feedback_received = false;

		if (motor_get_latest_feedback(&fb) == W_SUCCESS) {
			feedback_received = true;
			latest_feedback = fb;
			timer_get_ms(&last_feedback_ms);

			if (feedback_received && fb.fault_code != MOTOR_FAULT_NONE) {
				motor_error_stats.fault_count++;
				motor_error_stats.last_fault = fb.fault_code;

				log_text(LOG_WAIT_MS, "motor", "Motor fault: %d", fb.fault_code);

				if (motor_is_fatal_fault(fb.fault_code)) {
					log_text(LOG_WAIT_MS, "motor", "FATAL fault: %d", fb.fault_code);
					motor_send_disable_cmd();
					proc_handle_fatal_error("motor");
				}
			}
		}

		// check for feedback timeouts
		uint32_t current_time_ms = 0;
		if (timer_get_ms(&current_time_ms) == W_SUCCESS) {
			if ((current_time_ms - last_feedback_ms) > MOTOR_FEEDBACK_TIMEOUT_MS) {
				motor_error_stats.feedback_timeouts++;
			}
		}

		// check flight phase and act accordingly
		switch (current_phase) {
			case STATE_ACT_ALLOWED:
			case STATE_RECOVERY: {
				controller_output_t command = {0};
				if (controller_get_latest_output(&command) == W_SUCCESS) {
					float angle_deg = (float)(command.commanded_angle * (180.0 / M_PI));
					if (motor_send_position_cmd(angle_deg) != W_SUCCESS) {}
				}

				vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
				break;
			}
			default:
				motor_send_disable_cmd();
				vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
				break;
		}

		// kick dog
		watchdog_kick();
	}

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
