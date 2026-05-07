#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/controller/controller_module.h"
#include "application/flight_phase/flight_phase_types.h"
#include "application/fsm/fsm_types.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "third_party/canlib/message/msg_actuator.h"

#define DATA_WAIT_MS 10
#define LOG_WAIT_MS 10
#define RECOVERY_PERIOD_MS 1000
#define IDLE_PERIOD_MS 1

// these two are redundent but different health check structs, should we just have one instead???
static controller_t controller_state = {0};
static controller_error_data_t controller_error_stats = {0};

/**
 * Initialize controller module
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
	// Initialize error tracking
	controller_error_stats.is_init = true;
	controller_error_stats.can_send_errors = 0;
	controller_error_stats.data_miss_counter = 0;
	controller_error_stats.timestamp_errors = 0;
	controller_error_stats.gain_interpolation_errors = 0;
	controller_error_stats.angle_calculation_errors = 0;
	controller_error_stats.log_errors = 0;
	// return w_status_t state
	log_text(LOG_WAIT_MS, "controller", "initialization successful");
	return W_SUCCESS;
}

// helper to run 1 loop of the controller task, including delaying where needed.
w_status_t controller_step(controller_ctx_t *ctx, const fsm_state_t curr_fsm_state,
						   const uint32_t act_allowed_timestamp_ms,
						   const uint32_t curr_timestamp_ms) {
	if (NULL == ctx) {
		log_text(LOG_WAIT_MS, "controller", "ERROR: Invalid context ptr.");
		return W_INVALID_PARAM;
	}

	log_data_container_t log_container = {0};
	w_status_t status = W_SUCCESS;

	// do the following steps which vary depending on curr flight phase:
	// 1. determine new canard angle cmd
	// 2. process new cmd (send to CAN, update output queue, log)
	// 3. do task delay
	switch (curr_fsm_state) {
			// recovery: actively cmd 0 deg at 1hz
			// status |= send_cmd(cmd_angle_zero);

		// // ref_signal ignored here so set to 0
		// log_container.controller.cmd_angle = (float)cmd_angle_zero;
		// log_container.controller.ref_signal = 0.0f;
		// if (W_SUCCESS != log_data(5, LOG_TYPE_CANARD_CMD, &log_container)) {
		//     log_text(LOG_WAIT_MS, "cntl recovery", "log cmd fail");
		//     status |= W_FAILURE;
		// }
		// // delay 1s per iteration. not as precise as taskdelayuntil but doesnt matter here.
		// // AVOID vtaskdelayuntil: it breaks cuz we dont use it in ACT_ALLOWED phase, so
		// // last_wake_time doesnt get updated consistently and causes this to break as it
		// // tries to catch up in time. TDOO: update freertos versions to where delayuntil has
		// // a return value...
		// vTaskDelay(pdMS_TO_TICKS(RECOVERY_PERIOD_MS));
		// // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS));
		// break;

		// actuation allowed: run controller module.
		// actuation continues through recovery phase too until the board shuts off
		case STATE_RECOVERY:
		case STATE_ACT_ALLOWED: {
			uint32_t act_allowed_ms = curr_timestamp_ms - act_allowed_timestamp_ms;

			float ref_signal = 0.0f; // track latest reference signal for logging only
			double new_cmd = 0.0;

			// make sure the state is fresh
			if (!(ctx->new_input_state.state_updated)) {
				// make sure command is stale
				ctx->cmd_output.cmd_updated = false;
				controller_state.data_miss_counter++;
				log_text(LOG_WAIT_MS, "controller", "data miss");
				return W_FAILURE;
			} else {
				ctx->new_input_state.state_updated = false;
			}

			// run controller module
			status |=
				controller_module(ctx->new_input_state, act_allowed_ms, &new_cmd, &ref_signal);

			// send cmd if we can
			if (W_SUCCESS == status) {
				// update the controller output
				// TODO: currently assuming the timer didn't fail, this should be reevaluated
				ctx->cmd_output.commanded_angle = new_cmd;
				ctx->cmd_output.timestamp = curr_timestamp_ms;
				ctx->cmd_output.cmd_updated = true;

				log_container.controller.cmd_angle = (float)new_cmd;
				log_container.controller.ref_signal = ref_signal;
				if (W_SUCCESS != log_data(5, LOG_TYPE_CANARD_CMD, &log_container)) {
					log_text(LOG_WAIT_MS, "cntl act", "log cmd fail");
					status |= W_FAILURE;
				}
			} else {
				ctx->cmd_output.cmd_updated = false;
				// if anything fails, send no cmd. MCB failsafes to 0 after some ms of silence
				log_text(LOG_WAIT_MS, "cntl act", "fail; send no cmd");
			}

			break;
		}

		default: // do nothing / wait for new state
			vTaskDelay(pdMS_TO_TICKS(IDLE_PERIOD_MS));
			break;
	}

	return status;
}

/**
 * Controller freertos task
 */
void controller_task(void *argument) {
	(void)argument;

	while (true) {
		// if (controller_step() != W_SUCCESS) {
		// 	log_text(LOG_WAIT_MS, "controller", "run loop fail");
		// }
	}
}

uint32_t controller_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log all error statistics
	log_text(0,
			 "controller",
			 "can_send=%lu, data_misses=%lu, timestamp=%lu, gain_interp=%lu, "
			 "angle_calc=%lu, log=%lu",
			 controller_error_stats.can_send_errors,
			 controller_error_stats.data_miss_counter,
			 controller_error_stats.timestamp_errors,
			 controller_error_stats.gain_interpolation_errors,
			 controller_error_stats.angle_calculation_errors,
			 controller_error_stats.log_errors);

	// Also log the internal controller state error counters for comparison
	log_text(0,
			 "controller",
			 "%s can_send_errors=%lu, data_miss_counter=%lu",
			 controller_error_stats.is_init ? "true" : "false",
			 controller_state.can_send_errors,
			 controller_state.data_miss_counter);

	return status_bitfield;
}

