#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "application/controller/controller.h"
#include "application/controller/controller_module.h"
#include "application/fsm/fsm.h"
#include "application/logger/log.h"

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
	controller_error_stats = (controller_error_data_t){.is_init = true};
	// return w_status_t state
	log_text(LOG_WAIT_MS, "controller", "initialization successful");
	return W_SUCCESS;
}

// helper to run 1 iteration of the controller algo, including delaying where needed.
w_status_t controller_step(controller_ctx_t *ctx, const controller_input_t *input,
						   controller_output_t *output, const uint32_t act_allowed_timestamp_ms,
						   const uint32_t curr_timestamp_ms) {
	if (NULL == ctx) {
		log_text(LOG_WAIT_MS, "controller", "ERROR: Invalid context ptr.");
		return W_INVALID_PARAM;
	}

	log_data_container_t log_container = {0};
	w_status_t status = W_SUCCESS;

	uint32_t act_allowed_ms = curr_timestamp_ms - act_allowed_timestamp_ms;

	float ref_signal = 0.0f; // track latest reference signal for logging only

	// TODO: call codegen to run controller module
	// status |=
	//     controller_module(ctx, act_allowed_ms, &output, &ref_signal);
	(void)input;

	// TODO: reorder structre to multiple return points
	// send motor cmd if we can
	if (W_SUCCESS == status) {
		// TODO: motor

		log_container.controller.cmd_angle = (float)output->commanded_angle;
		log_container.controller.ref_signal = ref_signal;
		if (W_SUCCESS != log_data(5, LOG_TYPE_CANARD_CMD, &log_container)) {
			log_text(LOG_WAIT_MS, "cntl act", "log cmd fail");
			status |= W_FAILURE;
		}
	} else {
		// if anything fails, send no cmd. MCB failsafes to 0 after some ms of silence
		log_text(LOG_WAIT_MS, "cntl act", "fail; send no cmd");
	}

	return status;
}

health_status_t controller_get_status(void) {
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

	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_CONTROLLER, .error_bitfield = 0};

	return status;
}

