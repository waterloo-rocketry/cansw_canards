#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "GNC_codegen.h"
#include "application/controller/controller.h"
#include "application/fsm/fsm.h"
#include "application/logger/log.h"

#define DATA_WAIT_MS 10
#define LOG_WAIT_MS 10
static const float64_t MS_TO_SEC = 0.001;

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
w_status_t controller_step(controller_ctx_t *ctx, GNC_codegenStackData *p_codegen_stack_data,
						   const controller_input_t *input, controller_output_t *output) {
	if (NULL == ctx) {
		log_text(LOG_WAIT_MS, "controller", "ERROR: Invalid context ptr.");
		return W_INVALID_PARAM;
	}

	// TODO: check with Tristan

	uint32_t flight_time_ms = (input->curr_timestamp_ms) - (input->launch_timestamp_ms);
	float64_t dt_controller_sec =
		((float64_t)((input->curr_timestamp_ms) - (ctx->last_run_ms))) * MS_TO_SEC;

	float64_t ref_signal = 0.0;

	controller_codegen_ctx_t output_ctx = {0};

	controller_codegen_entry(p_codegen_stack_data,
							 flight_time_ms,
							 dt_controller_sec,
							 input->xR,
							 input->pdyn,
							 input->motor_angle_rad,
							 &(ctx->codegen_ctx),
							 &(output->motor_command_angle_rad),
							 &(output->ref_roll_angle_rad),
							 &output_ctx);

	// copy over the new ctx
	memcpy(&(ctx->codegen_ctx), &output_ctx, sizeof(controller_codegen_ctx_t));

	// update new timestamp
	output->timestamp_ms = input->curr_timestamp_ms;
	ctx->last_run_ms = input->curr_timestamp_ms;

	return W_SUCCESS;
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

