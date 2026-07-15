#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "GNC_codegen.h"
#include "application/controller/controller.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "common/gnc/gnc_types.h"

#define DATA_WAIT_MS 10
#define LOG_WAIT_MS 10
static const float64_t MS_TO_SEC = 0.001;
static const float64_t TENTH_MS_TO_MS = 0.1;

static controller_error_data_t controller_error_stats = {0};

/**
 * Initialize controller module
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
	// Initialize error tracking
	controller_error_stats = (controller_error_data_t){.is_init = true};
	// return w_status_t state
	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "controller", "initialization successful");
	return W_SUCCESS;
}

// helper to run 1 iteration of the controller algo, including delaying where needed.
w_status_t controller_step(const controller_input_t *p_input, const uint32_t timestamp_tenth_ms,
						   controller_ctx_t *p_ctx, controller_output_t *p_output) {
	if ((NULL == p_input) || (NULL == p_ctx) || (NULL == p_output)) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "controller", "Invalid context ptr.");
		controller_error_stats.null_ctx_count++;
		controller_error_stats.ctx_is_null = true;
		return W_INVALID_PARAM;
	}

	// TODO: check with Tristan

	float64_t flight_time_sec = ((float64_t)((uint32_t)(timestamp_tenth_ms * TENTH_MS_TO_MS) -
											 (p_input->launch_timestamp_ms))) *
								MS_TO_SEC;
	float64_t dt_controller_sec =
		((float64_t)(timestamp_tenth_ms - (p_ctx->last_run_tenth_ms))) * MS_TO_SEC;

	float64_t ref_signal = 0.0;

	bool is_run = false;

	controller_codegen_entry(p_ctx->p_gnc_stack_data,
							 flight_time_sec,
							 dt_controller_sec,
							 p_input->xR,
							 p_input->dynamic_pressure,
							 p_input->canard_angle_rad,
							 &(p_ctx->gnc_controller_ctx),
							 &(p_output->canard_command_angle_rad),
							 p_output->ref_roll,
							 &is_run);

	if (is_run) { // the controller ran
		// update new timestamp
		p_output->timestamp_tenth_ms = timestamp_tenth_ms;
		p_ctx->last_run_tenth_ms = timestamp_tenth_ms;
	} else {
		log_text(0, LOG_LVL_WARN, "controller", "Controller was not run");
		controller_error_stats.controller_not_run_count++;
		controller_error_stats.controller_not_run = true;
	}

	return W_SUCCESS;
}

health_status_t controller_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_CONTROLLER, .error_bitfield = 0};

	if (controller_error_stats.is_init == false) {
		status.severity = HEALTH_FATAL;
		status.error_bitfield |= 1 << MODULE_ERR_NOT_INIT;
	}

	if (controller_error_stats.ctx_is_null) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_NULL_CTX;
	}

	if (controller_error_stats.controller_not_run) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_NOT_RUN;
	}

	// reset flags
	controller_error_stats.ctx_is_null = false;
	controller_error_stats.controller_not_run = false;

	// Log all error statistics
	log_text(10,
			 LOG_LVL_INFO,
			 "controller",
			 "init=%d, null_ctx=%d, not_run=%d",
			 controller_error_stats.is_init,
			 controller_error_stats.null_ctx_count,
			 controller_error_stats.controller_not_run_count);
	return status;
}

