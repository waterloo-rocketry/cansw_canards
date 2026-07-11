#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "common/gnc/gnc_types.h"

#define DATA_WAIT_MS 10
#define LOG_WAIT_MS 10
static const float64_t MS_TO_SEC = 0.001;
static const float64_t TENTH_MS_TO_MS = 0.1;

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
	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "controller", "initialization successful");
	return W_SUCCESS;
}

// helper to run 1 iteration of the controller algo, including delaying where needed.
w_status_t controller_step(const controller_input_t *p_input, const uint32_t timestamp_tenth_ms,
						   controller_ctx_t *p_ctx, controller_output_t *p_output) {
	if ((NULL == p_input) || (NULL == p_ctx) || (NULL == p_output)) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "controller", "Invalid context ptr.");
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

		// scale and broadcast the commanded canard angle over CAN. can_encode_scaled_float
		// writes the native scaled int16 into cmd_scaled; the builder repacks it big-endian.
		int16_t cmd_scaled = 0;
		w_status_t enc = can_encode_scaled_float(
			SCALE_CTRL_CMD, (float32_t)p_output->canard_command_angle_rad, &cmd_scaled);
		can_msg_t msg = {0};
		build_analog_sensor_16bit_msg(PRIO_LOW,
									  (uint16_t)(timestamp_tenth_ms * TENTH_MS_TO_MS),
									  SENSOR_CANARD_CTRL_CMD_ANGLE,
									  (uint16_t)cmd_scaled,
									  &msg);
		if ((W_SUCCESS != enc) || (W_SUCCESS != can_handler_transmit(&msg))) {
			controller_error_stats.can_send_errors++;
		}
		// TODO(telemetry): SCALE_CTRL_COEF_OF_ROLL_CTRL (coeff of lift / CL) is not broadcast
		// -- CL is an estimator state-vector output, not exposed in controller_output_t.
		// SCALE_CTRL_ROLL_TARGET has no canlib canard telemetry ID (see can_telemetry_scaling.h).
	} else {
		log_text(0, LOG_LVL_WARN, "controller", "Controller was not run");
	}

	return W_SUCCESS;
}

health_status_t controller_get_status(void) {
	// Log all error statistics
	log_text(0,
			 LOG_LVL_INFO,
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
			 LOG_LVL_INFO,
			 "controller",
			 "%s can_send_errors=%lu, data_miss_counter=%lu",
			 controller_error_stats.is_init ? "true" : "false",
			 controller_state.can_send_errors,
			 controller_state.data_miss_counter);

	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_CONTROLLER, .error_bitfield = 0};

	return status;
}

