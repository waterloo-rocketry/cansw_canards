#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/can_handler/can_telemetry_scaling.h"
#include "application/controller/controller.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "application/telemetry/telemetry.h"
#include "canlib.h"
#include "common/gnc/gnc_types.h"
#include "drivers/timer/timer.h"

#define DATA_WAIT_MS 10
#define LOG_WAIT_MS 10
static const float64_t MS_TO_SEC = 0.001;
static const float64_t TENTH_MS_TO_MS = 0.1;
static const uint32_t CTRL_LOG_DATA_TIMEOUT = 0;

// TODO: send roll target angle through can and body lift coeff (need updated canlib)
typedef struct {
	float64_t command;
	// canard lift coeff (coefficient_of_roll_control[0])
	// body lift coeff (coefficient_of_roll_control[1])
	float64_t coefficient_of_roll_control[2];
	// Roll Angle Target ref_roll[0]
	// Roll Rate Target ref_roll[1]
	float64_t ref_roll[2];

	// omitted from sending through can since we dont have a
	// sensor id for it...
} ctrl_value_handle_t;

static QueueHandle_t ctrl_value_queue;

// these two are redundent but different health check structs, should we just have one instead???
static controller_t controller_state = {0};
static controller_error_data_t controller_error_stats = {0};

static w_status_t ctrl_can_telemetry(void) {
	ctrl_value_handle_t ctrl_value_latest_raw;

	if (xQueuePeek(ctrl_value_queue, &ctrl_value_latest_raw, 0) == pdTRUE) {
		int16_t cmd;
		int16_t coef_canard_lift;

		if (W_SUCCESS !=
			can_encode_scaled_float(SCALE_CTRL_CMD, ctrl_value_latest_raw.command, &cmd)) {
			log_text(0, LOG_LVL_WARN, "controller", "Can encode failed for command.");
			return W_FAILURE;
		}

		if (W_SUCCESS !=
			can_encode_scaled_float(SCALE_CTRL_COEF_OF_ROLL_CTRL,
									ctrl_value_latest_raw.coefficient_of_roll_control[0],
									&coef_canard_lift)) {
			log_text(
				0, LOG_LVL_WARN, "controller", "Can encode failed for canard lift coefficient.");
			return W_FAILURE;
		}

		uint32_t timestamp;

		if (timer_get_ms(&timestamp) != W_SUCCESS) {
			log_text(0, LOG_LVL_WARN, "controller", "Failed to get timestamp for can msg tx");
			return W_FAILURE;
		}

		w_status_t status = W_SUCCESS;

		can_msg_t msg;
		build_analog_sensor_16bit_msg(PRIO_LOW,
									  (uint16_t)timestamp,
									  SENSOR_CANARD_CTRL_CMD_ANGLE,
									  (uint16_t)(cmd + TELEMETRY_INT16_OFFSET),
									  &msg);

		if (can_handler_transmit(&msg) != W_SUCCESS) {
			log_text(
				0, LOG_LVL_WARN, "controller", "Failed to transmit command value through can.");
			status |= W_FAILURE;
		}

		build_analog_sensor_16bit_msg(PRIO_LOW,
									  (uint16_t)timestamp,
									  SENSOR_CANARD_CTRL_COEFF_LIFT,
									  (uint16_t)(coef_canard_lift + TELEMETRY_INT16_OFFSET),
									  &msg);

		if (can_handler_transmit(&msg) != W_SUCCESS) {
			log_text(0,
					 LOG_LVL_WARN,
					 "controller",
					 "Failed to transmit canard lift coefficient value through can.");
			status |= W_FAILURE;
		}

		return status;
	} else {
		log_text(0,
				 LOG_LVL_WARN,
				 "controller",
				 "Failed to peek mailbox queue while sending current ctrl values through can.");

		return W_FAILURE;
	}
}

static w_status_t ctrl_sd_telemetry(void) {
	ctrl_value_handle_t ctrl_value_latest_raw;

	w_status_t status = W_SUCCESS;

	if (xQueuePeek(ctrl_value_queue, &ctrl_value_latest_raw, 0) != pdTRUE) {
		log_text(0,
				 LOG_LVL_WARN,
				 "controller",
				 "Failed to peek mailbox queue while sending current nav values through can.");

		return W_FAILURE;
	}

	// set up the log data
	log_data_container_t log_container = {0};

	// ctrl
	log_container.controller.command = (float32_t)ctrl_value_latest_raw.command;
	log_container.controller.canard_coeff =
		(float32_t)ctrl_value_latest_raw.coefficient_of_roll_control[0];
	log_container.controller.body_coeff =
		(float32_t)ctrl_value_latest_raw.coefficient_of_roll_control[1];
	log_container.controller.roll_angle_target = (float32_t)ctrl_value_latest_raw.ref_roll[0];
	log_container.controller.roll_rate_target = (float32_t)ctrl_value_latest_raw.ref_roll[1];

	if (log_data(CTRL_LOG_DATA_TIMEOUT, LOG_TYPE_CONTROLLER, &log_container) != W_SUCCESS) {
		log_text(0, LOG_LVL_WARN, "Controller", "Failed to log cntrl data.");
		return W_FAILURE;
	}
	return W_SUCCESS;
}

/**
 * Initialize controller module
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
	ctrl_value_queue = xQueueCreate(1, sizeof(ctrl_value_handle_t));
	configASSERT(ctrl_value_queue != NULL);

	static const telemetry_source_config_t telemetry_sources[] = {
		{"Ctrl CAN", ctrl_can_telemetry, STATE_PAD_NAV, 1000 / 10},
		{"Ctrl CAN", ctrl_can_telemetry, STATE_BOOST, 1000 / 10},
		{"Ctrl CAN", ctrl_can_telemetry, STATE_ACT_ALLOWED, 1000 / 10},

		{"Ctrl SD", ctrl_sd_telemetry, STATE_PAD_NAV, 1000 / 200},
		{"Ctrl SD", ctrl_sd_telemetry, STATE_BOOST, 1000 / 200},
		{"Ctrl SD", ctrl_sd_telemetry, STATE_ACT_ALLOWED, 1000 / 200},
		{"Ctrl SD", ctrl_sd_telemetry, STATE_RECOVERY, 1000 / 20},
		{"Ctrl SD", ctrl_sd_telemetry, STATE_SLEEPY, 1000 / 1},
	};

	static const size_t telemetry_source_count =
		sizeof(telemetry_sources) / sizeof(telemetry_source_config_t);
	w_status_t telemetry_register_status = W_SUCCESS;

	// Register callbacks and check
	for (size_t i = 0; i < telemetry_source_count; i++) {
		telemetry_register_status |= telemetry_register(&telemetry_sources[i]);
	}

	if (W_SUCCESS != telemetry_register_status) {
		log_text(1, LOG_LVL_WARN, "controller", "Failed to register telemetry sources.");
		return W_FAILURE;
	}

	// Initialize error tracking
	controller_error_stats = (controller_error_data_t){.is_init = true};
	// return w_status_t state
	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "controller", "initialization successful");
	return W_SUCCESS;
}

w_status_t controller_codegen_init(controller_ctx_t *p_ctx) {
	// init to default values
	p_ctx->gnc_controller_ctx.coeffs[0] = 2;
	p_ctx->gnc_controller_ctx.P[0] = 1e-9;
	p_ctx->gnc_controller_ctx.P[3] = 1e-5;

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
		((float64_t)(timestamp_tenth_ms - (p_ctx->last_run_tenth_ms))) * TENTH_MS_TO_MS * MS_TO_SEC;

	float64_t ref_signal = 0.0;

	bool is_success = false;

	controller_codegen_entry(p_ctx->p_gnc_stack_data,
							 flight_time_sec,
							 dt_controller_sec,
							 p_input->xR,
							 p_input->dynamic_pressure,
							 p_input->canard_angle_rad,
							 &(p_ctx->gnc_controller_ctx),
							 &(p_output->canard_command_angle_rad),
							 p_output->ref_roll,
							 &is_success);

	if (is_success) { // the controller ran
		// update new timestamp
		p_output->timestamp_tenth_ms = timestamp_tenth_ms;

		ctrl_value_handle_t ctrl_latest_values;

		ctrl_latest_values.coefficient_of_roll_control[0] =
			p_ctx->gnc_controller_ctx.coeffs[0]; // canard lift
		ctrl_latest_values.coefficient_of_roll_control[1] =
			p_ctx->gnc_controller_ctx.coeffs[1]; // body lift (not send through can)

		ctrl_latest_values.ref_roll[0] = p_output->ref_roll[0]; // target roll angle
		ctrl_latest_values.ref_roll[1] = p_output->ref_roll[1]; // target roll rate

		ctrl_latest_values.command = p_output->canard_command_angle_rad;

		xQueueOverwrite(ctrl_value_queue, &ctrl_latest_values);

	} else {
		log_text(0, LOG_LVL_WARN, "controller", "Controller failed");
	}
	p_ctx->last_run_tenth_ms = timestamp_tenth_ms;

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

	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_CONTROLLER,
							  .error_bitfield = 0};

	return status;
}
