#include "FreeRTOS.h"
#include "math.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "canlib.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "application/navigator/navigator.h"
#include "common/gnc/gnc_types.h"
#include "drivers/timer/timer.h"

// ---------- private variables ----------
// IDEAL task period, for calculating CAN send rate limiter
static const uint32_t ESTIMATOR_TASK_PERIOD_MS = 5;
static const float64_t TENTH_MS_TO_SEC = 0.0001;
// Rate limit CAN tx: only send data at 10Hz, every 100ms
// TODO: if kept change to static const
#define ESTIMATOR_CAN_TX_PERIOD_MS 100
#define ESTIMATOR_CAN_TX_RATE (ESTIMATOR_CAN_TX_PERIOD_MS / ESTIMATOR_TASK_PERIOD_MS)
// wait for imu data for >5ms to avoid false failure if imu takes like 5.1ms
#define DATA_WAIT_MS 10

// Error tracking
static navigator_error_data_t estimator_error_stats = {0};

// ---------- public functions ----------

w_status_t navigator_init(void) {
	// Initialize error tracking
	estimator_error_stats = (navigator_error_data_t){.is_init = true};

	return W_SUCCESS;
}

w_status_t navigator_step(const navigator_input_t *p_input, const uint32_t timestamp_tenth_ms,
						  navigator_ctx_t *p_ctx, navigator_output_t *p_output) {
	if ((NULL == p_input) || (NULL == p_ctx) || (NULL == p_output)) {
		log_text(0, LOG_LVL_WARN, "navigator", "Invalid context ptr.");
		return W_INVALID_PARAM;
	}
	// calculate remainder navigator data
	float64_t dt_sec = (timestamp_tenth_ms - (p_ctx->last_run_tenth_ms)) * TENTH_MS_TO_SEC;
	bool in_flight_phase =
		(STATE_PAD_FILTER != p_input->fsm_state); // since earliest entry point is pad filter

	// construct sensor inputs
	gnc_navigator_sensor_input_t codegen_sensor_input = {0};

	// status repersents if this sensor should be used in this cycle of nav
	// lsm6
	memcpy(codegen_sensor_input.board_accel.meas,
		   p_input->sensor_data->board_meas.board_imu.accel.array,
		   sizeof(codegen_sensor_input.board_accel.meas));
	codegen_sensor_input.board_accel.status = p_input->sensor_data->board_meas.board_imu.is_new;

	memcpy(codegen_sensor_input.board_gyro.meas,
		   p_input->sensor_data->board_meas.board_imu.gyro.array,
		   sizeof(codegen_sensor_input.board_gyro.meas));
	codegen_sensor_input.board_gyro.status = p_input->sensor_data->board_meas.board_imu.is_new;

	// Baro
	codegen_sensor_input.board_baro.meas = p_input->sensor_data->board_meas.board_baro.meas;
	codegen_sensor_input.board_baro.status = p_input->sensor_data->board_meas.board_baro.is_new;

	// Mag
	memcpy(codegen_sensor_input.board_mag.meas,
		   p_input->sensor_data->board_meas.board_mag.meas.array,
		   sizeof(codegen_sensor_input.board_mag.meas));
	codegen_sensor_input.board_mag.status = p_input->sensor_data->board_meas.board_mag.is_new;

	// MTI
	memcpy(codegen_sensor_input.mti_accel.meas,
		   p_input->sensor_data->mti_meas.mti_accel.meas.array,
		   sizeof(codegen_sensor_input.mti_accel.meas));
	codegen_sensor_input.mti_accel.status = p_input->sensor_data->mti_meas.mti_accel.is_new;

	memcpy(codegen_sensor_input.mti_gyro.meas,
		   p_input->sensor_data->mti_meas.mti_gyro.meas.array,
		   sizeof(codegen_sensor_input.mti_gyro.meas));
	codegen_sensor_input.mti_gyro.status = p_input->sensor_data->mti_meas.mti_gyro.is_new;

	memcpy(codegen_sensor_input.mti_mag.meas,
		   p_input->sensor_data->mti_meas.mti_mag.meas.array,
		   sizeof(codegen_sensor_input.mti_mag.meas));
	codegen_sensor_input.mti_mag.status = p_input->sensor_data->mti_meas.mti_mag.is_new;

	codegen_sensor_input.mti_baro.meas = p_input->sensor_data->mti_meas.mti_baro.meas;
	codegen_sensor_input.mti_baro.status = p_input->sensor_data->mti_meas.mti_baro.is_new;

	// AD Accel
	memcpy(codegen_sensor_input.ad_accel.meas,
		   p_input->sensor_data->ad_meas.ad_accel.meas.array,
		   sizeof(codegen_sensor_input.ad_accel.meas));
	codegen_sensor_input.ad_accel.status = p_input->sensor_data->ad_meas.ad_accel.is_new;

	codegen_sensor_input.ad_gyro.meas[0] = p_input->sensor_data->ad_meas.ad_gyro.meas; // x axis
	codegen_sensor_input.ad_gyro.meas[1] = 0.0;
	codegen_sensor_input.ad_gyro.meas[2] = 0.0;
	codegen_sensor_input.ad_gyro.status = p_input->sensor_data->ad_meas.ad_gyro.is_new;

	bool is_run = false;

	navigation_codegen_entry(p_ctx->p_gnc_stack_data,
							 dt_sec,
							 in_flight_phase,
							 p_ctx->gnc_navigator_ctx.x,
							 p_ctx->gnc_navigator_ctx.P,
							 &(p_ctx->gnc_navigator_ctx.bias),
							 &(p_ctx->gnc_navigator_ctx.sensor_filter),
							 &codegen_sensor_input,
							 &(p_output->cov_norm),
							 p_output->roll_state,
							 &(p_output->dynamic_pressure),
							 &is_run);

	if (is_run) { // if nav ran
		p_ctx->last_run_tenth_ms = timestamp_tenth_ms;
	} else {
		log_text(0, LOG_LVL_WARN, "Navigator", "Nav failed to run");
	}

	return W_SUCCESS;
}

w_status_t pad_filter_init(navigator_ctx_t *p_ctx, all_sensors_data_t *p_sensor_data) {
	if ((NULL == p_ctx) || (NULL == p_sensor_data)) {
		log_text(0, LOG_LVL_WARN, "navigator", "Invalid context ptr.");
		return W_INVALID_PARAM;
	}

	// check if sensor is alive
	if (p_sensor_data->board_meas.board_imu.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.board_accel,
			   p_sensor_data->board_meas.board_imu.accel.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.board_accel));
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.board_gyro,
			   p_sensor_data->board_meas.board_imu.gyro.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.board_gyro));
	}
	if (p_sensor_data->board_meas.board_mag.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.board_mag,
			   p_sensor_data->board_meas.board_mag.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.board_mag));
	}
	if (p_sensor_data->board_meas.board_baro.is_new) {
		p_ctx->gnc_navigator_ctx.sensor_filter.board_baro =
			p_sensor_data->board_meas.board_baro.meas;
	}

	if (p_sensor_data->ad_meas.ad_accel.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.ad_accel,
			   p_sensor_data->ad_meas.ad_accel.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.ad_accel));
	}
	if (p_sensor_data->ad_meas.ad_gyro.is_new) {
		p_ctx->gnc_navigator_ctx.sensor_filter.ad_gyro[0] = p_sensor_data->ad_meas.ad_gyro.meas;
		p_ctx->gnc_navigator_ctx.sensor_filter.ad_gyro[1] = 0;
		p_ctx->gnc_navigator_ctx.sensor_filter.ad_gyro[2] = 0;
	}

	if (p_sensor_data->mti_meas.mti_accel.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.mti_accel,
			   p_sensor_data->mti_meas.mti_accel.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.mti_accel));
	}
	if (p_sensor_data->mti_meas.mti_gyro.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.mti_gyro,
			   p_sensor_data->mti_meas.mti_gyro.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.mti_gyro));
	}
	if (p_sensor_data->mti_meas.mti_mag.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.mti_mag,
			   p_sensor_data->mti_meas.mti_mag.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.mti_mag));
	}
	if (p_sensor_data->mti_meas.mti_baro.is_new) {
		p_ctx->gnc_navigator_ctx.sensor_filter.mti_baro = p_sensor_data->mti_meas.mti_baro.meas;
	}
	return W_SUCCESS;
}

// TODO: to be revived
// w_status_t navigator_log_state_to_can(const x_state_t *current_state) {
// 	can_msg_t msg;
// 	uint32_t current_time_ms;
// 	w_status_t status = W_SUCCESS;

// 	if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
// 		current_time_ms = 0; // Default to 0 if timer fails
// 	}
// 	uint16_t timestamp_16bit = (uint16_t)current_time_ms;

// 	// TODO: Redo how messages are built and sent
// 	// Iterate through all defined state IDs
// 	for (can_state_est_id_t state_id = 0; state_id < STATE_ID_ENUM_MAX; ++state_id) {
// 		// The x_state_t union maps directly to the enum order if accessed as an array
// 		// Convert the doubles in x_state_t to floats for CAN message
// 		float state_value = (float)current_state->array[state_id];

// 		if (!build_state_est_data_msg(PRIO_LOW, timestamp_16bit, state_id, &state_value, &msg)) {
// 			log_text(0, "Estimator", "Failed to build CAN message for state ID %d", state_id);
// 			estimator_error_stats.can_log_fails++;
// 			status = W_FAILURE; // Mark as failure but continue trying other states
// 			continue;
// 		}

// 		if (W_SUCCESS != can_handler_transmit(&msg)) {
// 			log_text(0, "Estimator", "Failed to transmit CAN message for state ID %d", state_id);
// 			estimator_error_stats.can_log_fails++;
// 			status = W_FAILURE; // Mark as failure but continue trying other states
// 		}
// 	}

// 	return status;
// }

health_status_t navigator_get_status(void) {
	// Log all error statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "Navigator",
			 "imu_timeouts=%lu, encoder_miss=%lu, controller_miss=%lu,",
			 estimator_error_stats.imu_data_timeouts,
			 estimator_error_stats.encoder_data_fails,
			 estimator_error_stats.controller_data_fails);
	log_text(0,
			 LOG_LVL_INFO,
			 "Navigator",
			 "pad_filter_fails=%lu, can_log_fails=%lu, invalid_phase=%lu",
			 estimator_error_stats.pad_filter_fails,
			 estimator_error_stats.can_log_fails,
			 estimator_error_stats.invalid_phase_errors);

	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_NAVIGATOR,
							  .error_bitfield = 0};

	return status;
}
