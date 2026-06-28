#include "FreeRTOS.h"
#include "math.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "canlib.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/fsm/fsm.h"
#include "application/logger/log.h"
#include "application/navigator/navigator.h"
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

// TODO remove this once we get encorder reintegrated into the system
// latest encoder reading (millidegrees) from CAN
static QueueHandle_t encoder_data_queue_rad = NULL;

// ---------- public functions ----------

w_status_t navigator_init(void) {
	// create queues for imu data, encoder data, and controller cmd
	encoder_data_queue_rad = xQueueCreate(1, sizeof(float));

	if (NULL == encoder_data_queue_rad) {
		log_text(1, "adc", "initfailq");
		return W_FAILURE;
	}

	// Initialize error tracking
	estimator_error_stats = (navigator_error_data_t){.is_init = true};

	return W_SUCCESS;
}

w_status_t navigator_step(navigator_ctx_t *p_ctx, GNC_codegenStackData *p_codegen_stack_data,
						  const navigator_input_t *p_input, const all_sensors_data_t *p_sensor_data,
						  navigator_output_t *p_output) {
	// calculate remainder navigator data
	float64_t dt_sec =
		((p_input->curr_timestamp_tenth_ms) - (p_ctx->last_run_tenth_ms)) * TENTH_MS_TO_SEC;
	bool in_flight_phase =
		(STATE_PAD_FILTER != p_input->fsm_state); // since earliest entry point is pad filter

	// construct sensor inputs
	navigator_codegen_sensor_input_t codegen_sensor_input = {0};
	// lsm6
	memcpy(codegen_sensor_input.board_accel.meas,
		   p_sensor_data->board_meas.board_imu.accel.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.board_accel.status =
		p_sensor_data->board_meas.board_imu.is_new; // status true is on

	memcpy(codegen_sensor_input.board_gyro.meas,
		   p_sensor_data->board_meas.board_imu.gyro.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.board_gyro.status =
		p_sensor_data->board_meas.board_imu.is_new; // status true is on

	// Baro
	codegen_sensor_input.board_baro.meas = p_sensor_data->board_meas.board_baro.meas;
	codegen_sensor_input.board_baro.status =
		p_sensor_data->board_meas.board_baro.is_new; // status true is on

	// Mag
	memcpy(codegen_sensor_input.board_mag.meas,
		   p_sensor_data->board_meas.board_mag.meas.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.board_mag.status =
		p_sensor_data->board_meas.board_mag.is_new; // status true is on

	// MTI
	memcpy(codegen_sensor_input.mti_accel.meas,
		   p_sensor_data->mti_meas.mti_accel.meas.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.mti_accel.status =
		p_sensor_data->mti_meas.mti_accel.is_new; // status true is on

	memcpy(codegen_sensor_input.mti_gyro.meas,
		   p_sensor_data->mti_meas.mti_gyro.meas.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.mti_gyro.status =
		p_sensor_data->mti_meas.mti_gyro.is_new; // status true is on

	memcpy(codegen_sensor_input.mti_mag.meas,
		   p_sensor_data->mti_meas.mti_mag.meas.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.mti_mag.status =
		p_sensor_data->mti_meas.mti_mag.is_new; // status true is on

	codegen_sensor_input.mti_baro.meas = p_sensor_data->mti_meas.mti_baro.meas;
	codegen_sensor_input.mti_baro.status =
		p_sensor_data->mti_meas.mti_baro.is_new; // status true is on

	// AD Accel
	memcpy(codegen_sensor_input.ad_accel.meas,
		   p_sensor_data->ad_meas.ad_accel.meas.array,
		   sizeof(float64_t[3]));
	codegen_sensor_input.ad_accel.status =
		p_sensor_data->ad_meas.ad_accel.is_new; // status true is on

	codegen_sensor_input.ad_gyro.meas[0] = p_sensor_data->ad_meas.ad_gyro.meas; // x axis
	codegen_sensor_input.ad_gyro.meas[1] = 0.0;
	codegen_sensor_input.ad_gyro.meas[2] = 0.0;
	codegen_sensor_input.ad_gyro.status =
		p_sensor_data->ad_meas.ad_gyro.is_new; // status true is on

	bool nav_status = false;

	navigation_codegen_entry(p_codegen_stack_data,
							 dt_sec,
							 in_flight_phase,
							 p_ctx->codegen_ctx.x,
							 p_ctx->codegen_ctx.P,
							 &(p_ctx->codegen_ctx.bias),
							 &(p_ctx->codegen_ctx.sensor_filter),
							 &codegen_sensor_input,
							 &(p_output->cov_norm),
							 p_output->roll_state,
							 &(p_output->dynamic_pressure),
							 &nav_status);

	if (nav_status) { // if nav ran
		p_ctx->last_run_tenth_ms = p_input->curr_timestamp_tenth_ms;
	} else {
		log_text(0, "Navigator", "WARN: Nav failed to run");
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
			 "estimator",
			 "imu_timeouts=%lu, encoder_miss=%lu, controller_miss=%lu,",
			 estimator_error_stats.imu_data_timeouts,
			 estimator_error_stats.encoder_data_fails,
			 estimator_error_stats.controller_data_fails);
	log_text(0,
			 "estimator",
			 "pad_filter_fails=%lu, can_log_fails=%lu, invalid_phase=%lu",
			 estimator_error_stats.pad_filter_fails,
			 estimator_error_stats.can_log_fails,
			 estimator_error_stats.invalid_phase_errors);

	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_ESTIMATOR, .error_bitfield = 0};

	return status;
}
