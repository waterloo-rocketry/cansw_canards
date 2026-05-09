#include "math.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "canlib.h"

#include "application/can_handler/can_handler.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"

// ---------- private variables ----------
// IDEAL task period, for calculating CAN send rate limiter
static const uint32_t ESTIMATOR_TASK_PERIOD_MS = 5;
// Rate limit CAN tx: only send data at 10Hz, every 100ms
#define ESTIMATOR_CAN_TX_PERIOD_MS 100
#define ESTIMATOR_CAN_TX_RATE (ESTIMATOR_CAN_TX_PERIOD_MS / ESTIMATOR_TASK_PERIOD_MS)
// wait for imu data for >5ms to avoid false failure if imu takes like 5.1ms
#define DATA_WAIT_MS 10

// Error tracking
static estimator_error_data_t estimator_error_stats = {0};

// TODO remove this once we get encorder reintegrated into the system
// latest encoder reading (millidegrees) from CAN
static QueueHandle_t encoder_data_queue_rad = NULL;

// ---------- private static functions ----------
/**
 * callback to be registered with CAN handler for encoder msgs.
 * msg_type = MSG_SENSOR_ANALOG
 */
static w_status_t can_encoder_msg_callback(const can_msg_t *msg) {
	can_analog_sensor_id_t sensor_id;
	uint16_t raw_data;

	if (get_analog_data(msg, &sensor_id, &raw_data) == false) {
		log_text(1, "Estimator", "get_analog_data fail");
		return W_FAILURE;
	}

	if (SENSOR_CANARD_ENCODER_1 == sensor_id) {
		// shift to [-10000, 10000] mdeg then convert to radians. raw is centered at 32768
		int16_t angle_mdeg = (int16_t)(raw_data - 32768);
		float encoder_val_rad = ((int32_t)angle_mdeg * (RAD_PER_DEG) / 1000.0f);

		// send to internal data queue
		xQueueOverwrite(encoder_data_queue_rad, &encoder_val_rad);
	}
	return W_SUCCESS;
}

// ---------- public functions ----------

w_status_t estimator_init(void) {
	// register the callback for the encoder can msgs
	if (W_SUCCESS != can_handler_register_callback(MSG_SENSOR_ANALOG, can_encoder_msg_callback)) {
		log_text(1, "adc", "initfailenc");
		return W_FAILURE;
	}

	// create queues for imu data, encoder data, and controller cmd
	encoder_data_queue_rad = xQueueCreate(1, sizeof(float));

	if (NULL == encoder_data_queue_rad) {
		log_text(1, "adc", "initfailq");
		return W_FAILURE;
	}

	// Initialize error tracking
	estimator_error_stats.is_init = true;
	estimator_error_stats.imu_data_timeouts = 0;
	estimator_error_stats.encoder_data_fails = 0;
	estimator_error_stats.controller_data_fails = 0;
	estimator_error_stats.pad_filter_fails = 0;
	estimator_error_stats.can_log_fails = 0;
	estimator_error_stats.invalid_phase_errors = 0;

	return W_SUCCESS;
}

w_status_t estimator_step(estimator_module_ctx_t *ctx, navigator_input_t *p_input,
						  navigator_output_t *p_output, uint32_t loop_count) {
	(void)ctx;
	(void)p_input;
	(void)p_output;
	(void)loop_count;
	return W_SUCCESS;
}

w_status_t estimator_log_state_to_can(const x_state_t *current_state) {
	can_msg_t msg;
	uint32_t current_time_ms;
	w_status_t status = W_SUCCESS;

	if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
		current_time_ms = 0; // Default to 0 if timer fails
	}
	uint16_t timestamp_16bit = (uint16_t)current_time_ms;

	// Iterate through all defined state IDs
	for (can_state_est_id_t state_id = 0; state_id < STATE_ID_ENUM_MAX; ++state_id) {
		// The x_state_t union maps directly to the enum order if accessed as an array
		// Convert the doubles in x_state_t to floats for CAN message
		float state_value = (float)current_state->array[state_id];

		if (!build_state_est_data_msg(PRIO_LOW, timestamp_16bit, state_id, &state_value, &msg)) {
			log_text(0, "Estimator", "Failed to build CAN message for state ID %d", state_id);
			estimator_error_stats.can_log_fails++;
			status = W_FAILURE; // Mark as failure but continue trying other states
			continue;
		}

		if (W_SUCCESS != can_handler_transmit(&msg)) {
			log_text(0, "Estimator", "Failed to transmit CAN message for state ID %d", state_id);
			estimator_error_stats.can_log_fails++;
			status = W_FAILURE; // Mark as failure but continue trying other states
		}
	}

	return status;
}

uint32_t estimator_get_status(void) {
	uint32_t status_bitfield = 0;

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

	return status_bitfield;
}

void estimator_task(void *argument) {
	(void)argument;
	// TickType_t last_wake_time;
	// last_wake_time = xTaskGetTickCount();

	// track how many times we ran estimator to ratelimit the CAN tx per N loops
	uint32_t estimator_loop_counter = 0;

	// estimator_module persistent ctx for the whole program
	// estimator_module_ctx_t g_estimator_ctx = {0};

	// initialize ctx timestamp to current time
	uint32_t init_time_tenth_ms = 0;
	if (timer_get_tenth_ms(&init_time_tenth_ms) != W_SUCCESS) {
		proc_handle_fatal_error("estini");
	}
	// g_estimator_ctx.t_sec = ((float64_t)init_time_tenth_ms) / 10000.0; // convert 0.1ms to
	// seconds

	// initialize ctx to reasonable values in case pad filter never runs
	// g_estimator_ctx.x.attitude.w = 1.0;
	// g_estimator_ctx.x.attitude.x = 0.0;
	// g_estimator_ctx.x.attitude.y = 0.0;
	// g_estimator_ctx.x.attitude.z = 0.0;
	// g_estimator_ctx.x.altitude = 420;
	// g_estimator_ctx.x.CL = 3;

	log_text(10, "EstimatorTask", "Estimator task started.");

	while (true) {
		// w_status_t run_status = estimator_step(&g_estimator_ctx, estimator_loop_counter);

		// if (run_status != W_SUCCESS) {
		// 	log_text(
		// 		1, "EstimatorTask", "ERROR: Estimator run loop failed (status: %d).", run_status);
		// }

		// estimator_loop_counter++;

		// // do delay here instead of inside the run to unify the timing
		// vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ESTIMATOR_TASK_PERIOD_MS));
	}
}
