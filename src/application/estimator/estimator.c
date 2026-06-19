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
// TODO: if kept change to static const
#define ESTIMATOR_CAN_TX_PERIOD_MS 100
#define ESTIMATOR_CAN_TX_RATE (ESTIMATOR_CAN_TX_PERIOD_MS / ESTIMATOR_TASK_PERIOD_MS)
// wait for imu data for >5ms to avoid false failure if imu takes like 5.1ms
#define DATA_WAIT_MS 10

// Error tracking
static estimator_error_data_t estimator_error_stats = {0};

// TODO remove this once we get encorder reintegrated into the system
// latest encoder reading (millidegrees) from CAN
static QueueHandle_t encoder_data_queue_rad = NULL;

// ---------- public functions ----------

w_status_t estimator_init(void) {
	// create queues for imu data, encoder data, and controller cmd
	encoder_data_queue_rad = xQueueCreate(1, sizeof(float));

	if (NULL == encoder_data_queue_rad) {
		log_text(1, "adc", "initfailq");
		return W_FAILURE;
	}

	// Initialize error tracking
	estimator_error_stats = (estimator_error_data_t){.is_init = true};

	return W_SUCCESS;
}

w_status_t estimator_step(estimator_module_ctx_t *ctx, const navigator_input_t *p_input,
						  navigator_output_t *p_output) {
	(void)ctx;
	(void)p_input;
	(void)p_output;
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

	// TODO: Redo how messages are built and sent
	// Iterate through all defined state IDs
	// for (can_state_est_id_t state_id = 0; state_id < STATE_ID_ENUM_MAX; ++state_id) {
	// 	// The x_state_t union maps directly to the enum order if accessed as an array
	// 	// Convert the doubles in x_state_t to floats for CAN message
	// 	float state_value = (float)current_state->array[state_id];

	// 	if (!build_state_est_data_msg(PRIO_LOW, timestamp_16bit, state_id, &state_value, &msg)) {
	// 		log_text(0, "Estimator", "Failed to build CAN message for state ID %d", state_id);
	// 		estimator_error_stats.can_log_fails++;
	// 		status = W_FAILURE; // Mark as failure but continue trying other states
	// 		continue;
	// 	}

	// 	if (W_SUCCESS != can_handler_transmit(&msg)) {
	// 		log_text(0, "Estimator", "Failed to transmit CAN message for state ID %d", state_id);
	// 		estimator_error_stats.can_log_fails++;
	// 		status = W_FAILURE; // Mark as failure but continue trying other states
	// 	}
	// }

	return status;
}

health_status_t estimator_get_status(void) {
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
