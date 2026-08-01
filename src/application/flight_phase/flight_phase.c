#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "common/gnc/gnc_types.h"
#include "common/math/math-algebra3d.h"
#include "drivers/timer/timer.h"

// TODO: these are made up values, up to FIDO what these actually are
// See the flowchart in the design doc for more context on these

static const float32_t ACCEL_THRESHOLD_LAUNCH_M_S2 =
	20; // minimum acceleration in m/s^2 for a launch to be detected

static const uint32_t ACT_DELAY_MS =
	7000; // Q - the minimum time after launch before allowing canards to actuate

// TODO: updated once better times are out
static const uint32_t RECOVERY_LOG_TIMEOUT_MS =
	300000; // K - the approximate time between launch and 2+ minute after nominal flight.
// This is done to make sure we exit high rate logging

// to be matched with main at apogee time
static const uint32_t SLEEPY_LOG_TIMEOUT_MS =
	1600000; // K - the approximate time between launch and main at apogee launch to land time

static const uint32_t NUM_IMUS_REQUIRED_FOR_LAUNCH_ACCEL =
	2; // minimum number of IMUs that must detect launch acceleration for launch event to be
	   // triggered
static const uint32_t NUM_CONSEC_LAUNCH_DETECT_THRESHOLD =
	5; // minimum number of consecutive detections required for an IMU to be considered as detecting
	   // launch

// #define TASK_TIMEOUT_MS 1000

/**
 * module health status trackers
 */
typedef struct {
	bool initialized;
	bool is_in_error_state;
	bool ctx_is_null;
	bool invalid_event;
	bool invalid_actuator_data_format;
	bool dead_imu;

	uint32_t state_transitions;
	uint32_t null_ctx_count;
	uint32_t failed_init_count;
	uint32_t invalid_event_count;
	uint32_t invalid_actuator_data_count;
	uint32_t dead_imu_count;
	uint32_t timer_event_send_fail_count;
	uint32_t sensor_event_send_fail_count;
	uint32_t queue_full_count;

	// Per-event counters
	struct {
		uint32_t pad_filter;
		uint32_t ignitor;
		uint32_t inj_open;
		uint32_t launch_accel;
		uint32_t act_delay_elapsed;
		uint32_t recovery_rate;
		uint32_t sleep_rate;
		uint32_t reset;
	} event_counts;

	bool is_queue_full;
} flight_phase_status_t;

// static members
static QueueHandle_t event_queue = NULL;

static w_status_t act_cmd_callback(const can_msg_t *msg);

static flight_phase_status_t flight_phase_status = (flight_phase_status_t){.event_counts = {0}};

/**
 * Intialize flight phase module.
 * Creates and allocates event queues
 */
w_status_t flight_phase_init(void) {
	// larger size in case of burst events like multiple inj valve opens
	event_queue = xQueueCreate(3, sizeof(flight_phase_event_t));

	if ((NULL == event_queue) ||
		(W_SUCCESS !=
		 can_handler_act_cmd_register_callback(ACTUATOR_OX_INJECTOR_VALVE, act_cmd_callback)) ||
		(W_SUCCESS != can_handler_act_cmd_register_callback(ACTUATOR_IGNITION, act_cmd_callback)) ||
		(W_SUCCESS !=
		 can_handler_act_cmd_register_callback(ACTUATOR_CANARD_PAD_FILTER, act_cmd_callback))) {
		flight_phase_status.failed_init_count++;
		return W_FAILURE;
	}

	flight_phase_status.initialized = true;
	return W_SUCCESS;
}

/**
 * Send a flight phase event to the state machine
 * ISR safe
 */
w_status_t flight_phase_send_event_isr(flight_phase_event_t event) {
	// Update event statistics
	switch (event) {
		case EVENT_PAD_FILTER:
			flight_phase_status.event_counts.pad_filter++;
			break;
		case EVENT_IGNITOR:
			flight_phase_status.event_counts.ignitor++;
			break;
		case EVENT_INJ_OPEN:
			flight_phase_status.event_counts.inj_open++;
			break;
		case EVENT_LAUNCH_ACCEL:
			flight_phase_status.event_counts.launch_accel++;
			break;
		case EVENT_ACT_DELAY_ELAPSED:
			flight_phase_status.event_counts.act_delay_elapsed++;
			break;
		case EVENT_RECOVERY_START:
			flight_phase_status.event_counts.recovery_rate++;
			break;
		case EVENT_SLEEP_START:
			flight_phase_status.event_counts.sleep_rate++;
			break;
		default:
			flight_phase_status.invalid_event = true;
			flight_phase_status.invalid_event_count++;
			break;
	}

	// Allow sending from ISR
	if (xQueueSendFromISR(event_queue, &event, 0) != pdPASS) {
		flight_phase_status.is_queue_full = true;
		flight_phase_status.queue_full_count++;
		return W_FAILURE;
	}
	return W_SUCCESS;
}

/**
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event) {
	// Update event statistics
	switch (event) {
		case EVENT_PAD_FILTER:
			flight_phase_status.event_counts.pad_filter++;
			break;
		case EVENT_IGNITOR:
			flight_phase_status.event_counts.ignitor++;
			break;
		case EVENT_INJ_OPEN:
			flight_phase_status.event_counts.inj_open++;
			break;
		case EVENT_LAUNCH_ACCEL:
			flight_phase_status.event_counts.launch_accel++;
			break;
		case EVENT_ACT_DELAY_ELAPSED:
			flight_phase_status.event_counts.act_delay_elapsed++;
			break;
		case EVENT_RECOVERY_START:
			flight_phase_status.event_counts.recovery_rate++;
			break;
		case EVENT_SLEEP_START:
			flight_phase_status.event_counts.sleep_rate++;
			break;
		default:
			flight_phase_status.invalid_event = true;
			flight_phase_status.invalid_event_count++;
			break;
	}

	if (xQueueSend(event_queue, &event, 0) != pdPASS) {
		flight_phase_status.is_queue_full = true;
		flight_phase_status.queue_full_count++;
		return W_FAILURE;
	}
	return W_SUCCESS;
}

/**
 * Global CAN callback for messages of type MSG_ACTUATOR_CMD
 * Handles OX_INJECTOR_VALVE->OPEN and CANARD_PAD_FILTER->OPEN
 */
static w_status_t act_cmd_callback(const can_msg_t *msg) {
	can_actuator_id_t msg_id;
	can_actuator_state_t msg_state;

	if ((get_actuator_id(msg, &msg_id) != W_SUCCESS) ||
		(get_cmd_actuator_state(msg, &msg_state) != W_SUCCESS)) {
		flight_phase_status.invalid_actuator_data_format = true;
		flight_phase_status.invalid_actuator_data_count++;
		return W_FAILURE;
	}

	if ((ACTUATOR_OX_INJECTOR_VALVE == msg_id) && (ACT_STATE_ON == msg_state)) {
		return flight_phase_send_event(EVENT_INJ_OPEN);
	} else if ((ACTUATOR_IGNITION == msg_id) && (ACT_STATE_ON == msg_state)) {
		return flight_phase_send_event(EVENT_IGNITOR);
	} else if ((ACTUATOR_CANARD_PAD_FILTER == msg_id) && (ACT_STATE_ON == msg_state)) {
		return flight_phase_send_event(EVENT_PAD_FILTER);
	}
	return W_SUCCESS;
}

flight_phase_event_t flight_phase_get_next_event(void) {
	flight_phase_event_t queue_event = EVENT_NONE;
	// no waiting. Return immediately if no events are in the queue
	if (pdPASS != xQueueReceive(event_queue, &queue_event, 0)) {
		return EVENT_NONE;
	}
	return queue_event;
}

/**
 * Updates the input state according to the input event
 * @return W_SUCCESS if the input state was valid, W_FAILURE otherwise (this means W_SUCCESS is
 * returned event if we go into STATE_ERROR)
 */
fsm_state_t flight_phase_update_state(flight_phase_event_t event, fsm_state_t curr_state,
									  flight_phase_ctx_t *p_ctx) {
	if (NULL == p_ctx) {
		flight_phase_status.null_ctx_count++;
		flight_phase_status.ctx_is_null = true;
		// just return the current state if invalid
		return curr_state;
	}

	if (EVENT_NONE == event) {
		return curr_state;
	}

	fsm_state_t new_state = curr_state;

	switch (curr_state) {
		case STATE_IDLE:
			if (EVENT_PAD_FILTER == event) {
				new_state = STATE_PAD_FILTER;
			}
			break;

		case STATE_PAD_FILTER:
			if ((EVENT_LAUNCH_ACCEL == event) || (EVENT_INJ_OPEN == event)) {
				new_state = STATE_BOOST;
				// flight starts now
				timer_get_ms(&(p_ctx->launch_timestamp_ms));

			} else if (EVENT_IGNITOR == event) {
				new_state = STATE_PAD_NAV;
			}
			break;

		case STATE_PAD_NAV:
			if ((EVENT_LAUNCH_ACCEL == event) || (EVENT_INJ_OPEN == event)) {
				new_state = STATE_BOOST;
				// flight starts now
				timer_get_ms(&(p_ctx->launch_timestamp_ms));
			}
			break;

		case STATE_BOOST:
			if (EVENT_INJ_OPEN == event) {
				new_state = STATE_BOOST;
				// restart flight timer constantly till lose signal
				timer_get_ms(&(p_ctx->launch_timestamp_ms));

			} else if (EVENT_ACT_DELAY_ELAPSED == event) {
				new_state = STATE_ACT_ALLOWED;
				// record timestamp of actuation-allowed start (aka we just exited boost phase)
				timer_get_ms(&(p_ctx->act_allowed_timestamp_ms));
			}
			break;

		case STATE_ACT_ALLOWED:
			if (EVENT_RECOVERY_START == event) {
				new_state = STATE_RECOVERY;
			}
			break;

		case STATE_RECOVERY:
			if (EVENT_SLEEP_START == event) {
				new_state = STATE_SLEEPY;
			}
			break;

		case STATE_SLEEPY:
			// event not useful in this state, ignore - no transition
			break;

		// deprecate time?
		case STATE_ERROR:
			// Stay in error state, count repeated invalid event
			flight_phase_status.is_in_error_state = true;
			break;
		default:
			flight_phase_status.is_in_error_state = true;
			new_state = curr_state; // return the same state
			break;
	}

	// Only count as a transition if the state actually changed
	if (new_state != curr_state) {
		flight_phase_status.state_transitions++;
	}

	return new_state;
}

/**
 * @brief performs any timer based state transition detection
 * @param p_context is the global flight phase global context
 * @param curr_state current fsm state
 * @param timestamp_ms is the current timestamp
 * @return generated timer event
 */
static flight_phase_event_t flight_phase_timer_detection(const flight_phase_ctx_t *p_ctx,
														 const fsm_state_t curr_state,
														 const uint32_t timestamp_ms) {
	if (NULL == p_ctx) {
		flight_phase_status.null_ctx_count++;
		flight_phase_status.ctx_is_null = true;
		// just return the current state if invalid
		return EVENT_NONE;
	}

	flight_phase_event_t output_state = EVENT_NONE;

	uint32_t launch_elapsed_time_ms = timestamp_ms - p_ctx->launch_timestamp_ms;

	switch (curr_state) {
		// act delayed state
		case STATE_BOOST:
			if (ACT_DELAY_MS <= launch_elapsed_time_ms) {
				output_state = EVENT_ACT_DELAY_ELAPSED;
			}
			break;

		// act recovery state
		case STATE_ACT_ALLOWED:
			if (RECOVERY_LOG_TIMEOUT_MS <= launch_elapsed_time_ms) {
				output_state = EVENT_RECOVERY_START;
			}
			break;

		// act sleep state
		case STATE_RECOVERY:
			if (SLEEPY_LOG_TIMEOUT_MS <= launch_elapsed_time_ms) {
				output_state = EVENT_SLEEP_START;
			}
			break;

		default:
			output_state = EVENT_NONE;
			break;
	}

	return output_state;
}

/**
 * @brief increments number of consecutive detections if the IMU is alive and
 *        its acceleration magnitude meets the launch threshold, resets otherwise.
 * @param is_new whether this IMU is currently receiving new data
 * @param accel pointer to the IMU's acceleration vector
 * @param num_consec_detection pointer to this IMU's number of consecutive launch detections
 * @param imu_name name of the IMU for logging purposes
 */
static void process_imu_meas(bool is_new, const vector3d_t *accel, uint8_t *num_consec_detection,
							 const char *imu_name) {
	if (!is_new) {
		(void)imu_name;
		flight_phase_status.dead_imu = true;
		flight_phase_status.dead_imu_count++;
		(*num_consec_detection) = 0;
		return;
	}

	float64_t accel_magnitude = math_vector3d_norm(accel);

	if (accel_magnitude >= ACCEL_THRESHOLD_LAUNCH_M_S2) {
		(*num_consec_detection)++;
	} else {
		(*num_consec_detection) = 0;
	}
}

/**
 * @brief performs any sensor based state transition detection
 * @param p_context pointer to the global flight phase global context
 * @param curr_state current fsm state
 * @param p_sensor_data pointer to the current sensor data
 * @return generated sensor event
 */
static flight_phase_event_t flight_phase_sensor_detection(flight_phase_ctx_t *p_ctx,
														  const fsm_state_t curr_state,
														  const all_sensors_data_t *p_sensor_data) {
	if ((curr_state != STATE_PAD_FILTER) && (curr_state != STATE_PAD_NAV)) {
		p_ctx->num_consec_board = 0;
		p_ctx->num_consec_movella = 0;
		p_ctx->num_consec_ad = 0;
		return EVENT_NONE;
	}
	bool board_imu_new = p_sensor_data->board_meas.board_imu.is_new;
	bool mti_imu_new = p_sensor_data->mti_meas.mti_accel.is_new;
	bool ad_imu_new = p_sensor_data->ad_meas.ad_accel.is_new;

	process_imu_meas(board_imu_new,
					 &p_sensor_data->board_meas.board_imu.accel,
					 &p_ctx->num_consec_board,
					 "board");
	process_imu_meas(mti_imu_new,
					 &p_sensor_data->mti_meas.mti_accel.meas,
					 &p_ctx->num_consec_movella,
					 "movella");
	process_imu_meas(
		ad_imu_new, &p_sensor_data->ad_meas.ad_accel.meas, &p_ctx->num_consec_ad, "ad");

	uint32_t num_imu_detect_launch = 0;

	if (p_ctx->num_consec_board >= NUM_CONSEC_LAUNCH_DETECT_THRESHOLD) {
		num_imu_detect_launch++;
	}
	if (p_ctx->num_consec_movella >= NUM_CONSEC_LAUNCH_DETECT_THRESHOLD) {
		num_imu_detect_launch++;
	}
	if (p_ctx->num_consec_ad >= NUM_CONSEC_LAUNCH_DETECT_THRESHOLD) {
		num_imu_detect_launch++;
	}

	if (num_imu_detect_launch >= NUM_IMUS_REQUIRED_FOR_LAUNCH_ACCEL) {
		p_ctx->num_consec_board = 0;
		p_ctx->num_consec_movella = 0;
		p_ctx->num_consec_ad = 0;
		return EVENT_LAUNCH_ACCEL;
	}

	return EVENT_NONE;
}

/**
 * @brief generate syncronous flight phase evnets
 * @param p_context is the global flight phase global context
 * @param curr_state current fsm state
 * @param timestamp_ms is the current timestamp
 * @param p_sensor_data pointer to the current sensor data
 * @return the status of function
 */
w_status_t flight_phase_gen_sync_events(flight_phase_ctx_t *p_ctx, const fsm_state_t curr_state,
										const uint32_t timestamp_ms,
										const all_sensors_data_t *p_sensor_data) {
	w_status_t status = W_SUCCESS;

	// timer
	flight_phase_event_t timer_event =
		flight_phase_timer_detection(p_ctx, curr_state, timestamp_ms);

	if (EVENT_NONE != timer_event) {
		if (flight_phase_send_event(timer_event) != W_SUCCESS) {
			flight_phase_status.timer_event_send_fail_count++;
			status = W_FAILURE;
		}
	}

	// sensor
	flight_phase_event_t sensor_event =
		flight_phase_sensor_detection(p_ctx, curr_state, p_sensor_data);

	if (EVENT_NONE != sensor_event) {
		if (flight_phase_send_event(sensor_event) != W_SUCCESS) {
			flight_phase_status.sensor_event_send_fail_count++;
			status = W_FAILURE;
		}
	}

	return status;
}

health_status_t flight_phase_get_status(void) {
	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_FLIGHT_PHASE,
							  .error_bitfield = 0};

	log_text(10,
			 LOG_LVL_INFO,
			 "FlightPhase",
			 "init=%d, failed_init=%lu, null_ctx=%lu",
			 flight_phase_status.initialized,
			 flight_phase_status.failed_init_count,
			 flight_phase_status.null_ctx_count);

	log_text(10,
			 LOG_LVL_INFO,
			 "FlightPhase",
			 "state_transitions=%lu, invalid_event=%lu, dead_imu=%lu",
			 flight_phase_status.state_transitions,
			 flight_phase_status.invalid_event_count,
			 flight_phase_status.dead_imu_count);

	log_text(10,
			 LOG_LVL_INFO,
			 "FlightPhase",
			 "invalid_act_data=%lu, timer_send_fail=%lu, sensor_send_fail=%lu, queue_full=%lu",
			 flight_phase_status.invalid_actuator_data_count,
			 flight_phase_status.timer_event_send_fail_count,
			 flight_phase_status.sensor_event_send_fail_count,
			 flight_phase_status.queue_full_count);

	log_text(10,
			 LOG_LVL_INFO,
			 "FlightPhase",
			 "pad_filter=%lu, ignitor=%lu, inj_open=%lu, launch_accel=%lu",
			 flight_phase_status.event_counts.pad_filter,
			 flight_phase_status.event_counts.ignitor,
			 flight_phase_status.event_counts.inj_open,
			 flight_phase_status.event_counts.launch_accel);

	log_text(10,
			 LOG_LVL_INFO,
			 "FlightPhase",
			 "act_delay_elapsed=%lu, recovery_rate=%lu, sleep_rate=%lu, reset=%lu",
			 flight_phase_status.event_counts.act_delay_elapsed,
			 flight_phase_status.event_counts.recovery_rate,
			 flight_phase_status.event_counts.sleep_rate,
			 flight_phase_status.event_counts.reset);

	// Null context pointer
	if (flight_phase_status.ctx_is_null) {
		flight_phase_status.ctx_is_null = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET;
	}

	// Full event queue
	if (flight_phase_status.is_queue_full) {
		flight_phase_status.is_queue_full = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_OS_OFFSET;
	}

	// One of the IMU's sent old data
	if (flight_phase_status.dead_imu) {
		flight_phase_status.dead_imu = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_NO_DATA_OFFSET;
	}

	// Not initialized
	if (!flight_phase_status.initialized) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_NOT_INIT_OFFSET;
	}

	// Invalid event sent
	if (flight_phase_status.invalid_event) {
		flight_phase_status.invalid_event = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_UNEXPECTED_EVENT_OFFSET;
	}

	// Error state flight phase
	if (flight_phase_status.is_in_error_state) {
		flight_phase_status.is_in_error_state = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_ERROR_STATE_OFFSET;
	}

	// Invalid data actuator data
	if (flight_phase_status.invalid_actuator_data_format) {
		flight_phase_status.invalid_actuator_data_format = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_COMM_FAILURE_OFFSET;
	}

	return status;
}
