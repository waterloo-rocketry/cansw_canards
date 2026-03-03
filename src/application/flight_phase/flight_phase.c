#include "application/flight_phase/flight_phase.h"
#include "application/can_handler/can_handler.h"
#include "application/estimator/estimator.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "common/math/math-algebra3d.h"
#include "drivers/timer/timer.h"

#include "canlib.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

// TODO: these are made up values, up to FIDO what these actually are
// See the flowchart in the design doc for more context on these
static const uint32_t ACT_DELAY_MS =
	11000; // Q - the minimum time after launch before allowing canards to actuate
static const uint32_t FLIGHT_TIMEOUT_MS =
	49000; // K - the approximate time between launch and apogee

static const uint32_t TASK_TIMEOUT_MS = 1000;

static const float32_t ACCEL_THRESHOLD_LAUNCH =
	20; // minimum acceleration in m/s^2 for a launch to be detected
static const uint32_t NUM_CONSEC_THRESHOLD =
	20; // number of consecutive detection beyond threshold to satisfy for condition
static const uint32_t MAX_TIMESTAMP_DIFFERENCE =
	10; // the max timestamp difference before consec_num_detection resets

/**
 * module health status trackers
 */
typedef struct {
	bool initialized;
	uint32_t loop_run_errs;
	uint32_t state_transitions;
	uint32_t invalid_events;

	// Per-event counters
	struct {
		uint32_t estimator_init;
		uint32_t inj_open;
		uint32_t launch_accel;
		uint32_t act_delay_elapsed;
		uint32_t flight_elapsed;
		uint32_t reset;
	} event_counts;

	// Queue statistics
	uint32_t event_queue_full_count;
} flight_phase_status_t;

// static members
static flight_phase_state_t curr_state = STATE_IDLE;

static QueueHandle_t state_mailbox = NULL;
static QueueHandle_t event_queue = NULL;
static TimerHandle_t act_delay_timer = NULL;
static TimerHandle_t flight_timer = NULL;

// timestamp of the moment of launch
static float launch_timestamp_ms = 0;
// timestamp of the moment actuation allowed started
static float act_allowed_timestamp_ms = 0;

static void act_delay_timer_callback(TimerHandle_t xTimer);
static void flight_timer_callback(TimerHandle_t xTimer);
static w_status_t act_cmd_callback(const can_msg_t *msg);

static flight_phase_status_t flight_phase_status = {
	.initialized = false,
	.loop_run_errs = 0,
	.state_transitions = 0,
	.invalid_events = 0,
	.event_counts = {0},
	.event_queue_full_count = 0,
};

// number of valid sensor detection that would cause a state change
static int consec_num_detection = 0;
static uint32_t last_imu_timestamp = 0; // latest imu timestamp

/**
 * Intialize flight phase module.
 * Creates and allocates state/event queues and timers
 * Sets and populates the default state.
 */
w_status_t flight_phase_init(void) {
	state_mailbox = xQueueCreate(1, sizeof(flight_phase_state_t));
	// larger size in case of burst events like multiple inj valve opens
	event_queue = xQueueCreate(3, sizeof(flight_phase_event_t));
	act_delay_timer = xTimerCreate(
		"act delay", pdMS_TO_TICKS(ACT_DELAY_MS), pdFALSE, NULL, act_delay_timer_callback);
	flight_timer = xTimerCreate(
		"flight", pdMS_TO_TICKS(FLIGHT_TIMEOUT_MS), pdFALSE, NULL, flight_timer_callback);

	if (NULL == state_mailbox || NULL == event_queue || NULL == act_delay_timer ||
		NULL == flight_timer ||
		(W_SUCCESS != can_handler_register_callback(MSG_ACTUATOR_CMD, act_cmd_callback))) {
		log_text(1, "FlightPhaseInit", "ERROR: Failed to create queues/timers/register callback.");
		return W_FAILURE;
	}

	xQueueOverwrite(state_mailbox, &curr_state); // initialize state queue
	flight_phase_status.initialized = true;
	log_text(10, "FlightPhaseInit", "Flight Phase Initialized Successfully.");
	return W_SUCCESS;
}

/**
 * Returns the current state of the state machine
 * Not ISR safe
 * @return STATE_ERROR if getting the current state failed/timed out, otherwise the current flight
 * phase
 */
flight_phase_state_t flight_phase_get_state() {
	flight_phase_state_t state = STATE_ERROR;
	// Use a timeout of 0 to prevent blocking
	if (xQueuePeek(state_mailbox, &state, 0) != pdPASS) {
		// Log error if peek fails - this indicates a potentially serious issue
		log_text(1, "FlightPhaseGetState", "ERROR: Failed to peek state mailbox.");
		return STATE_ERROR;
	}
	return state;
}

/**
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event) {
	// Update event statistics
	switch (event) {
		case EVENT_ESTIMATOR_INIT:
			flight_phase_status.event_counts.estimator_init++;
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
		case EVENT_FLIGHT_ELAPSED:
			flight_phase_status.event_counts.flight_elapsed++;
			break;
		case EVENT_RESET:
			flight_phase_status.event_counts.reset++;
			break;
		default:
			// Unexpected event type
			break;
	}

	if (xQueueSend(event_queue, &event, 0) != pdPASS) {
		log_text(0,
				 "FlightPhaseSendEvent",
				 "ERROR: Failed to send event %d to queue. Queue full?",
				 event);
		flight_phase_status.event_queue_full_count++;
		return W_FAILURE;
	}
	return W_SUCCESS;
	// This cannot be allowed to block because it is called in the timer
	// daemon task
}

/**
 * Timer callback for actuation delay timer
 */
static void act_delay_timer_callback(TimerHandle_t xTimer) {
	(void)xTimer;
	flight_phase_send_event(EVENT_ACT_DELAY_ELAPSED);
}

/**
 * Timer callback for flight elapsed timer
 */
static void flight_timer_callback(TimerHandle_t xTimer) {
	(void)xTimer;
	flight_phase_send_event(EVENT_FLIGHT_ELAPSED);
}

/**
 * Global CAN callback for messages of type MSG_ACTUATOR_CMD
 * Handles OX_INJECTOR_VALVE->OPEN and PROC_ESTIMATOR_INIT->OPEN
 */
static w_status_t act_cmd_callback(const can_msg_t *msg) {
	if ((ACTUATOR_OX_INJECTOR_VALVE == get_actuator_id(msg)) &&
		(ACT_STATE_ON == get_cmd_actuator_state(msg))) {
		return flight_phase_send_event(EVENT_INJ_OPEN);
	} else if ((ACTUATOR_PROC_ESTIMATOR_INIT == get_actuator_id(msg)) &&
			   (ACT_STATE_ON == get_cmd_actuator_state(msg))) {
		return flight_phase_send_event(EVENT_ESTIMATOR_INIT);
	}
	return W_SUCCESS;
}

/**
 * Resets the flight phase state machine to initial state
 */
w_status_t flight_phase_reset(void) {
	return flight_phase_send_event(EVENT_RESET);
}

w_status_t flight_phase_get_flight_ms(uint32_t *flight_ms) {
	if (NULL == flight_ms) {
		return W_INVALID_PARAM;
	}

	// elapsed time is 0 if we havent launched yet
	if (curr_state < STATE_BOOST) {
		*flight_ms = 0;
		return W_SUCCESS;
	} else {
		float current_time_ms = 0;
		if (timer_get_ms(&current_time_ms) != W_SUCCESS) {
			log_text(1, "FlightPhaseFlightTime", "get_ms fail");
			return W_FAILURE;
		}
		*flight_ms = current_time_ms - launch_timestamp_ms;
		return W_SUCCESS;
	}
}

w_status_t flight_phase_get_act_allowed_ms(uint32_t *act_allowed_ms) {
	if (NULL == act_allowed_ms) {
		return W_INVALID_PARAM;
	}

	// elapsed time is 0 if we havent reached act-allowed yet
	if (curr_state < STATE_ACT_ALLOWED) {
		*act_allowed_ms = 0;
		return W_SUCCESS;
	} else {
		float current_time_ms = 0;
		if (timer_get_ms(&current_time_ms) != W_SUCCESS) {
			log_text(1, "FlightPhaseAllowedTime", "get_ms fail");
			return W_FAILURE;
		}
		*act_allowed_ms = current_time_ms - act_allowed_timestamp_ms;
		return W_SUCCESS;
	}
}

/**
 * Updates the input state according to the input event
 * @return W_SUCCESS if the input state was valid, W_FAILURE otherwise (this means W_SUCCESS is
 * returned event if we go into STATE_ERROR)
 */
w_status_t flight_phase_update_state(flight_phase_event_t event, flight_phase_state_t *state) {
	flight_phase_state_t previous_state = *state;

	switch (*state) {
		case STATE_IDLE:
			if (EVENT_ESTIMATOR_INIT == event) {
				*state = STATE_PAD_FILTER;
			} else {
				// Ignore redundant PAD events or other unexpected events
				log_text(
					5, "FlightPhaseUpdate", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_PAD_FILTER:
			if ((EVENT_INJ_OPEN == event) || (EVENT_LAUNCH_ACCEL == event)) {
				*state = STATE_BOOST;
				// flight starts now
				xTimerReset(act_delay_timer, 0);
				xTimerReset(flight_timer, 0);
				timer_get_ms(&launch_timestamp_ms);
			} else {
				// Ignore redundant or unexpected events - this is a known safe state
				log_text(
					5, "FlightPhaseUpdate", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_BOOST:
			if (EVENT_ACT_DELAY_ELAPSED == event) {
				*state = STATE_ACT_ALLOWED;
				// record timestamp of actuation-allowed start (aka we just exited boost phase)
				timer_get_ms(&act_allowed_timestamp_ms);
			} else if (EVENT_FLIGHT_ELAPSED == event) {
				xTimerStop(act_delay_timer, 0);
				*state = STATE_RECOVERY;
			} else {
				// Ignore redundant or unexpected events - this is a known safe state
				log_text(
					5, "FlightPhaseUpdate", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_ACT_ALLOWED:
			if (EVENT_FLIGHT_ELAPSED == event) {
				*state = STATE_RECOVERY;
			} else {
				// Ignore redundant or unexpected events - already in flight
				log_text(
					5, "FlightPhaseUpdate", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_RECOVERY:
			if (EVENT_RESET == event) {
				*state = STATE_IDLE;
			} else {
				// Ignore redundant or unexpected events - already in flight
				log_text(
					5, "FlightPhaseUpdate", "Unexpected event %d in state %d", event, curr_state);
			}
			break;
		case STATE_ERROR:
			if (EVENT_RESET == event) {
				*state = STATE_IDLE;
			} else {
				// Stay in error state, log repeated invalid event
				log_text(1, "FlightPhaseUpdate", "Invalid event %d in STATE_ERROR", event);
			}
			break;
		default:
			log_text(10, "FlightPhaseUpdate", "Unhandled state %d", *state);
			*state = STATE_ERROR; // Ensure state becomes ERROR
			return W_FAILURE;
			break;
	}

	// Only count as a transition if the state actually changed
	if (previous_state != *state) {
		flight_phase_status.state_transitions++;
		consec_num_detection = 0;
	}

	return W_SUCCESS;
}

/**
 * @brief would complete sensor-based detection of state change for flight phase
 * @param state is a pointer to the present state
 * @param all_imu_data is a pointer to the current the imu data
 * @param num_consec_detection is a pointer to the number of consecutive detection made in this
 * particular flight phase
 * @param sensor_event a pointer to return the event the sensor triggers (default returns
 * EVENT_NULL)
 * @return the status of if the function completed properly
 */
w_status_t flight_phase_sensor_detection(const flight_phase_state_t *state,
										 const estimator_all_imus_input_t *all_imu_data,
										 const uint32_t *last_imu_timestamp,
										 int *num_consec_detection,
										 flight_phase_event_t *sensor_event) {
	*sensor_event = EVENT_NULL;
	bool threshold_detection = false;
	flight_phase_event_t trigger_event = EVENT_NULL; // Temporary

	if (true == all_imu_data->pololu.is_dead) { // TO BE CAHNGED to ST IMU
		log_text(5, "FlightPhaseSensorDetection", "ERROR: POLOLU IMU is DEAD");
		*num_consec_detection = 0;
		return W_FAILURE;
	}

	if (*last_imu_timestamp == all_imu_data->pololu.timestamp_imu) {
		log_text(5, "FlightPhaseSensorDetection", "WARNING: POLOLU IMU did not update");
		return W_SUCCESS;

	} else if ((MAX_TIMESTAMP_DIFFERENCE + *last_imu_timestamp) <
			   all_imu_data->pololu.timestamp_imu) {
		log_text(
			5, "FlightPhaseSensorDetection", "WARNING: POLOLU IMU timestamp exceed max difference");
		*num_consec_detection = 0;
	}

	double acceleration_magnitude =
		math_vector3d_norm(&(all_imu_data->pololu.accelerometer)); // TO BE CAHNGED to ST IMU

	switch (*state) {
		case STATE_PAD_FILTER:
			trigger_event = EVENT_LAUNCH_ACCEL;

			if (ACCEL_THRESHOLD_LAUNCH <= acceleration_magnitude) {
				threshold_detection = true;
				log_text(5,
						 "FlightPhaseSensorDetection",
						 "%d Event Occurance %d Detected",
						 trigger_event,
						 *num_consec_detection);
			}
			break;

		default:
			// TODO
			break;
	}

	if (threshold_detection) {
		(*num_consec_detection)++;

		if (NUM_CONSEC_THRESHOLD <= *num_consec_detection) {
			*sensor_event = trigger_event;
			*num_consec_detection = 0;
			log_text(5, "FlightPhaseSensorDetection", "%d Event Trigger", trigger_event);
		}

	} else {
		*num_consec_detection = 0;
	}

	return W_SUCCESS;
}

/**
 * Task to execute the state machine itself. Consumes events and transitions the state
 * Perform checks to determine if a sensor based transition is necessary
 */
void flight_phase_task(void *args) {
	(void)args;
	flight_phase_event_t event;
	flight_phase_event_t sensor_event;
	estimator_all_imus_input_t imu_data;
	w_status_t data_collection_status;

	while (1) {
		data_collection_status = W_SUCCESS;
		sensor_event = EVENT_NULL;

		data_collection_status |= imu_handler_get_data(&imu_data);
		// TO DO ADD Estimator data collection call

		// TO CONSIDER IF consec_num_detection RESETS if data dies
		if (W_SUCCESS == data_collection_status) {
			if (flight_phase_sensor_detection(&curr_state,
											  &imu_data,
											  &last_imu_timestamp,
											  &consec_num_detection,
											  &sensor_event) != W_SUCCESS) {
				log_text(
					5, "FlightPhaseTask", "ERROR: Failed sensor detection in state %d", curr_state);

			} else {
				if (EVENT_NULL != sensor_event) {
					log_text(5,
							 "FlightPhaseTask",
							 "Sensor Triggered %d Event in %d State",
							 sensor_event,
							 curr_state);

					if (flight_phase_send_event(sensor_event) != W_SUCCESS) {
						log_text(5, "FlightPhaseTask", "ERROR: Unable to send sensor event");
					}
				}
			}
			last_imu_timestamp = imu_data.pololu.timestamp_imu; // TO DO UPDATE ST IMU
		} else {
			log_text(5,
					 "FlightPhaseTask",
					 "ERROR: Failed to get data from IMU Handler and/or Estimator");
		}

		if (pdPASS == xQueueReceive(event_queue, &event, pdMS_TO_TICKS(TASK_TIMEOUT_MS))) {
			log_text(
				10, "FlightPhaseTask", "transition\nentry-state:%d\nevent:%d", curr_state, event);

			if (flight_phase_update_state(event, &curr_state) != W_SUCCESS) {
				flight_phase_status.loop_run_errs++;
			}

			log_text(10, "FlightPhaseTask", "exit-state:%d", curr_state);

			// pdPASS is guaranteed for a queue of length 1, so no error check needed
			(void)xQueueOverwrite(state_mailbox, &curr_state);
		} else {
			log_text(10, "FlightPhaseTask", "curr state:%d", curr_state);
		}
	}
}

uint32_t flight_phase_get_status(void) {
	uint32_t status_bitfield = 0;

	// Get current state
	flight_phase_state_t current_state = flight_phase_get_state();

	// Map state enum to descriptive string for logging
	const char *state_strings[] = {"IDLE", "PADFILTER", "BOOST", "ACTALLOWED", "RECOVERY", "ERROR"};

	// Log initialization status and current state
	log_text(0,
			 "FlightPhaseGetStatus",
			 "%s %s (%d) q full: %lu",
			 flight_phase_status.initialized ? "INIT" : "NOT INIT",
			 (current_state <= STATE_ERROR) ? state_strings[current_state] : "???",
			 current_state,
			 flight_phase_status.event_queue_full_count);

	return status_bitfield;
}