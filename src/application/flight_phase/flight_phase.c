#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "drivers/timer/timer.h"

// TODO: these are made up values, up to FIDO what these actually are
// See the flowchart in the design doc for more context on these
static const uint32_t ACT_DELAY_MS =
	11000; // Q - the minimum time after launch before allowing canards to actuate
static const uint32_t FLIGHT_TIMEOUT_MS =
	49000; // K - the approximate time between launch and apogee

// #define TASK_TIMEOUT_MS 1000

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
		uint32_t act_delay_elapsed;
		uint32_t flight_elapsed;
		uint32_t reset;
	} event_counts;

	// Queue statistics
	uint32_t event_queue_full_count;
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
		(W_SUCCESS != can_handler_register_callback(MSG_ACTUATOR_CMD, act_cmd_callback))) {
		log_text(1, "FlightPhase", "ERROR: Failed to create queues/timers/register callback.");
		return W_FAILURE;
	}

	flight_phase_status.initialized = true;
	log_text(10, "FlightPhase", "Flight Phase Initialized Successfully.");
	return W_SUCCESS;
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
		log_text(0, "FlightPhase", "ERROR: Failed to send event %d to queue. Queue full?", event);
		flight_phase_status.event_queue_full_count++;
		return W_FAILURE;
	}
	return W_SUCCESS;
}

/**
 * Global CAN callback for messages of type MSG_ACTUATOR_CMD
 * Handles OX_INJECTOR_VALVE->OPEN and PROC_ESTIMATOR_INIT->OPEN
 */
static w_status_t act_cmd_callback(const can_msg_t *msg) {
	can_actuator_id_t msg_id;
	can_actuator_state_t msg_state;

	if ((get_actuator_id(msg, &msg_id) != W_SUCCESS) ||
		(get_cmd_actuator_state(msg, &msg_state) != W_SUCCESS)) {
		log_text(1, "FlightPhase", "invalid actuator data");
		return W_FAILURE;
	}

	if ((ACTUATOR_OX_INJECTOR_VALVE == msg_id) && (ACT_STATE_ON == msg_state)) {
		return flight_phase_send_event(EVENT_INJ_OPEN);
	} else if ((ACTUATOR_CANARD_PAD_FILTER == msg_id) && (ACT_STATE_ON == msg_state)) {
		return flight_phase_send_event(EVENT_ESTIMATOR_INIT);
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
 * Resets the flight phase state machine to initial state
 */
w_status_t flight_phase_reset(void) {
	return flight_phase_send_event(EVENT_RESET);
}

/**
 * Updates the input state according to the input event
 * @return W_SUCCESS if the input state was valid, W_FAILURE otherwise (this means W_SUCCESS is
 * returned event if we go into STATE_ERROR)
 */
fsm_state_t flight_phase_update_state(flight_phase_event_t event, fsm_state_t curr_state,
									  flight_phase_ctx_t *p_ctx) {
	if (NULL == p_ctx) {
		log_text(5, "FlightPhase", "ERROR: Invalid ptrs in update states");
		// just return the current state if invalid
		return curr_state;
	}

	if (EVENT_NONE == event) {
		return curr_state;
	}

	fsm_state_t new_state = curr_state;

	switch (curr_state) {
		case STATE_IDLE:
			if (EVENT_ESTIMATOR_INIT == event) {
				new_state = STATE_SE_INIT;
			} else if (EVENT_INJ_OPEN == event) {
				// allowed to skip pad filter state in case it was forgotten or failed etc.
				// not ideal but would rather run without pad filter than not fly at all
				new_state = STATE_BOOST;
				// flight starts now
				timer_get_ms(&(p_ctx->launch_timestamp_ms));
			} else {
				// Ignore redundant PAD events or other unexpected events
				log_text(5, "FlightPhase", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_SE_INIT:
			if (EVENT_INJ_OPEN == event) {
				new_state = STATE_BOOST;
				// flight starts now
				timer_get_ms(&(p_ctx->launch_timestamp_ms));
			} else {
				// Ignore redundant or unexpected events - this is a known safe state
				log_text(5, "FlightPhase", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_BOOST:
			if (EVENT_ACT_DELAY_ELAPSED == event) {
				new_state = STATE_ACT_ALLOWED;
				// record timestamp of actuation-allowed start (aka we just exited boost phase)
				timer_get_ms(&(p_ctx->act_allowed_timestamp_ms));
			} else if (EVENT_FLIGHT_ELAPSED == event) {
				new_state = STATE_RECOVERY;
			} else {
				// Ignore redundant or unexpected events - this is a known safe state
				log_text(5, "FlightPhase", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_ACT_ALLOWED:
			if (EVENT_FLIGHT_ELAPSED == event) {
				new_state = STATE_RECOVERY;
			} else {
				// Ignore redundant or unexpected events - already in flight
				log_text(5, "FlightPhase", "Unexpected event %d in state %d", event, curr_state);
			}
			break;

		case STATE_RECOVERY:
			if (EVENT_RESET == event) {
				new_state = STATE_IDLE;
			} else {
				// Ignore redundant or unexpected events - already in flight
				log_text(5, "FlightPhase", "Unexpected event %d in state %d", event, curr_state);
			}
			break;
		case STATE_ERROR:
			if (EVENT_RESET == event) {
				new_state = STATE_IDLE;
			} else {
				// Stay in error state, log repeated invalid event
				log_text(1, "FlightPhase", "Invalid event %d in STATE_ERROR", event);
			}
			break;
		default:
			log_text(10, "FlightPhase", "Unhandled state %d", curr_state);
			new_state = curr_state; // return thee same state
			break;
	}

	// Only count as a transition if the state actually changed
	if (new_state != curr_state) {
		flight_phase_status.state_transitions++;
	}

	return new_state;
}

uint32_t flight_phase_get_status(void) {
	uint32_t status_bitfield = 0;

	// Get current state
	// TODO: refactor this to match new design
	// fsm_state_t current_state = flight_phase_get_state();
	uint8_t current_state = 0;

	// Map state enum to descriptive string for logging
	const char *state_strings[] = {"IDLE", "PADFILTER", "BOOST", "ACTALLOWED", "RECOVERY", "ERROR"};

	// Log initialization status and current state
	log_text(0,
			 "flight_phase",
			 "%s %s (%d) q full: %lu",
			 flight_phase_status.initialized ? "INIT" : "NOT INIT",
			 (current_state <= STATE_ERROR) ? state_strings[current_state] : "???",
			 current_state,
			 flight_phase_status.event_queue_full_count);

	return status_bitfield;
}

/**
 * @brief performs any timer based state transition detection
 * @param p_context is the global flight phase global context
 * @param curr_state current fsm state
 * @param timestamp_ms is the current timestamp
 * @return generated timer event
 */
static flight_phase_event_t flight_phase_timer_detection(flight_phase_ctx_t *p_ctx,
														 const fsm_state_t curr_state,
														 const uint32_t timestamp_ms) {
	return EVENT_NONE;
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
			log_text(1, "flight_phase", "ERROR: timer send event failed.");
			status = W_FAILURE;
		}
	}

	// sensor
	flight_phase_event_t sensor_event =
		flight_phase_sensor_detection(p_ctx, curr_state, p_sensor_data);

	if (EVENT_NONE != sensor_event) {
		if (flight_phase_send_event(sensor_event) != W_SUCCESS) {
			log_text(1, "flight_phase", "ERROR: sensor send event failed.");
			status = W_FAILURE;
		}
	}

	return status;
}
