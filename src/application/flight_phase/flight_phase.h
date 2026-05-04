#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "Queue.h"
#include "rocketlib/include/common.h"

/**
 * Enum representing phase of flight (state machine state)
 */
typedef enum {
	STATE_IDLE,
	STATE_SE_INIT,
	STATE_BOOST,
	STATE_ACT_ALLOWED,
	STATE_RECOVERY,
	STATE_ERROR
} flight_phase_state_t;

/**
 * Enum representing a state transition event
 */
typedef enum {
	EVENT_NONE,
	EVENT_ESTIMATOR_INIT,
	EVENT_INJ_OPEN,
	EVENT_ACT_DELAY_ELAPSED,
	EVENT_FLIGHT_ELAPSED,
	EVENT_RESET
} flight_phase_event_t;

typedef struct {
	flight_phase_state_t curr_state;
	QueueHandle_t event_queue; // TODO: should event queue live here???????
	uint32_t launch_timestamp_ms;
	uint32_t act_allowed_timestamp_ms;
	uint8_t num_consec_detections;
} flight_phase_ctx_t;

#include "application/estimator/estimator.h"

/**
 * Intialize flight phase module.
 * Creates and allocates state/event queues and timers
 * Sets and populates the default state.
 */
w_status_t flight_phase_init(void);

/**
 * Task to execute the state machine itself. Consumes events and transitions the state
 */
void flight_phase_task(void *args);

/**
 * Returns the current state of the state machine
 * Not ISR safe
 * @return STATE_ERROR if getting the current state failed/timed out, otherwise the current flight
 * phase
 */
flight_phase_state_t flight_phase_get_state(void);

/**
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event);

/**
 * process 1 transition.
 */
w_status_t flight_phase_update_state(flight_phase_event_t event, flight_phase_ctx_t *p_context);

/**
 * Resets the flight phase state machine to initial state
 */
w_status_t flight_phase_reset(void);

/**
 * @brief Reports the current status of the flight phase module
 * @return CAN board status bitfield
 * @details Logs initialization status, state machine state, event statistics,
 * and error conditions for the flight phase state machine
 */
uint32_t flight_phase_get_status(void);

/**
 * return time (ms) elapsed since the moment of launch
 */
w_status_t flight_phase_get_flight_ms(uint32_t *flight_ms);

/**
 * return time (ms) elapsed since the moment actuation-allowed started
 */
w_status_t flight_phase_get_act_allowed_ms(uint32_t *act_allowed_ms);

/**
 * @brief performs any timer based state transition detection
 * @param p_context is the global flight phase global context
 * @param timestamp_ms is the current timestamp
 * @param p_timer_event is the pointer to any generated timer event
 * @return the status of the function
 */
w_status_t flight_phase_timer_detection(flight_phase_ctx_t *p_context, const uint32_t timestamp_ms,
										flight_phase_event_t *p_timer_event);

/**
 * @brief performs any sensor based state transition detection
 * @param p_context pointer to the global flight phase global context
 * @param p_sensor_data pointer to the current sensor data
 * @param p_sensor_event pointer to any generated sensor event
 * @return the status of the function
 */
w_status_t flight_phase_sensor_detection(flight_phase_ctx_t *p_context,
										 const all_sensors_data_t *p_sensor_data,
										 flight_phase_event_t *p_sensor_event);

#endif
