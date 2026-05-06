#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "application/estimator/estimator_types.h"
#include "application/flight_phase/flight_phase_types.h"
#include "application/fsm/fsm_types.h"
#include "rocketlib/include/common.h"

typedef struct flight_phase_ctx_t {
	uint32_t launch_timestamp_ms;
	uint32_t act_allowed_timestamp_ms;
	uint8_t num_consec_detections;
} flight_phase_ctx_t;

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
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event);

/**
 * @brief get the newest event in the event queue
 * @param timeout_ms timeout time
 * @return return the newest event or return EVENT_NONE otherwise
 */
flight_phase_event_t flight_phase_get_queue_event(uint8_t timeout_ms);

/**
 * process 1 transition.
 */
w_status_t flight_phase_update_state(flight_phase_event_t event, fsm_state_t *p_state,
									 flight_phase_ctx_t *p_ctx);

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
 * @brief performs any timer based state transition detection
 * @param p_context is the global flight phase global context
 * @param p_state pointer to global fsm state
 * @param timestamp_ms is the current timestamp
 * @param p_timer_event is the pointer to any generated timer event
 * @return the status of the function
 */
w_status_t flight_phase_timer_detection(flight_phase_ctx_t *p_ctx, const fsm_state_t *p_state,
										const uint32_t timestamp_ms,
										flight_phase_event_t *p_timer_event);

/**
 * @brief performs any sensor based state transition detection
 * @param p_context pointer to the global flight phase global context
 * @param p_state pointer to global fsm state
 * @param p_sensor_data pointer to the current sensor data
 * @param p_sensor_event pointer to any generated sensor event
 * @return the status of the function
 */
w_status_t flight_phase_sensor_detection(flight_phase_ctx_t *p_ctx, const fsm_state_t *p_state,
										 const all_sensors_data_t *p_sensor_data,
										 flight_phase_event_t *p_sensor_event);

#endif
