#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#include <stdint.h>

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
    EVENT_ESTIMATOR_INIT,
    EVENT_INJ_OPEN,
    EVENT_ACT_DELAY_ELAPSED,
    EVENT_FLIGHT_ELAPSED,
    EVENT_RESET
} flight_phase_event_t;

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

#endif