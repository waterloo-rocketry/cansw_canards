#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "application/fsm/fsm.h"
#include "common/gnc/gnc_types.h"
#include "rocketlib/include/common.h"

/**
 * Enum representing a state transition event
 */
typedef enum {
	EVENT_NONE,
	EVENT_PAD_FILTER,
	EVENT_IGNITOR,
	EVENT_LAUNCH_ACCEL,
	EVENT_INJ_OPEN,
	EVENT_ACT_DELAY_ELAPSED,
	EVENT_RECOVERY_START, // This is event recovery log rate timer event
	EVENT_SLEEP_START // get into sleep rate
} flight_phase_event_t;

typedef struct {
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
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event);

/**
 * @brief get the next event that is waiting to be consumed. Does not block (no waiting)
 * @return return the next event or return EVENT_NONE if no events are waiting
 */
flight_phase_event_t flight_phase_get_next_event(void);

/**
 * process 1 transition.
 */
fsm_state_t flight_phase_update_state(flight_phase_event_t event, fsm_state_t curr_state,
									  flight_phase_ctx_t *p_ctx);

/**
 * @brief Reports the current status of the flight phase module
 * @return CAN board status bitfield
 * @details Logs initialization status, state machine state, event statistics,
 * and error conditions for the flight phase state machine
 */
uint32_t flight_phase_get_status(void);

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
										const all_sensors_data_t *p_sensor_data);

#endif
