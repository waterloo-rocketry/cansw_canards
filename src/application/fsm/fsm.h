#ifndef FSM_H
#define FSM_H

#include <stdint.h>

#include "rocketlib/include/common.h"

/**
 * Enum representing phase of flight (state machine state)
 */
typedef enum : uint8_t { // make sure copying this data type is atomic
	STATE_IDLE,
	STATE_PAD_FILTER,
	STATE_PAD_NAV,
	STATE_BOOST,
	STATE_ACT_ALLOWED,
	STATE_RECOVERY,
	STATE_SLEEPY,
	STATE_ERROR
} fsm_state_t;

/**
 * @brief init fsm
 */
w_status_t fsm_init();

/**
 * @brief get current fsm state
 * @return the current fsm state
 */
fsm_state_t fsm_get_state();

/**
 * run in 500 hz freertos task
 */
void fsm_task(void *args);

#endif
