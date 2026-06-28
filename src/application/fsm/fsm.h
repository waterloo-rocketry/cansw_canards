#ifndef FSM_H
#define FSM_H

#include "GNC_codegen_types.h"
#include "rocketlib/include/common.h"

/**
 * Enum representing phase of flight (state machine state)
 */
typedef enum {
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
 * @param codegen_stack_data the pointer to our global codegen data
 */
w_status_t fsm_init(GNC_codegenStackData *codegen_stack_data);

/**
 * run in 500 hz freertos task
 */
void fsm_task(void *args);

#endif
