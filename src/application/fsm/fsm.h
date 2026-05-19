#ifndef FSM_H
#define FSM_H

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
} fsm_state_t;

/**
 * @brief init fsm
 */
w_status_t fsm_init();

/**
 * run in 500 hz freertos task
 */
void fsm_task(void *args);

#endif
