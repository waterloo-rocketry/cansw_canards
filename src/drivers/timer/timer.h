#ifndef TIMER_H
#define TIMER_H

#include "rocketlib/include/common.h"
#include <stdint.h>

/**
 * @brief Timer module health stats
 */
typedef struct {
	uint32_t valid_calls; /**< Count of successful calls to timer_get_ms */
	uint32_t invalid_param; /**< Count of calls with NULL parameter */
	uint32_t timer_stopped; /**< Count of calls when timer was stopped */
	uint32_t timer_invalid; /**< Count of calls when timer was invalid */
} timer_health_t;

// tracks system time since program startup
// retrieves time passed in the form of clock ticks
// timer resolution set to 0.1ms (10000Hz frequency)
w_status_t timer_get_ms(float *ms);

/**
 * @brief Report timer module health status
 *
 * Retrieves and reports timer error statistics through log messages.
 *
 * @return CAN board specific err bitfield
 */
uint32_t timer_get_status(void);

#endif
