#ifndef TIMER_H
#define TIMER_H

#include "application/health_checks/health_checks.h"

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

w_status_t timer_init();

/**
 * @brief tracks system time since program startup
 * @details retrieves time passed in the form of clock ticks, timer resolution set to 0.1ms (10000Hz
 * frequency), however rounded to 1 ms resolution
 * @param p_ms the pointer to the time (ms)
 * @return status of function call
 */
w_status_t timer_get_ms(uint32_t *p_ms);

/**
 * @brief tracks system time since program startup
 * @details retrieves time passed in the form of clock ticks, timer resolution set to 0.1ms (10000Hz
 * frequency)
 * @param p_time the pointer to the time (tenth of a ms)
 * @return status of function call
 */
w_status_t timer_get_tenth_ms(uint32_t *p_time);

/**
 * @brief Report timer module health status
 *
 * Retrieves and reports timer error statistics through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t timer_get_status(void);

#endif
