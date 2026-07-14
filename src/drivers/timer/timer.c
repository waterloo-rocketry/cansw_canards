/**
 * Timer module
 * Driver for tracking system time
 * (currently configured to tick at 10,000 Hz or 0.1ms/tick)
 */

#include "drivers/timer/timer.h"
#include "application/logger/log.h"
#include "stm32h7xx_hal.h"

// external timer handle declaration
extern TIM_HandleTypeDef htim2;

/**
 * @brief Timer module health stats
 */
typedef struct {
	bool is_init; /**< Whether the timer module has been initialized */
	uint32_t timer_start_fails; /**< Count of HAL_TIM_Base_Start failures */
	uint32_t valid_calls; /**< Count of successful calls to timer_get_ms */
	uint32_t invalid_param; /**< Count of calls with NULL parameter */
	uint32_t timer_invalid_handle; /**< Count of calls where timer handle was invalid */
	uint32_t timer_uninitialized; /**< Count of calls while timer was uninitialized */
} timer_health_t;

// Error tracking
static timer_health_t timer_health = {0};

/**
 * @brief initialize the HAL TIM for the timer
 * @details Use TIM CHANNEL 2. Without initialization any timer call will be invalid
 * @return the status of the initialization
 */
w_status_t timer_init() {
	// can only init once
	if (timer_health.is_init) {
		return W_SUCCESS;
	}

	if (HAL_OK != HAL_TIM_Base_Start(&htim2)) {
		timer_health.timer_start_fails++;
		return W_FAILURE;
	}

	timer_health.is_init = true;
	return W_SUCCESS;
}

/**
 * @brief tracks system time since program startup
 * @details retrieves time passed in the form of clock ticks, timer resolution set to 0.1ms (10000Hz
 * frequency), however rounded to 1 ms resolution
 * @param p_ms the pointer to the time (ms)
 * @return status of function call
 */
w_status_t timer_get_ms(uint32_t *p_ms) {
	// check if there exists a valid location to store the time
	if (p_ms == NULL) {
		timer_health.invalid_param++;
		return W_INVALID_PARAM;
	}
	// check the timer handle pointer to ensure it is valid
	if (htim2.Instance == NULL) {
		timer_health.timer_invalid_handle++;
		return W_FAILURE;
	}
	// check initialization
	if (!timer_health.is_init) {
		timer_health.timer_uninitialized++;
		return W_FAILURE;
	}

	// retrieve the current timer count (in clock ticks)
	uint32_t timer_count = __HAL_TIM_GET_COUNTER(&htim2);

	// convert the timer count to milliseconds
	// each tick is 0.1 ms, so we divide by 10 to truncate to ms accuracy
	*p_ms = (uint32_t)(timer_count / 10);

	timer_health.valid_calls++;
	return W_SUCCESS;
}

/**
 * @brief tracks system time since program startup
 * @details retrieves time passed in the form of clock ticks, timer resolution set to 0.1ms (10000Hz
 * frequency)
 * @param p_time the pointer to the time (tenth of a ms)
 * @return status of function call
 */
w_status_t timer_get_tenth_ms(uint32_t *p_time) {
	// check if there exists a valid location to store the time
	if (p_time == NULL) {
		timer_health.invalid_param++;
		return W_INVALID_PARAM;
	}
	// check the timer handle pointer to ensure it is valid
	if (htim2.Instance == NULL) {
		timer_health.timer_invalid_handle++;
		return W_FAILURE;
	}
	// check initialization
	if (!timer_health.is_init) {
		timer_health.timer_uninitialized++;
		return W_FAILURE;
	}

	// retrieve the current timer count (in clock ticks)
	uint32_t timer_count = __HAL_TIM_GET_COUNTER(&htim2);

	// convert the timer count to milliseconds
	// each tick is 0.1 ms, so we just keep the same value for a tenth of ms accuracy
	*p_time = timer_count;

	timer_health.valid_calls++;
	return W_SUCCESS;
}

health_status_t timer_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_TIMER, .error_bitfield = 0};

	// Calculate total calls
	uint32_t total_calls = timer_health.valid_calls + timer_health.invalid_param +
						   timer_health.timer_invalid_handle + timer_health.timer_uninitialized;

	if (!timer_health.is_init) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_NOT_INIT;
	}

	// Log call statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "timer",
			 "Call statistics: total=%lu, successful=%lu, is_init=%lu",
			 total_calls,
			 timer_health.valid_calls,
			 timer_health.is_init);

	// Log error statistics if any errors occurred
uint32_t total_errors = timer_health.invalid_param + timer_health.timer_invalid_handle +
							timer_health.timer_uninitialized + timer_health.timer_start_fails;
	if (total_errors > 0) {
		log_text(0, LOG_LVL_WARN, "timer", "Total errors: %lu", total_errors);

		log_text(0,
				 LOG_LVL_WARN,
				 "timer",
				 "invalid_param=%lu, invalid_handle=%lu, uninitialized=%lu, timer_start_fails=%lu",
				 timer_health.invalid_param,
				 timer_health.timer_invalid_handle,
				 timer_health.timer_uninitialized,
				 timer_health.timer_start_fails);
	}

	return status;
}
