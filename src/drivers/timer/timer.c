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

// Error tracking
static timer_health_t timer_health = {0};

// make sure to report error if not initialized
static bool g_timer_initialized = false;

/**
 * @brief initialize the HAL TIM for the timer
 * @details Use TIM CHANNEL 2. Without initialization any timer call will be invalid
 * @return the status of the initialization
 */
w_status_t timer_init() {
	// can only init once
	if (g_timer_initialized) {
		return W_SUCCESS;
	}

	if (HAL_OK != HAL_TIM_Base_Start(&htim2)) {
		return W_FAILURE;
	}

	g_timer_initialized = true;
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
		timer_health.timer_invalid++;
		return W_FAILURE;
	}
	// check initialization
	if (!g_timer_initialized) {
		timer_health.timer_invalid++;
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
		timer_health.timer_invalid++;
		return W_FAILURE;
	}
	// check initialization
	if (!g_timer_initialized) {
		timer_health.timer_invalid++;
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
	uint32_t status_bitfield = 0;

	// Calculate total calls
	uint32_t total_calls = timer_health.valid_calls + timer_health.invalid_param +
						   timer_health.timer_stopped + timer_health.timer_invalid;

	// Log call statistics
	log_text(0,
			 "timer",
			 "Call statistics: total=%lu, successful=%lu",
			 total_calls,
			 timer_health.valid_calls);

	// Log error statistics if any errors occurred
	uint32_t total_errors =
		timer_health.invalid_param + timer_health.timer_stopped + timer_health.timer_invalid;

	if (total_errors > 0) {
		log_text(0,
				 "timer",
				 "Error statistics: invalid_param=%lu, timer_stopped=%lu, timer_invalid=%lu",
				 timer_health.invalid_param,
				 timer_health.timer_stopped,
				 timer_health.timer_invalid);
	}

	health_status_t status = {.severity = HEALTH_OK, .module_id = MODULE_TIMER, .error_code = 0};

	return status;
}
