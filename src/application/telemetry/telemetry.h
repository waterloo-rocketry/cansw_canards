#include <stdbool.h>
#include <stdint.h>

#include "application/health_checks/health_checks.h"
#include "rocketlib/include/common.h"

// Pass in any fuction your module desire to log data (had to be NON-BLOCKING and returns a status
// code)
typedef w_status_t (*telemetry_log_fn_t)(void);

// struct for telemetry source configuration
typedef struct {
	const char *name; // for status/debug logging

	telemetry_log_fn_t log_fn; // function to call periodically to log data
	fsm_state_t flight_phase_state; // the flight phase state in which this source should be logged

	uint32_t period_ms; // the period in ms between logs, derived from desired_frequency_hz
	uint32_t
		last_logged_ms; // the last ms timestamp when this source was logged (for health checks)
	uint32_t due_date_ms; // the next ms timestamp when this source should be logged
} telemetry_source_config_t;

/*
 * @brief Initialize the telemetry module
 * @return Status of initialization
 */
w_status_t telemetry_init(void);

/**
 * @brief Register a new telemetry source
 * @param config Pointer to the telemetry source configuration
 * @note Initializes the config with name, log_function, flight_phase_state, and period_ms with specified values. Initializes last_logged_ms and due_date_ms to 0.
 * @return Status of registration
 */
w_status_t telemetry_register(const telemetry_source_config_t *config);

/**
 * @brief RTOS task for handling telemetry logging
 */
void telemetry_task(void *argument);

/**
 * @brief Reports the current status of the telemetry module
 * @return telemetry status bitfield
 */
health_status_t telemetry_get_status(void);
