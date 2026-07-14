#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "application/fsm/fsm.h"
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

	uint32_t period_ms; // the period in ms between logs

	// The fields below are managed internally by the telemetry module. Callers should leave them
	// zero-initialized; telemetry_register() populates them.
	uint32_t
		last_logged_ms; // the last ms timestamp when this source was logged (for health checks)
	uint32_t due_date_ms; // the next ms timestamp when this source should be logged
} telemetry_source_config_t;

/**
 * @brief Initialize the telemetry module
 * @return Status of initialization
 */
w_status_t telemetry_init(void);

/**
 * @brief Register a new telemetry source
 * @param config Pointer to the telemetry source configuration
 * @note Caller supplies name, log_fn, flight_phase_state, and period_ms. The last_logged_ms and
 * due_date_ms fields are managed internally and should be left zero-initialized.
 * @note Must be called before telemetry_task starts running (call it in your init!!).
 * @return Status of registration
 */
w_status_t telemetry_register(const telemetry_source_config_t *config);

/**
 * @brief RTOS task for handling telemetry logging
 */
void telemetry_task(void *argument);

/**
 * @brief run telemtry once, calling all registered function due at call time
 */
void telemetry_run_once(void);

/**
 * @brief Reports the current status of the telemetry module
 * @return telemetry status bitfield
 */
health_status_t telemetry_get_status(void);

#endif // TELEMETRY_H
