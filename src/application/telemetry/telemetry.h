#include "application/fsm/fsm.h" // fsm_state_t
#include "application/health_checks/health_checks.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// pass in any fuction your module desire to log data
typedef void (*telemetry_log_fn_t)(void);

// struct for telemetry source configuration
typedef struct {
	const char *name; // for status/debug logging

	telemetry_log_fn_t log_fn; // function to call periodically to log data
	uint32_t desired_frequency_hz;
	fsm_state_t flight_phase_state;

	uint32_t period_ms; // the period in ms between logs, derived from desired_frequency_hz
	uint32_t last_logged_ms; // the last ms timestamp when this source was logged
	uint32_t due_date_ms; // the next ms timestamp when this source should be logged
} telemetry_source_config_t;

// initialize the telemetry module
w_status_t telemetry_init(void);

// register function
w_status_t telemetry_register(const telemetry_source_config_t *config);

// rtos task
void telemetry_task(void *argument);

// for health checks
health_status_t telemetry_get_status(void);
