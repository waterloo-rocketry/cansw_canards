#include "telemetry.h"
#include "FreeRTOS.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "task.h"

#define TELEMETRY_MAX_SOURCES 100 // TODO: find out real value for this
#define TELEMETRY_TASK_PERIOD_MS 1 // 1000hz

// struct to keep track of due dates and last logged time
typedef struct {
	telemetry_source_config_t source_config;
	uint32_t
		last_logged_ms; // the last ms timestamp when this source was logged (for health checks)
	uint32_t due_date_ms; // the next ms timestamp when this source should be logged
} telemetry_internal_config_t;

// struct to keep track of registered telemetry sources
typedef struct {
	uint32_t num_sources; // number of sources currently registered
	telemetry_internal_config_t
		sources[TELEMETRY_MAX_SOURCES]; // array of registered telemetry sources
} telemetry_registry_t;

// struct to keep track of telemetry statistics
typedef struct {
	bool initialized;
	uint16_t failed_transmissions;
	uint16_t successful_transmissions;
	uint16_t overdue_count;
} telemetry_stats_t;

// global struct to track registry and stats (static storage is zero-initialized)
static telemetry_registry_t g_telemetry_registry = {.num_sources = 0};
static telemetry_stats_t g_telemetry_stats = {.initialized = false};

/**
 * @brief clears registry + stats, including the initialized flag
 */
void telemetry_clear_all_data(void) {
	// zero init telemetry stats
	g_telemetry_stats.initialized = false;
	g_telemetry_stats.failed_transmissions = 0;
	g_telemetry_stats.successful_transmissions = 0;
	g_telemetry_stats.overdue_count = 0;

	// zero init telemetry registry source array
	for (uint32_t i = 0; i < TELEMETRY_MAX_SOURCES; i++) {
		g_telemetry_registry.sources[i].source_config.name = NULL;
		g_telemetry_registry.sources[i].source_config.log_fn = NULL;
		g_telemetry_registry.sources[i].source_config.flight_phase_state = STATE_ERROR;
		g_telemetry_registry.sources[i].source_config.period_ms = 0;
		g_telemetry_registry.sources[i].last_logged_ms = 0;
		g_telemetry_registry.sources[i].due_date_ms = 0;
	}

	g_telemetry_registry.num_sources = 0;
}

/**
 * @brief seed the due date of every registered source relative to curr_time
 * @param curr_time current time in ms (start of telemetry task)
 */
void telemetry_init_due_dates(uint32_t curr_time) {
	// set due date to every registered function once telemetry starts
	for (uint32_t i = 0; i < g_telemetry_registry.num_sources; i++) {
		telemetry_internal_config_t *curr = &g_telemetry_registry.sources[i];
		curr->due_date_ms =
			curr->source_config.period_ms + curr_time; // first log is due one period after t=0
	}
}

/**
 * @brief initialize telemetry module
 * @return status of init - double init returns failure
 */
w_status_t telemetry_init(void) {
	if (g_telemetry_stats.initialized) {
		log_text(1, LOG_LVL_WARN, "telemetry module", "Telemetry module already initialized");
		return W_FAILURE;
	}

	// clears registry + stats
	telemetry_clear_all_data();

	// successful init
	g_telemetry_stats.initialized = true;

	return W_SUCCESS;
}

/**
 * @brief run a single telemetry scheduling iteration
 * @note logs each registered source whose flight phase matches the current fsm
 * state and whose due date has passed, updating stats. Excludes the RTOS delay. For unit testing!
 */
void telemetry_run_once(void) {
	fsm_state_t phase = fsm_get_state();

	uint32_t curr_time;
	w_status_t time_status = timer_get_ms(&curr_time);

	// loop through the array of registered sources and log if it's the correct flight phase
	// and it's due/overdue
	if (W_SUCCESS == time_status) {
		for (uint32_t i = 0; i < g_telemetry_registry.num_sources; i++) {
			telemetry_internal_config_t *source = &g_telemetry_registry.sources[i];

			if (phase != source->source_config.flight_phase_state) {
				continue;
			}

			// signed-difference compare so this survives the 32-bit ms counter wrapping around
			int32_t overdue_by = (int32_t)(curr_time - source->due_date_ms);

			// overdue euqal to 0 not handled
			if (overdue_by < 0) {
				continue; // not due yet
			} else if (overdue_by > 0) {
				// TODO: report to health check with module name
				g_telemetry_stats.overdue_count++;
			}

			// call the function and check execution status
			if (source->source_config.log_fn() == W_SUCCESS) {
				g_telemetry_stats.successful_transmissions++;
			} else {
				g_telemetry_stats.failed_transmissions++;
			}

			source->last_logged_ms = curr_time;
			source->due_date_ms = curr_time + source->source_config.period_ms;
		}
	}
}

/**
 * @brief telem task function
 */
void telemetry_task(void *argument) {
	(void)argument;
	TickType_t lastWakeTime = xTaskGetTickCount();
	uint32_t curr_time = 0;
	w_status_t time_status = timer_get_ms(&curr_time);

	if (time_status != W_SUCCESS) {
		log_text(1,
				 LOG_LVL_WARN,
				 "telemetry module",
				 "Failed to get current time for updating registered function due dates! Using 0 "
				 "as curr time.");
	}

	telemetry_init_due_dates(curr_time);

	while (1) {
		telemetry_run_once(); // abstracted for unit testing

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS));
	}
}

/**
 * @brief register config handler with telemetry module
 * @return status of registering
 */
w_status_t telemetry_register(const telemetry_source_config_t *config) {
	if (!g_telemetry_stats.initialized) {
		log_text(1, LOG_LVL_WARN, "telemetry module", "Telemetry module not initialized");
		return W_FAILURE;
	}

	if (g_telemetry_registry.num_sources >= TELEMETRY_MAX_SOURCES) {
		log_text(
			1, LOG_LVL_WARN, "telemetry module", "Maximum number of telemetry sources reached");
		return W_FAILURE;
	}

	if ((config->name == NULL) || (config->log_fn == NULL) || (config->period_ms == 0)) {
		log_text(1, LOG_LVL_WARN, "telemetry module", "Invalid telemetry source configuration");
		return W_INVALID_PARAM;
	}

	// Copy the caller's config into the registry slot, then initialize the internally-managed
	// scheduling fields (avoids mutating the caller's const struct).
	telemetry_internal_config_t *slot =
		&g_telemetry_registry.sources[g_telemetry_registry.num_sources];
	slot->source_config = *config;
	slot->last_logged_ms = 0;
	slot->due_date_ms = 0;

	g_telemetry_registry.num_sources++;

	return W_SUCCESS;
}

/**
 * @brief health check getter for telemtry module
 * @return health check handle for telem error
 */
health_status_t telemetry_get_status(void) {
	// TODO: make health check work
	health_status_t status = {
		.error_bitfield = 0, .module_id = MODULE_TELEMETRY, .severity = HEALTH_OK};

	if (g_telemetry_stats.failed_transmissions > 0 || g_telemetry_stats.overdue_count > 0) {
		status.severity = HEALTH_ERROR;
		// status.error_bitfield = (1) << MODULE_ERR_TELEMETRY_NOT_INIT;
		//  status.error_bitfield = (1) << MODULE_ERR_TELEMETRY_FAILED_TX;
	}

	return status;
}
