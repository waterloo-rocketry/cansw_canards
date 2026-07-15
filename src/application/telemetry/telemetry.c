#include "telemetry.h"
#include "FreeRTOS.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "task.h"

#define TELEMETRY_MAX_SOURCES 100 // TODO: find out real value for this
#define TELEMETRY_PER_MODULE_MAX_SOURCE 20 // TODO: find out real value for this
#define TELEMETRY_TASK_PERIOD_MS 1 // 1000hz

// struct to keep track of due dates and last logged time
typedef struct {
	telemetry_source_config_t public_config_handle;
	uint32_t
		last_logged_ms; // the last ms timestamp when this source was logged (for health checks)
	uint32_t due_date_ms; // the next ms timestamp when this source should be logged
} telemetry_timestamped_config_t;

// struct to keep track of registered telemetry sources
typedef struct {
	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_idle[TELEMETRY_MAX_SOURCES];
	} state_idle;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_pad_filter[TELEMETRY_MAX_SOURCES];
	} state_pad_filter;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_pad_nav[TELEMETRY_MAX_SOURCES];
	} state_pad_nav;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_boost[TELEMETRY_MAX_SOURCES];
	} state_boost;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_act_allowed[TELEMETRY_MAX_SOURCES];
	} state_act_allowed;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_recovery[TELEMETRY_MAX_SOURCES];
	} state_recovery;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_sleepy[TELEMETRY_MAX_SOURCES];
	} state_sleepy;

	struct {
		uint32_t num_sources;
		telemetry_timestamped_config_t
			registered_state_error[TELEMETRY_MAX_SOURCES];
	} state_error;

	telemetry_user_stats_t user_register_stats[MODULE_MAX];
} telemetry_registry_t;

// struct to keep track of telemetry statistics
typedef struct {
	bool initialized;
	uint16_t failed_transmissions;
	uint16_t successful_transmissions;
	uint16_t overdue_count;
} telemetry_stats_t;

 // for tracking - making sure one module does not register too much function calls
typedef struct {
	telemetry_module_name_t name;
	uint32_t number_registered;
} telemetry_user_stats_t;

// global struct to track registry and stats (static storage is zero-initialized)
static telemetry_registry_t g_telemetry_registry;
static telemetry_stats_t g_telemetry_stats = {.initialized = false};

/**
 * @brief clears registry + stats
 */
static void telemetry_clear_all_data(void) {

	// zero init telemetry stats
	g_telemetry_stats.failed_transmissions = 0;
	g_telemetry_stats.successful_transmissions = 0;
	g_telemetry_stats.overdue_count = 0;

	// zero init telemetry registry source array
	for (uint32_t i = 0; i < TELEMETRY_MAX_SOURCES; i++) {
		g_telemetry_registry.sources[i].name = NULL;
		g_telemetry_registry.sources[i].log_fn = NULL;
		g_telemetry_registry.sources[i].flight_phase_state = STATE_ERROR;
		g_telemetry_registry.sources[i].period_ms = 0;
	}

	g_telemetry_registry.num_sources = 0;
}

/**
 * @brief seed the due date of every registered source relative to curr_time
 * @param curr_time current time in ms (start of telemetry task)
 */
static void telemetry_init_due_dates(uint32_t curr_time) {
	// set due date to every registered function once telemetry starts
	for (uint32_t i = 0; i < g_telemetry_registry.num_sources; i++) {
		telemetry_source_config_t *curr = &g_telemetry_registry.sources[i];
		curr->due_date_ms = curr->period_ms + curr_time; // first log is due one period after t=0
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
			telemetry_source_config_t *source = &g_telemetry_registry.sources[i];

			if (phase != source->flight_phase_state) {
				continue;
			}

			// signed-difference compare so this survives the 32-bit ms counter wrapping around
			int32_t overdue_by = (int32_t)(curr_time - source->due_date_ms);
			if (overdue_by < 0) {
				continue; // not due yet
			}

			if (overdue_by > 0) {
				// TODO: report to health check with module name
				g_telemetry_stats.overdue_count++;
			}

			// call the function and check execution status
			if (W_SUCCESS == source->log_fn()) {
				g_telemetry_stats.successful_transmissions++;
			} else {
				g_telemetry_stats.failed_transmissions++;
			}

			source->last_logged_ms = curr_time;
			source->due_date_ms = curr_time + source->period_ms;
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
	
	if (W_SUCCESS != time_status){
		log_text(1, LOG_LVL_WARN, "telemetry module", "Failed to get current time for updating registered function due dates! Using 0 as curr time.");
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

	if ((config->name == NULL) || (config->log_fn == NULL)|| (config->period_ms == 0)) {
	log_text(1, LOG_LVL_WARN, "telemetry module", "Invalid telemetry source configuration");
	return W_INVALID_PARAM;
	}

	if ((config->last_logged_ms != 0) || (config->due_date_ms != 0)) {
	log_text(
		1,
		LOG_LVL_WARN,
		"telemetry module",
		"Wrong init with the config handler, should init last_logged_ms and due_date_ms to "
		"zero.");
	}

	// Copy into the registry slot, then initialize the internally-managed scheduling fields on the copy (avoids mutating the caller's const struct).
	telemetry_source_config_t *slot =
	&g_telemetry_registry.sources[g_telemetry_registry.num_sources];
	*slot = *config;
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
	health_status_t status = {
		.error_bitfield = 0, .module_id = MODULE_TELEMETRY, .severity = HEALTH_OK};

	if (g_telemetry_stats.failed_transmissions > 0 || g_telemetry_stats.overdue_count > 0) {
		status.severity = HEALTH_ERROR;
		// status.error_bitfield = (1) << MODULE_ERR_TELEMETRY_NOT_INIT;
		//  status.error_bitfield = (1) << MODULE_ERR_TELEMETRY_FAILED_TX;
	}

	return status;
}
