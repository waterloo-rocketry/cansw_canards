#include "telemetry.h"
#include "FreeRTOS.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/logging/logging.h"

#define TELEMETRY_MAX_SOURCES 32 // TODO: find out real value for this
#define TELEMETRY_TASK_PERIOD_MS 1 // 1000hz

// struct to keep track of registered telemetry sources
typedef struct {
	uint32_t num_sources; // number of sources currently registered
	telemetry_source_config_t
		sources[TELEMETRY_MAX_SOURCES]; // array of registered telemetry sources
} telemetry_registry_t;

// struct to keep track of telemetry statistics
typedef struct {
	bool initialized;
	uint16_t failed_transmissions;
	uint16_t successful_transmissions;
} telemetry_stats_t;

// global struct to track registry and stats
static telemetry_registry_t g_telemetry_registry = {0};
static telemetry_stats_t g_telemetry_stats = {0};

static void 
w_status_t telemetry_init(void) {
	if(g_telemetry_stats.initialized) {
		log_text(1, LOG_LEVEL_WARN, "telemetry module", "Telemetry module already initialized");
		return W_FAILURE;
	}

	g_telemetry_stats.failed_transmissions = 0;
	g_telemetry_stats.successful_transmissions = 0;

	for(uint32_t i = 0; i < TELEMETRY_MAX_SOURCES; i++) {
		g_telemetry_registry.sources[i].name = NULL;
		g_telemetry_registry.sources[i].log_fn = NULL;
		g_telemetry_registry.sources[i].desired_frequency_hz = 0;
		g_telemetry_registry.sources[i].flight_phase_state = STATE_ERROR;
		g_telemetry_registry.sources[i].period_ms = 0;
		g_telemetry_registry.sources[i].last_logged_ms = 0;
		g_telemetry_registry.sources[i].due_date_ms = 0;
	}

	g_telemetry_registry.num_sources = 0;
	g_telemetry_stats.initialized = true;

	return W_SUCCESS;
}

/**
 * @brief register config handler with telemetry module
 * @return status of registering
 */
w_status_t telemetry_register(const telemetry_source_config_t *config) {
	if(!g_telemetry_stats.initialized) {
		log_text(1, LOG_LEVEL_ERROR, "telemetry module", "Telemetry module not initialized");
		return W_FAILURE;
	}
	
	if (g_telemetry_registry.num_sources >= TELEMETRY_MAX_SOURCES) {
		log_text(1, LOG_LEVEL_ERROR, "telemetry module", "Maximum number of telemetry sources reached");
		return W_FAILURE;
	}

	if(config->name == NULL || config->log_fn == NULL || config->desired_frequency_hz == 0) {
		log_text(1, LOG_LEVEL_ERROR, "telemetry module", "Invalid telemetry source configuration");
		return W_INVALID_PARAM;
	}

	config->period_ms = 1000 / config->desired_frequency_hz; // todo: add a static function to conv hz to ms

	g_telemetry_registry.sources[g_telemetry_registry.num_sources] = *config;
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

	return status;
}

/**
 * @brief telem task function
 */
void telemetry_task(void *argument) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS));
	}

	return;
}
