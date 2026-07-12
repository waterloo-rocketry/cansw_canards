#include "telemetry.h"
#include "FreeRTOS.h"
#include "application/fsm/fsm.h"

static const uint16_t TELEMETRY_MAX_SOURCES = 32; // TODO: find out real value for this
static const uint16_t TELEMETRY_TASK_PERIOD_MS = 1; // 1000hz

// struct to keep track of registered telemetry sources
typedef struct {
	telemetry_source_config_t
		sources[TELEMETRY_MAX_SOURCES]; // array of registered telemetry sources
	uint32_t num_sources; // number of sources currently registered
} telemetry_registry_t;

// struct to keep track of telemetry statistics
typedef struct {
	bool initialized;
	uint16_t failed_transmissions;
	uint16_t successful_transmissions;
} telemetry_stats_t;

static telemetry_registry_t g_telemetry_registry = {0};
static telemetry_stats_t g_telemetry_stats = {0};

/**
 * @brief Initialize the telemetry module
 * @return Status of initialization
 */
w_status_t telemetry_init(void) {
	g_telemetry_stats.initialized = true;
	return W_SUCCESS;
}

/*
 * @brief register config handler with telemetry module
 * @return status of registering
 */
w_status_t telemetry_register(const telemetry_source_config_t *config) {
	return W_SUCCESS;
}

/*
 * @brief health check getter for telemtry module
 * @return health check handle for telem error
 */
health_status_t telemetry_get_status(void) {
	// health_status_t status = {
	// 	.error_bitfield = 0, .module_id = MODULE_TELEMETRY, .severity = HEALTH_OK};

	// return status;
}

/*
 * @brief telem task function
 */
void telemetry_task(void *argument) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		// todo: implement telemetry logging based on registered sources and their desired
		// frequencies
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS));
	}

	return;
}
