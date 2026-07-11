#include "telemetry.h"
#include "FreeRTOS.h"
#include "application/fsm/fsm.h"

#define TELEMETRY_MAX_SOURCES 32 // TODO: find out real value for this
#define TELEMETRY_TASK_FREQUENCY_HZ 1000 // run telemetry task at 1000hz
#define TELEMETRY_TASK_PERIOD_MS (1 / TELEMETRY_TASK_FREQUENCY_HZ) * 1000 // period in ms for 1000hz

// struct to keep track of registered telemetry sources
typedef struct {
	telemetry_source_config_t
		sources[TELEMETRY_MAX_SOURCES]; // array of registered telemetry sources
	uint32_t num_sources;
} telemetry_registry_t;

typedef struct {
	uint16_t failed_transmissions;
	uint16_t successful_transmissions;
} telemetry_stats_t;

static telemetry_registry_t g_telemetry_registry = {0};

w_status_t telemetry_init(void) {
	return W_SUCCESS;
}

w_status_t telemetry_register(const telemetry_source_config_t *config) {
	return W_SUCCESS;
}

health_status_t telemetry_get_status(void) {
	health_status_t status = {
		.error_bitfield = 0, .module_id = MODULE_TELEMETRY, .severity = HEALTH_OK};

	return status;
}

void telemetry_task(void *argument) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		// todo: implement telemetry logging based on registered sources and their desired
		// frequencies
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS));
	}

	return;
}
