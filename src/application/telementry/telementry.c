#include "telementry.h"
#include "FreeRTOS.h"

#define TELEMETRY_MAX_SOURCES   32 // TODO: find out real value for this

// returns true if a source at freq_hz should fire now
static bool telemetry_is_due(uint16_t freq_hz, uint32_t now_ms, uint32_t last_ms) {
    return true;
}

w_status_t telemetry_init(void){
    return W_SUCCESS;
}

w_status_t telemetry_register(const telemetry_source_config_t *config){
    return W_SUCCESS;
}

health_status_t telemetry_get_status(void){
    health_status_t status;
    status.error_bitfield = 0;
    status.module_id = MODULE_TELEMENTRY;
    status.severity = HEALTH_OK;

    return status;
}

void telemetry_task(void *argument){
    while(1){
        // run as fast as possible - 1000hz
    }
    return;
}