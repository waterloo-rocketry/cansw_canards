#include "application/fsm/fsm.h"   // fsm_state_t
#include "rocketlib/include/common.h"
#include "application/health_checks/health_checks.h"
#include <stdbool.h>
#include <stdint.h>

typedef void (*telemetry_log_fn_t)(void);

typedef struct {
    const char *name;                          // for status/debug logging

    telemetry_log_fn_t log_fn;                 // fills container + calls log_data()
    uint32_t desired_frequency_hz;
    fsm_state_t flight_phase_state;

    uint32_t period_ms;
    uint32_t due_date_ms; // the rtos tickcount that the callback should get called next
} telemetry_source_config_t;

w_status_t telemetry_init(void);
// register function 
w_status_t telemetry_transmit_hz(const telemetry_source_config_t *config);
// rtos task
void telemetry_task(void *argument);
// for health checks
health_status_t telemetry_get_status(void);
