#include "power_handler.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "application/flight_phase/flight_phase.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"

/**
 * Initializes power handler.
 */
w_status_t power_handler_init(void) {
    return W_SUCCESS; 
}

/**
 * Main task that handles flight phase changes, detects faults and 
 * reports health statuses. 
 */
void power_handler_task(void *args) {
    return;
}

/**
 * Provides power handler status. 
 */
uint32_t power_handler_get_status(void) {
    return 0;
}

/**
 * Toggles 5V payload rail.
 */
w_status_t power_handler_set_payload_power(bool enabled) {
    return W_SUCCESS;
}