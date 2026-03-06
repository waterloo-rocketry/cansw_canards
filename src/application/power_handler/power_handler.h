#ifndef POWER_HANDLER_H
#define POWER_HANDLER_H

#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Initializes power handler.
 * Registers CAN callbacks to get low power mode and payload power commands from can handler. 
 */
w_status_t power_handler_init(void);

/**
 * Main task that responds to flight phase changes, handles voltage/current telemetry
 * and checks that thresholds are respected. 
 */
void power_handler_task(void *args);

/**
 * Provides power handler status and returns faults.
 */
uint32_t power_handler_get_status(void);

/**
 * Toggles 5V payload rail.
 */
w_status_t power_handler_set_payload_power(bool enabled);

#endif