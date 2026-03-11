#ifndef POWER_HANDLER_H
#define POWER_HANDLER_H

#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Initializes power handler.
 * Registers CAN callbacks for payload 5V and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void);

/**
 * Main task. Periodically reads ADC rails and checks voltage/current
 * against expected thresholds. Sends telemetry via can_handler_transmit.
 */
void power_handler_task(void *args);

/**
 * Returns uint32_t bitfield of active faults.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void);

/**
 * Toggles 5V payloads power rail via a GPIO pin.
 */
w_status_t power_handler_set_payload_power(bool enabled);

#endif