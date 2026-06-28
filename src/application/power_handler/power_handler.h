#ifndef POWER_HANDLER_H
#define POWER_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "rocketlib/include/common.h"

/**
 * Initializes power handler.
 * Registers CAN callbacks for 5V external power and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void);

/**
 * Returns uint32_t bitfield of active faults.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void);

#endif // POWER_HANDLER_H
