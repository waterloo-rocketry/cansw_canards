#ifndef POWER_HANDLER_H
#define POWER_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

#include "application/can_handler/can_handler.h"
#include "application/health_checks/health_checks.h"
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
 * Reports power-handler MODULE health.
 * The returned error_bitfield uses can_canards_module_error_bitfield_offset_t (CANARDS_MODULE_E_*)
 * and is carried by MSG_CANARD_FIRMWARE_ERROR. Reports module/software faults only (battery fault
 * pins, external 5V device fault, GPIO/ADC failures, illegal low-power state, not-init, CAN TX).
 * Also transmits the periodic voltage/current status CAN messages.
 * Called by health checks.
 */
health_status_t power_handler_get_status(void);

/**
 * Reports power-handler BOARD electrical health.
 * The returned bitfield uses can_board_error_bitfield_offset_t (E_*_OFFSET) and is carried by
 * MSG_GENERAL_BOARD_STATUS. Reports electrical rail faults only (5V/local rail overcurrent, ext 5V
 * efuse fault, charge/rocket/battery voltage and current out of range). Returns 0 if no faults.
 * Called by health checks.
 */
uint32_t power_handler_get_board_status(void);

#endif // POWER_HANDLER_H
