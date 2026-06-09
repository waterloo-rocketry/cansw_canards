#ifndef POWER_HANDLER_H
#define POWER_HANDLER_H

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// Fault bit positions
#define FAULT_BAT1_VOLT (1 << 0)
#define FAULT_BAT2_VOLT (1 << 1)
#define FAULT_RKT_VOLT (1 << 2)
#define FAULT_CHG_VOLT (1 << 3)
#define FAULT_5V_CURR (1 << 4)
#define FAULT_5V_OUTPUT (1 << 5)
#define FAULT_3V3_CURR (1 << 6)
#define FAULT_BAT1_CURR (1 << 7)
#define FAULT_BAT2_CURR (1 << 8)

/**
 * States of the power handler.
 */
typedef struct {
	bool initialized;
	bool external_5v_enabled;
	bool low_power_mode;
	uint32_t lipo_1_fault_count;
	uint32_t lipo_2_fault_count;
	uint32_t overcurrent_count;
} power_handler_status_t;

typedef enum {
	POWER_INPUT_CHG,
	POWER_INPUT_RKT,
	POWER_INPUT_BAT,
	POWER_INPUT_NONE
} power_input_source_t;

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

/**
 * Toggles 5V external power rail via a GPIO pin.
 */
w_status_t power_handler_set_5V_external(bool enabled);

/**
 * Toggles low power mode via GPIO pin.
 */
w_status_t power_handler_set_low_power_mode(bool enabled);

#endif // POWER_HANDLER_H
