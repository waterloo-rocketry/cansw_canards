#include "power_handler.h"
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "message_types.h"
#include "queue.h"
#include "rocketlib/include/common.h"

/**
 * Handles incoming actuator CAN commands for 5V external and low power mode
 */
static w_status_t power_actuator_callback(const can_msg_t *msg) {
	return W_SUCCESS;
}

/**
 * Initializes power handler.
 * Registers CAN callbacks for 5V external and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void) {
	(void)power_actuator_callback;
	return W_SUCCESS;
}

/**
 * Returns uint32_t bitfield of active faults.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void) {
	return 0;
}

/**
 * Toggles 5V external power rail via a GPIO pin.
 */
w_status_t power_handler_set_5V_external(bool enabled) {
	return W_SUCCESS;
}

/**
 * Toggles low power mode via GPIO pin.
 */
w_status_t power_handler_set_low_power_mode(bool enabled) {
	return W_SUCCESS;
}
