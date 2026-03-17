#include "power_handler.h"
#include "FreeRTOS.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "rocketlib/include/common.h"

/**
 * Thresholds that currently have random numbers. Used in power_handler_get_status.
 */
#define BAT_FLT1 1
#define BAT_FLT2 1
#define VSENS_BAT1 10 
#define VSENS_BAT2 11
#define VSENS_RKT 12
#define VSENS_CHG 13
#define VSENS_USB 3 
#define ISENS_BAT1 13
#define ISENS_BAT2 12
#define ISENS_3V3 12
#define ISENS_5V 11

/**
 * Initializes power handler.
 * Registers CAN callbacks for payload 5V and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void) {
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
 * Toggles 5V payloads power rail via a GPIO pin.
 */
w_status_t power_handler_set_payload_power(bool enabled) {
	return W_SUCCESS;
}

/**
 * Reads messages from RocketCAN that toggles payload power and
 * calls power_handler_set_payload_power.
 */
static w_status_t payload_power_callback(const can_msg_t *msg) {
	(void)msg;
	return W_SUCCESS;
}

/**
 * Reads messages from RocketCAN that toggles low power mode and
 * turns off canard board components via GPIO pins.
 */
static w_status_t low_power_callback(const can_msg_t *msg) {
	(void)msg;
	return W_SUCCESS;
}