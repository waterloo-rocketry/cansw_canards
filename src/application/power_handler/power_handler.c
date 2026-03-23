#include "power_handler.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "rocketlib/include/common.h"

#include "message_types.h"
#include "application/can_handler/can_handler.h"


/**
 * Thresholds that currently have random numbers. Used in power_handler_get_status.
 */
// TO DO: Get actual threshold values from someone in Rocketry
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

// actuator ids from https://docs.waterloorocketry.com/avionics/rocketcan/packet-format.html#actuator-id 
/**
 * CANARD_5V_OUTPUT - Enable Canard Board’s 5V output to payload
 * CANARD_LIPO_ON - Enable Canard board draw power from LiPo
 * 5V_RAIL_ROCKET 
 * 12V_RAIL_ROCKET
 */

 // analog sensor ids
 /**
  * 5V_VOLT - Voltage of 5V rail in mV
  * 5V_CURR - Current of 5V rail in mA
  * 12V_VOLT - Voltage of 12V rail in mV
  * 12V_CURR - Current of 12V rail in mA
  * CHARGE_VOLT - LiPo charging voltage in mV
  * CHARGE_CURR - LiPo charging current in mA
  * BATT_VOLT - Battery Voltage in mV
  * BATT_CURR - Battery Current in mA
  */

#define TASK_DELAY_MS 3000 // dunno 

/**
 * States of the power handler.
 */
typedef struct {
    bool initialized;
    bool payload_enabled;
    bool low_power_mode;
    bool fault_bat1;
    bool fault_bat2;
    uint32_t overcurrent_count;
} power_handler_status_t;

static power_handler_status_t power_handler_status = {0};

/**
 * Handles incoming actuator CAN commands for 5V external and low power mode.
 */ 
static w_status_t power_actuator_callback(const can_msg_t *msg) {
    // TO DO: Uncomment the code once 5V external and low power mode commands are merged to main:
    // if (get_actuator_id(msg) == CANARD_5V_OUTPUT && get_cmd_actuator_state(msg) == ACT_STATE_ON) {
    //     return power_handler_set_payload_power(true);
    // } else if (get_actuator_id(msg) == CANARD_5V_OUTPUT && get_cmd_actuator_state(msg) == ACT_STATE_OFF) {
    //     return power_handler_set_payload_power(false);
    // } else if (get_actuator_id(msg) == CANARD_LIPO_ON && get_cmd_actuator_state(msg) == ACT_STATE_ON) {
    //     // TODO: enable LiPo via GPIO
    // } else if (get_actuator_id(msg) == CANARD_LIPO_ON && get_cmd_actuator_state(msg) == ACT_STATE_OFF) {
    //     // TODO: disable LiPo via GPIO
    // }
    return W_SUCCESS;
}

/**
 * Initializes power handler.
 * Registers CAN callbacks for payload 5V and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void) {
    w_status_t init_status = W_SUCCESS;

    init_status |= can_handler_register_callback(MSG_ACTUATOR_CMD, power_actuator_callback);

    power_handler_status.initialized = true;
    power_handler_status.payload_enabled = true;
    power_handler_status.low_power_mode = false;

    return init_status;
}

/**
 * Returns uint32_t bitfield of active faults.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void) {
	uint32_t status_bitfield = 0;
    // TO DO: read BAT_FLT1, BAT_FLT2 GPIO pins
    // TO DO: read ADC rails and check thresholds


	// w_status_t adc_get_value(adc_channel_t channel, uint32_t *output, uint32_t timeout_ms);

	uint32_t voltage = W_SUCCESS;
	uint32_t adc_value = 0; // this is the value we get from adc, passed by reference into adc.h to get whatever we need 
	voltage |= adc_get_value(VSENS_BAT1, &adc_value, TASK_DELAY_MS);
	if (voltage != VSENS_BAT1) {
		return voltage; 
	}

    return status_bitfield;

}

/**
 * Toggles 5V external power rail via a GPIO pin.
 */
w_status_t power_handler_set_5V_external(bool enabled) {
	// TO DO: check if on CHG line, if so block enabling payload (1.5A limit)
    // TO DO: gpio_write(GPIO_PIN_EN_EXT_5V, enabled ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW, 5);
    power_handler_status.payload_enabled = enabled;
    return W_SUCCESS;

}

/**
 * Toggles low power mode via GPIO pin.
 */
w_status_t power_handler_set_low_power_mode(bool enabled) {
    power_handler_status.low_power_mode = enabled; 
    return W_SUCCESS;
}