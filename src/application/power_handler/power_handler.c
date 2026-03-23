#include "power_handler.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "rocketlib/include/common.h"

#include "application/can_handler/can_handler.h"
#include "message_types.h"

// TODO: add thresholds for various components
#define VBAT_MIN 10 // lipos
#define VBAT_MAX 10
#define IBAT_MAX 10

#define VRKT_MIN 10 // Rocket
#define VRKT_MAX 10

#define VCHG_MIN 10 // CHG
#define VCHG_MAX 10
// #define ICHG_MAX 10

#define V5V_EXTERNAL_MIN 10 // to 5V external
#define V5V_EXTERNAL_MAX 10
#define I5V_EXTERNAL_MAX 10

//  BAT_FLT1 - gpio fault pins
//  BAT_FLT2

// TO DO: add adc channel numbers to adc.h (done but not committed)
//  VSENS_BAT1
//  VSENS_BAT2
//  VSENS_RKT
//  VSENS_CHG
//  VSENS_USB
//  ISENS_BAT1
//  ISENS_BAT2
//  ISENS_3V3
//  ISENS_5V

/**
 * Thresholds that currently have random numbers. Used in power_handler_get_status.
 */
// TO DO: Get actual threshold values from someone in Rocketry

// actuator ids from
// https://docs.waterloorocketry.com/avionics/rocketcan/packet-format.html#actuator-id
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
	bool external_5v_enabled;
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
	// TO DO: Uncomment the code once 5V external and low power mode commands are merged to main
	// TO DO: find out what pins to get information from in new board 
	// if (get_actuator_id(msg) == CANARD_5V_OUTPUT && get_cmd_actuator_state(msg) == ACT_STATE_ON)
	// {
	//     return power_handler_set_5V_external(true); // turn on 5V external
	// }
	// else if (get_actuator_id(msg) == CANARD_5V_OUTPUT && get_cmd_actuator_state(msg) ==
	// ACT_STATE_OFF) {
	//     return power_handler_set_5V_external(false); // turn off 5V external
	// }
	// else if (get_actuator_id(msg) == CANARD_LIPO_ON && get_cmd_actuator_state(msg) ==
	// ACT_STATE_ON) {
	//     // enable LiPo via GPIO
	// 	power_handler_status.low_power_mode = false; // turn off low power
	// 	return gpio_write(GPIO_PIN_LIPO, GPIO_LEVEL_HIGH, 5);
	// }
	// else if (get_actuator_id(msg) == CANARD_LIPO_ON && get_cmd_actuator_state(msg) ==
	// ACT_STATE_OFF) {
	//     // TODO: disable LiPo via GPIO
	// 	power_handler_status.low_power_mode = true; // turn on low power
	// 	return gpio_write(GPIO_PIN_LIPO, GPIO_LEVEL_LOW, 5);
	// }
	(void)msg;
	return W_SUCCESS;
}

/**
 * Initializes power handler.
 * Registers CAN callbacks for payload 5V and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void) {
	// TO DO: find out whether we toggle the entire power system from the new canard board hardware
	// gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_HIGH, 5);

	w_status_t init_status = W_SUCCESS;

	init_status |= can_handler_register_callback(MSG_ACTUATOR_CMD, power_actuator_callback);

	power_handler_status.initialized = true;
	power_handler_status.external_5v_enabled = true;
	power_handler_status.low_power_mode = false;

	return init_status;
}

/**
 * Returns uint32_t bitfield of active faults.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void) {
	uint32_t status_bitfield = 0;
	uint32_t adc_value = 0;

	// TO DO: find what pins to read from 
	// gpio_level_t flt1 = GPIO_LEVEL_HIGH;
	// gpio_level_t flt2 = GPIO_LEVEL_HIGH;

	// gpio_read(BAT_FLT1, &flt1, 5);
	// gpio_read(BAT_FLT2, &flt2, 5);

	// if (flt1 == GPIO_LEVEL_LOW) {
	// 	status_bitfield |= 1;
	// 	power_handler_status.fault_bat1 = true;
	// }
	// if (flt2 == GPIO_LEVEL_LOW) {
	// 	status_bitfield |= 1;
	// 	power_handler_status.fault_bat2 = true;
	// }

	if (adc_get_value(VSENS_BAT1, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value < VBAT_MIN || adc_value > VBAT_MAX) {
			status_bitfield |= 1; 
		}
	}

	// check BAT2 voltage
	if (adc_get_value(VSENS_BAT2, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value < VBAT_MIN || adc_value > VBAT_MAX) {
			status_bitfield |= 1;
		}
	}

	// check RKT voltage
	if (adc_get_value(VSENS_RKT, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value < VRKT_MIN || adc_value > VRKT_MAX) {
			status_bitfield |= 1;
		}
	}

	// check CHG voltage
	if (adc_get_value(VSENS_CHG, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value < VCHG_MIN || adc_value > VCHG_MAX) {
			status_bitfield |= 1;
		}
	}

	// check 5V external current
	if (adc_get_value(ISENS_5V, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value > I5V_EXTERNAL_MAX) {
			status_bitfield |= 1;
		}
	}

	// check BAT1 current
	if (adc_get_value(ISENS_BAT1, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value > IBAT_MAX) {
			status_bitfield |= 1;
		}
	}

	// check BAT2 current
	if (adc_get_value(ISENS_BAT2, &adc_value, TASK_DELAY_MS) == W_SUCCESS) {
		if (adc_value > IBAT_MAX) {
			status_bitfield |= 1;
		}
	}

	return status_bitfield;
}

/**
 * Toggles 5V external power rail via a GPIO pin.
 */
w_status_t power_handler_set_5V_external(bool enabled) {
	// Shall not supply power when the charge line is enabled
	if (enabled) {
		uint32_t chg_voltage = 0;
		if (adc_get_value(VSENS_CHG, &chg_voltage, TASK_DELAY_MS) == W_SUCCESS) {
			if (chg_voltage > VCHG_MIN) {
				return W_FAILURE;
			}
		}
	}

	// TO DO: find what pins to turn on via GPIO
	// gpio_write(GPIO_PIN_5V_EXTERNAL, enabled ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW, 5);
	power_handler_status.external_5v_enabled = enabled;
	return W_SUCCESS;
}

/**
 * Toggles low power mode via GPIO pin.
 */
w_status_t power_handler_set_low_power_mode(bool enabled) {
	// TO DO: find what pins to turn on via GPIO
	// gpio_write(GPIO_PIN_LIPO, enabled ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH, 5);
	power_handler_status.low_power_mode = enabled;
	return W_SUCCESS;
}