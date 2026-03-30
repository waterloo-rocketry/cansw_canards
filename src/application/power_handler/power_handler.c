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
// LiPo Thresholds
#define VBAT_MIN 10 
#define VBAT_MAX 10
#define IBAT_MAX 10

// Rocket Thresholds
#define VRKT_MIN 10 
#define VRKT_MAX 10

// Charge Line Thresholds
#define VCHG_MIN 10 
#define VCHG_MAX 10

// 5V External Thresholds
#define V5V_EXTERNAL_MIN 10 
#define V5V_EXTERNAL_MAX 10
#define I5V_EXTERNAL_MAX 10

// TODO: Get the correct number for task delay. 
#define TASK_DELAY_MS 3000

// Fault bit positions
#define FAULT_BAT1_VOLT    (1 << 0)
#define FAULT_BAT2_VOLT    (1 << 1)
#define FAULT_RKT_VOLT     (1 << 2)
#define FAULT_CHG_VOLT     (1 << 3)
#define FAULT_5V_EXT_CURR  (1 << 4)
#define FAULT_BAT1_CURR    (1 << 5)
#define FAULT_BAT2_CURR    (1 << 6)

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

typedef enum {
	POWER_INPUT_CHG,
	POWER_INPUT_RKT,
	POWER_INPUT_BAT
} power_input_source_t;

static power_handler_status_t power_handler_status = {0};

/**
 * Handles incoming actuator CAN commands for 5V external and low power mode.
 */
static w_status_t power_actuator_callback(const can_msg_t *msg) 
{
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
 * Detects the active power source. 
 * The active power source has the highest voltage. 
 */
static power_input_source_t get_active_input(void) 
{
    uint32_t vsens_chg = 0;
    uint32_t vsens_rkt = 0;

    adc_get_value(VSENS_CHG, &vsens_chg, TASK_DELAY_MS);
    adc_get_value(VSENS_RKT, &vsens_rkt, TASK_DELAY_MS);

    if (vsens_chg > vsens_rkt && vsens_chg > 0) 
	{
		return POWER_INPUT_CHG;
    } 
	else if (vsens_rkt > 0) 
	{
        return POWER_INPUT_RKT;
    }
	else 
	{
		return POWER_INPUT_BAT;
	}
}

/**
 * Initializes power handler.
 * Registers CAN callbacks for payload 5V and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void) 
{
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
uint32_t power_handler_get_status(void) 
{
	uint32_t status_bitfield = 0;
	uint32_t adc_value = 0;

	// TO DO: Configure the fault pins so this following code works
	// gpio_level_t flt1 = GPIO_LEVEL_HIGH;
	// gpio_level_t flt2 = GPIO_LEVEL_HIGH;

	// gpio_read(BAT_FLT1, &flt1, 5);
	// gpio_read(BAT_FLT2, &flt2, 5);

	// if (flt1 == GPIO_LEVEL_LOW) 
	// {
	// 	status_bitfield |= 1;
	// 	power_handler_status.fault_bat1 = true;
	// }
	// if (flt2 == GPIO_LEVEL_LOW) 
	// {
	// 	status_bitfield |= 1;
	// 	power_handler_status.fault_bat2 = true;
	// }

	if (power_handler_status.external_5v_enabled) {
		if (adc_get_value(ISENS_5V, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if (adc_value > I5V_EXTERNAL_MAX) 
			{
				status_bitfield |= FAULT_5V_EXT_CURR;
			}
		}
	}


	if (get_active_input() == POWER_INPUT_BAT) 
	{
		if (adc_get_value(VSENS_BAT1, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if ((adc_value < VBAT_MIN) || (adc_value > VBAT_MAX)) 
			{
				status_bitfield |= FAULT_BAT1_VOLT;
			}
		}

		if (adc_get_value(VSENS_BAT2, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if ((adc_value < VBAT_MIN) || (adc_value > VBAT_MAX)) 
			{
				status_bitfield |= FAULT_BAT2_VOLT;
			}
		}

		// check BAT1 current
		if (adc_get_value(ISENS_BAT1, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if (adc_value > IBAT_MAX)
			{
				status_bitfield |= FAULT_BAT1_CURR;
			}
		}

		// check BAT2 current
		if (adc_get_value(ISENS_BAT2, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if (adc_value > IBAT_MAX) 
			{
				status_bitfield |= FAULT_BAT1_CURR;
			}
		}
	}
	else if (get_active_input() == POWER_INPUT_RKT) 
	{
		if (adc_get_value(VSENS_RKT, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if ((adc_value < VRKT_MIN) || (adc_value > VRKT_MAX)) 
			{
				status_bitfield |= FAULT_RKT_VOLT;
			}
		}
	}
	else 
	{
		if (adc_get_value(VSENS_CHG, &adc_value, TASK_DELAY_MS) == W_SUCCESS) 
		{
			if ((adc_value < VCHG_MIN) || (adc_value > VCHG_MAX)) 
			{
				status_bitfield |= FAULT_CHG_VOLT;
			}
		}
	}	

	return status_bitfield;
}

/**
 * Toggles 5V external power rail via a GPIO pin.
 */
w_status_t power_handler_set_5V_external(bool enabled) 
{
	// Shall not supply power when the charge line is enabled
	if (get_active_input() == POWER_INPUT_CHG) 
	{
		return W_FAILURE; 
	}

	// TO DO: find what pins to turn on via GPIO
	// gpio_write(GPIO_PIN_5V_EXTERNAL, enabled ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW, 5);
	
	power_handler_status.external_5v_enabled = enabled;
	return W_SUCCESS;
}

/**
 * Toggles low power mode via GPIO pin.
 */
w_status_t power_handler_set_low_power_mode(bool enabled) 
{
	// TO DO: find what pins to turn on via GPIO
	// gpio_write(GPIO_PIN_LIPO, enabled ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH, 5);

	power_handler_status.low_power_mode = enabled;
	return W_SUCCESS;
}
