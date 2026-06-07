#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "message_types.h"
#include "power_handler.h"
#include "rocketlib/include/common.h"
#include "drivers/timer/timer.h"

// LiPo Thresholds
static const uint32_t VBAT_MIN = 22.2;
static const uint32_t VBAT_MAX = 25.4;
static const uint32_t IBAT_MAX = 8000; // 8A in mA

// Rocket Thresholds
static const uint32_t VRKT_MIN = 11.1;
static const uint32_t VRKT_MAX = 12.8;

// Charge Line Thresholds
static const uint32_t VCHG_MIN = 9;
static const uint32_t VCHG_MAX = 14;

// power rail thresholds (mA)
static const uint32_t I3V3_MAX = 500; 
static const uint32_t I5V_MAX = 4000; 

// TODO: Get the correct number for task delay.
#define TASK_DELAY_MS 3000

// Fault bit positions
#define FAULT_BAT1_VOLT (1 << 0)
#define FAULT_BAT2_VOLT (1 << 1)
#define FAULT_RKT_VOLT (1 << 2)
#define FAULT_CHG_VOLT (1 << 3)
#define FAULT_5V_EXT_CURR (1 << 4)
#define FAULT_BAT1_CURR (1 << 5)
#define FAULT_BAT2_CURR (1 << 6)

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
 * Detects the active power source.
 * The active power source has the highest voltage.
 */
static power_input_source_t get_active_input(void) {
	uint32_t vsens_chg = 0;
	uint32_t vsens_rkt = 0;

	adc_get_converted_val(VSENS_CHG, &vsens_chg);
	adc_get_converted_val(VSENS_RKT, &vsens_rkt);

	if (vsens_chg > vsens_rkt && vsens_chg > 0) {
		return POWER_INPUT_CHG;
	} else if (vsens_rkt > 0) {
		return POWER_INPUT_RKT;
	} else {
		return POWER_INPUT_BAT;
	}
}

/**
 * Handles incoming actuator CAN commands for 5V external and low power mode.
 */
static w_status_t power_actuator_callback(const can_msg_t *msg) {
	can_actuator_id_t actuator_id;
	can_actuator_state_t cmd_state;
	w_status_t status = W_SUCCESS;
	uint32_t timestamp = 0;
	can_msg_t response_msg = {0};

	if (get_actuator_id(msg, &actuator_id) != W_SUCCESS ||
		get_cmd_actuator_state(msg, &cmd_state) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (actuator_id == ACTUATOR_CANARD_5V_OUTPUT) {
		status = power_handler_set_5V_external(cmd_state == ACT_STATE_ON);
	} else if (actuator_id == ACTUATOR_CANARD_LIPO_ON) {
		bool lipo_enable = (cmd_state == ACT_STATE_ON);
		status = power_handler_set_low_power_mode(!lipo_enable);

		if (status == W_SUCCESS) {
			timer_get_ms(&timestamp);
			build_actuator_status_msg(PRIO_MEDIUM, (uint16_t)timestamp,
									 ACTUATOR_CANARD_LIPO_ON, cmd_state,
									 lipo_enable ? ACT_STATE_ON : ACT_STATE_OFF,
									 &response_msg);
			can_handler_transmit(&response_msg);
		}
	}

	return status;
}

/**
 * Handles RESET_CMD from RocketCAN.
 */
static w_status_t power_reset_callback(const can_msg_t *msg) {
	uint8_t board_type_id, board_inst_id;
	bool need_reset = false;

	if (get_reset_board_id(msg, &board_type_id, &board_inst_id) != W_SUCCESS) {
		log_text(1, "power_handler", "ERROR: failed to read reset command");
		return W_FAILURE;
	}

	if (check_board_need_reset(msg, &need_reset) == W_SUCCESS && need_reset) {
		log_text(0, "power_handler", "System reset initiated");
		NVIC_SystemReset();
	}

	return W_SUCCESS;
}

/**
 * Initializes power handler.
 * Registers CAN callbacks for payload 5V and low power mode commands.
 * Defaults everything to ON.
 */
w_status_t power_handler_init(void) {
	w_status_t init_status = W_SUCCESS;
	w_status_t cb_status = W_SUCCESS;

	// Default: CHG_MUX_EN HIGH, external 5V OFF, LiPo ON
	gpio_write(GPIO_PIN_CHG_MUX_EN, GPIO_LEVEL_HIGH, 5);
	gpio_write(GPIO_PIN_EN_EXT_5V, GPIO_LEVEL_LOW, 5);
	gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_HIGH, 5);

	// Register callbacks
	cb_status |= can_handler_register_callback(MSG_ACTUATOR_CMD, power_actuator_callback);
	cb_status |= can_handler_register_callback(MSG_RESET_CMD, power_reset_callback);

	if (cb_status != W_SUCCESS) {
		init_status = cb_status;
		log_text(1, "power_handler", "ERROR: Failed to register CAN callbacks");
	}

	power_handler_status.initialized = true;
	power_handler_status.external_5v_enabled = false;
	power_handler_status.low_power_mode = false;

	return init_status;
}

/**
 * Returns uint32_t bitfield of active faults.
 * Fault conditions:
 * 		FAULT_BAT1_VOLT: BAT1 voltage exceeds thresholds
 * 		FAULT_BAT2_VOLT: BAT2 voltage exceeds thresholds
 * 		FAULT_RKT_VOLT: Rocket voltage exceeds thresholds
 * 		FAULT_CHG_VOLT: Charge line exceeds thresholds
 * 		FAULT_5V_EXT_CURR: 5V external overcurrent
 * 		FAULT_BAT1_CURR: BAT1 overcurrent
 * 		FAULT_BAT2_CURR: BAT2 overcurrent
 * Returns 0 if no faults are detected.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void) {
	uint32_t status_bitfield = 0;
	uint32_t adc_value = 0;

	// Check battery fault pins
	gpio_level_t flt1 = GPIO_LEVEL_HIGH;
	gpio_level_t flt2 = GPIO_LEVEL_HIGH;

	gpio_read(GPIO_PIN_BAT_FLT1, &flt1, 5);
	gpio_read(GPIO_PIN_BAT_FLT2, &flt2, 5);

	if (flt1 == GPIO_LEVEL_LOW) {
		status_bitfield |= FAULT_BAT1_VOLT;
		power_handler_status.fault_bat1 = true;
	}
	if (flt2 == GPIO_LEVEL_LOW) {
		status_bitfield |= FAULT_BAT2_VOLT;
		power_handler_status.fault_bat2 = true;
	}

	if (power_handler_status.external_5v_enabled) {
		if (adc_get_converted_val(ISENS_5V, &adc_value) == W_SUCCESS) {
			if (adc_value > I5V_MAX) {
				status_bitfield |= FAULT_5V_EXT_CURR;
			}
		}
	}

	if (get_active_input() == POWER_INPUT_BAT) {
		if (adc_get_converted_val(VSENS_BAT1, &adc_value) == W_SUCCESS) {
			if ((adc_value < VBAT_MIN) || (adc_value > VBAT_MAX)) {
				status_bitfield |= FAULT_BAT1_VOLT;
			}
		}

		if (adc_get_converted_val(VSENS_BAT2, &adc_value) == W_SUCCESS) {
			if ((adc_value < VBAT_MIN) || (adc_value > VBAT_MAX)) {
				status_bitfield |= FAULT_BAT2_VOLT;
			}
		}

		if (adc_get_converted_val(ISENS_BAT1, &adc_value) == W_SUCCESS) {
			if (adc_value > IBAT_MAX) {
				status_bitfield |= FAULT_BAT1_CURR;
			}
		}

		if (adc_get_converted_val(ISENS_BAT2, &adc_value) == W_SUCCESS) {
			if (adc_value > IBAT_MAX) {
				status_bitfield |= FAULT_BAT2_CURR;
			}
		}
	} else if (get_active_input() == POWER_INPUT_RKT) {
		if (adc_get_converted_val(VSENS_RKT, &adc_value) == W_SUCCESS) {
			if ((adc_value < VRKT_MIN) || (adc_value > VRKT_MAX)) {
				status_bitfield |= FAULT_RKT_VOLT;
			}
		}
	} else {
		if (adc_get_converted_val(VSENS_CHG, &adc_value) == W_SUCCESS) {
			if ((adc_value < VCHG_MIN) || (adc_value > VCHG_MAX)) {
				status_bitfield |= FAULT_CHG_VOLT;
			}
		}
	}

	return status_bitfield;
}

/**
 * Toggles 5V external power rail via a GPIO pin.
 * Prevents enabling when CHG is active or low power mode is enabled.
 */
w_status_t power_handler_set_5V_external(bool enabled) {
	if (enabled) {
		// Prevent enabling when low power mode active
		if (power_handler_status.low_power_mode) {
			log_text(10, "power_handler", "Cannot enable 5V external in low power mode");
			return W_FAILURE;
		}

		// Prevent enabling when charge line is active
		if (get_active_input() == POWER_INPUT_CHG) {
			log_text(10, "power_handler", "Cannot enable 5V external when CHG is active");
			return W_FAILURE;
		}

		// Enable 5V external and set CHG_MUX_EN LOW
		gpio_write(GPIO_PIN_EN_EXT_5V, GPIO_LEVEL_HIGH, 5);
		gpio_write(GPIO_PIN_CHG_MUX_EN, GPIO_LEVEL_LOW, 5);
	} else {
		// Disable 5V external and set CHG_MUX_EN HIGH
		gpio_write(GPIO_PIN_EN_EXT_5V, GPIO_LEVEL_LOW, 5);
		gpio_write(GPIO_PIN_CHG_MUX_EN, GPIO_LEVEL_HIGH, 5);
	}

	power_handler_status.external_5v_enabled = enabled;
	return W_SUCCESS;
}

/**
 * Toggles low power mode via GPIO pin.
 * Disables external 5V and turns off LiPo when enabling.
 */
w_status_t power_handler_set_low_power_mode(bool enabled) {
	if (enabled) {
		// Disable external 5V when entering low power mode
		power_handler_set_5V_external(false);

		// Disable LiPo
		gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_LOW, 5);
		log_text(10, "power_handler", "Low power mode enabled: LiPo disabled");
	} else {
		// Enable LiPo when exiting low power mode
		gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_HIGH, 5);
		log_text(10, "power_handler", "Low power mode disabled: LiPo enabled");
	}

	power_handler_status.low_power_mode = enabled;
	return W_SUCCESS;
}