#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "canlib/message_types.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "drivers/timer/timer.h"
#include "power_handler.h"
#include "rocketlib/include/common.h"

// Fault bit positions
static uint32_t FAULT_BAT1_VOLT = (UINT32_C(1) << 0);
static uint32_t FAULT_BAT2_VOLT = (UINT32_C(1) << 1);
static uint32_t FAULT_RKT_VOLT = (UINT32_C(1) << 2);
static uint32_t FAULT_CHG_VOLT = (UINT32_C(1) << 3);
static uint32_t FAULT_5V_CURR = (UINT32_C(1) << 4);
static uint32_t FAULT_5V_EXT_OUTPUT = (UINT32_C(1) << 5);
static uint32_t FAULT_3V3_CURR = (UINT32_C(1) << 6);
static uint32_t FAULT_BAT1_CURR = (UINT32_C(1) << 7);
static uint32_t FAULT_BAT2_CURR = (UINT32_C(1) << 8);

/**
 * States of the power handler.
 */
typedef struct {
	bool initialized;
	bool external_5v_enabled;
	bool low_power_mode;
	uint32_t external_5v_fault_count;
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

// LiPo Thresholds
static const float VBAT_MIN = 22.2;
static const float VBAT_MAX = 25.4;
static const float IBAT_MAX = 8000; // 8A in mA

// Rocket Thresholds
static const float VRKT_MIN = 11.1;
static const float VRKT_MAX = 12.8;

// Charge Line Thresholds
static const float VCHG_MIN = 9;
static const float VCHG_MAX = 14;

// power rail thresholds (mA)
static const float I3V3_MAX = 500;
static const float I5V_MAX = 4000;

static power_handler_status_t power_handler_status = {0};

/**
 * Detects the active power source.
 * The active power source has the highest voltage.
 */
static power_input_source_t get_active_input(void) {
	float vsens_chg = 0;
	float vsens_rkt = 0;
	float vsens_usb = 0;
	float vsens_bat1 = 0;
	float vsens_bat2 = 0;
	w_status_t status = W_FAILURE;

	status = adc_get_converted_val(VSENS_CHG, &vsens_chg);
	if (W_SUCCESS != status) {
		log_text(1,
				 "power handler",
				 "adc communication failed while getting charge voltage. Status: %d",
				 status);
	}

	status = adc_get_converted_val(VSENS_RKT, &vsens_rkt);
	if (W_SUCCESS != status) {
		log_text(1,
				 "power handler",
				 "adc communication failed while getting rocket voltage.Status: %d",
				 status);
	}

	status = adc_get_converted_val(VSENS_USB, &vsens_usb);
	if (W_SUCCESS != status) {
		log_text(1,
				 "power handler",
				 "adc communication failed while getting usb voltage.Status: %d",
				 status);
	}

	status = adc_get_converted_val(VSENS_BAT1, &vsens_bat1);
	if (W_SUCCESS != status) {
		log_text(1,
				 "power handler",
				 "adc communication failed while getting battery 1 voltage.Status: %d",
				 status);
	}

	status = adc_get_converted_val(VSENS_BAT2, &vsens_bat2);
	if (W_SUCCESS != status) {
		log_text(1,
				 "power handler",
				 "adc communication failed while getting battery 2 voltage.Status: %d",
				 status);
	}

	if (((int)vsens_chg == 0) && ((int)vsens_rkt == 0) && ((int)vsens_bat1 == 0) &&
		((int)vsens_bat2 == 0)) {
		return POWER_INPUT_NONE;
	} else if ((vsens_chg >= vsens_rkt) && (vsens_chg >= vsens_usb) && (vsens_chg >= vsens_bat1) &&
			   (vsens_chg >= vsens_bat2)) {
		return POWER_INPUT_CHG;
	} else if ((vsens_rkt >= vsens_chg) && (vsens_rkt >= vsens_usb) && (vsens_rkt >= vsens_bat1) &&
			   (vsens_rkt >= vsens_bat2)) {
		return POWER_INPUT_RKT;
	} else {
		return POWER_INPUT_BAT;
	}
}

/*
 * Transmits power status CAN messages and power fault CAN messages if faults are detected.
 * Power status messages include battery voltages and currents, rocket voltage, charge voltage, and
 * 5V rail current. Fault messages include a bitfield of active faults. Called by
 * power_handler_get_status
 */
static void transmit_status_can_msg(uint32_t status_bitfield) {
	can_msg_t status_msg = {0};
	float adc_value = 0;
	can_msg_t msg = {0};

	uint32_t timestamp = 0;
	w_status_t can_tx_status = W_SUCCESS;

	if (W_SUCCESS != timer_get_ms(&timestamp)) {
		log_text(1, "power_handler", "WARNING: Failed to get timestamp for power status can msg.");
	}

	if (W_SUCCESS == adc_get_converted_val(VSENS_BAT1, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_BATT_VOLT, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS == adc_get_converted_val(VSENS_BAT2, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_BATT_VOLT, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS == adc_get_converted_val(ISENS_BAT1, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_BATT_CURR, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS == adc_get_converted_val(ISENS_BAT2, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_BATT_CURR, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS == adc_get_converted_val(ISENS_5V, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_5V_CURR, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS == adc_get_converted_val(VSENS_RKT, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_12V_VOLT, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS == adc_get_converted_val(VSENS_CHG, &adc_value)) {
		build_analog_sensor_16bit_msg(
			PRIO_LOW, (uint16_t)timestamp, SENSOR_CHARGE_VOLT, adc_value, &msg);
		can_tx_status |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS != can_tx_status) {
		log_text(1, "power_handler", "WARNING: Some can messages failed to transmit.");
	}
}

/**
 * Returns static uint32_t bitfield of active faults.
 * Fault conditions:
 * 		FAULT_BAT1_VOLT: BAT1 voltage exceeds thresholds  (does not fault when the lipo is not
 * plugged in) FAULT_BAT2_VOLT: BAT2 voltage exceeds thresholds FAULT_RKT_VOLT: Rocket voltage
 * exceeds thresholds FAULT_CHG_VOLT: Charge line exceeds thresholds FAULT_5V_CURR: 5V power rail
 * overcurrent FAULT_5V_OUTPUT: 5V extern output fault FAULT_BAT1_CURR: BAT1 overcurrent
 * 		FAULT_BAT2_CURR: BAT2 overcurrent
 * Returns 0 if no faults are detected.
 * Called by health checks.
 */
uint32_t power_handler_get_status(void) {
	static uint32_t status_bitfield = 0;
	w_status_t gpio_read_status = W_SUCCESS;
	float adc_value = 0;

	// Check battery fault pins
	gpio_level_t flt1 = GPIO_LEVEL_HIGH;
	gpio_level_t flt2 = GPIO_LEVEL_HIGH;
	gpio_level_t pg_ext_5v = GPIO_LEVEL_HIGH;

	gpio_read_status |= gpio_read(GPIO_PIN_BAT_FLT1, &flt1, 5);
	gpio_read_status |= gpio_read(GPIO_PIN_BAT_FLT2, &flt2, 5);
	gpio_read_status |= gpio_read(GPIO_PIN_PG_EXT_5V, &pg_ext_5v, 5);

	if (GPIO_LEVEL_LOW == flt1) {
		status_bitfield |= FAULT_BAT1_VOLT;
		power_handler_status.lipo_1_fault_count++;
	}

	if (GPIO_LEVEL_LOW == flt2) {
		status_bitfield |= FAULT_BAT2_VOLT;
		power_handler_status.lipo_2_fault_count++;
	}

	// Check external 5V output fault
	if (GPIO_LEVEL_LOW == pg_ext_5v) {
		status_bitfield |= FAULT_5V_EXT_OUTPUT;
		power_handler_status.external_5v_fault_count++;
	}

	// external and internal 5V share the same current sense, so if either is overcurrent it will
	// trigger the fault
	if (W_SUCCESS == adc_get_converted_val(ISENS_5V, &adc_value)) {
		if (adc_value > I5V_MAX) {
			status_bitfield |= FAULT_5V_CURR;
			power_handler_status.overcurrent_count++;
		}
	}

	if (W_SUCCESS == adc_get_converted_val(ISENS_3V3, &adc_value)) {
		if (adc_value > I3V3_MAX) {
			status_bitfield |= FAULT_3V3_CURR;
			power_handler_status.overcurrent_count++;
		}
	}

	power_input_source_t active_input = get_active_input();

	// Log active power source
	switch (active_input) {
		case POWER_INPUT_CHG:
			if (W_SUCCESS == adc_get_converted_val(VSENS_CHG, &adc_value)) {
				if ((adc_value < VCHG_MIN) || (adc_value > VCHG_MAX)) {
					status_bitfield |= FAULT_CHG_VOLT;
				}
			}
			log_text(10, "power_handler", "Active power source: CHG");
			break;

		case POWER_INPUT_RKT:
			if (W_SUCCESS == adc_get_converted_val(VSENS_RKT, &adc_value)) {
				if ((adc_value < VRKT_MIN) || (adc_value > VRKT_MAX)) {
					status_bitfield |= FAULT_RKT_VOLT;
				}
			}
			log_text(10, "power_handler", "Active power source: RKT");
			break;

		case POWER_INPUT_BAT:
			if (W_SUCCESS == adc_get_converted_val(VSENS_BAT1, &adc_value)) {
				if ((adc_value < VBAT_MIN) || (adc_value > VBAT_MAX)) {
					status_bitfield |= FAULT_BAT1_VOLT;
				}
			}

			if (W_SUCCESS == adc_get_converted_val(VSENS_BAT2, &adc_value)) {
				if ((adc_value < VBAT_MIN) || (adc_value > VBAT_MAX)) {
					status_bitfield |= FAULT_BAT2_VOLT;
				}
			}

			if (W_SUCCESS == adc_get_converted_val(ISENS_BAT1, &adc_value)) {
				if (adc_value > IBAT_MAX) {
					status_bitfield |= FAULT_BAT1_CURR;
					power_handler_status.overcurrent_count++;
				}
			}

			if (W_SUCCESS == adc_get_converted_val(ISENS_BAT2, &adc_value)) {
				if (adc_value > IBAT_MAX) {
					status_bitfield |= FAULT_BAT2_CURR;
					power_handler_status.overcurrent_count++;
				}
			}

			log_text(10, "power_handler", "Active power source: BAT");
			break;

		default:
			log_text(10, "power_handler", "Active power source: NONE");
			break;
	}

	static uint32_t timestamp = 0;
	can_msg_t msg = {0};

	if (0 != status_bitfield) {
		timer_get_ms(&timestamp);
		log_text(10, "power_handler", "Power fault detected: 0x%lx", status_bitfield);
		build_general_board_status_msg(PRIO_HIGH, (uint16_t)timestamp, status_bitfield, &msg);
		if (W_SUCCESS != can_handler_transmit(&msg)) {
			log_text(1,
					 "power_handler",
					 "WARNING: can messsage tx failed at power handler status check.");
		}
	}

	transmit_status_can_msg(status_bitfield);

	return status_bitfield;
}

/**
 * Toggles 5V external power rail via a GPIO pin.
 * Prevents enabling when CHG is active or low power mode is enabled.
 */
static w_status_t power_handler_set_5V_external(bool enabled) {
	w_status_t gpio_status = W_SUCCESS;

	if (enabled) {
		// Prevent enabling when low power mode active
		if (power_handler_status.low_power_mode) {
			log_text(10, "power_handler", "Cannot enable 5V external in low power mode");
			return W_FAILURE;
		}

		// Prevent enabling when charge line is active
		if (POWER_INPUT_CHG == get_active_input()) {
			log_text(10, "power_handler", "Cannot enable 5V external when CHG is active");
			return W_FAILURE;
		}

		// Enable 5V external and set CHG_MUX_EN LOW
		// since chg_mux MUST be disabled before enabling 5v, u MUST disable the chg mux en pin
		// first!
		gpio_status |= gpio_write(GPIO_PIN_CHG_MUX_EN, GPIO_LEVEL_LOW, 5);
		gpio_status |= gpio_write(GPIO_PIN_EN_EXT_5V, GPIO_LEVEL_HIGH, 5);
	} else {
		// Disable 5V external and set CHG_MUX_EN HIGH
		gpio_status |= gpio_write(GPIO_PIN_EN_EXT_5V, GPIO_LEVEL_LOW, 5);
		gpio_status |= gpio_write(GPIO_PIN_CHG_MUX_EN, GPIO_LEVEL_HIGH, 5);
	}

	power_handler_status.external_5v_enabled = enabled;

	if (W_SUCCESS != gpio_status) {
		log_text(1,
				 "power_handler",
				 "ERROR: gpio write failed while toggling 5v external. Error code: %d",
				 gpio_status);
		return gpio_status;
	}

	return W_SUCCESS;
}

/**
 * Toggles low power mode via GPIO pin.
 * Disables external 5V and turns off LiPo when enabling.
 */
static w_status_t power_handler_set_low_power_mode(bool enabled) {
	w_status_t gpio_status = W_SUCCESS;

	if (enabled) {
		// Disable external 5V when entering low power mode
		power_handler_set_5V_external(false);

		// Disable LiPo
		gpio_status |= gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_LOW, 5);

		if (W_SUCCESS != gpio_status) {
			log_text(1,
					 "power_handler",
					 "ERROR: gpio write failed while diabling lipo. Error code: %d",
					 gpio_status);
			return gpio_status;
		}

		log_text(10, "power_handler", "Low power mode enabled: LiPo disabled");
	} else {
		// Enable LiPo when exiting low power mode
		gpio_status |= gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_HIGH, 5);

		if (W_SUCCESS != gpio_status) {
			log_text(1,
					 "power_handler",
					 "ERROR: gpio write failed while enabling lipo. Error code: %d",
					 gpio_status);
			return gpio_status;
		}

		log_text(10, "power_handler", "Low power mode disabled: LiPo enabled");
	}

	power_handler_status.low_power_mode = enabled;
	return W_SUCCESS;
}

/**
 * Handles incoming actuator CAN commands for 5V external and low power mode.
 */
static w_status_t power_actuator_callback(const can_msg_t *msg) {
	can_actuator_id_t actuator_id;
	can_actuator_state_t cmd_state;
	w_status_t status = W_SUCCESS;
	static uint32_t timestamp = 0;
	can_msg_t response_msg = {0};

	if ((W_SUCCESS != get_actuator_id(msg, &actuator_id)) ||
		(W_SUCCESS != get_cmd_actuator_state(msg, &cmd_state))) {
		log_text(5, "power_handler", "ERROR: Failed to get actuator id or state.");
		return W_FAILURE;
	}

	if (ACTUATOR_CANARD_5V_OUTPUT == actuator_id) {
		bool enable_5v = (ACT_STATE_ON == cmd_state);
		status = power_handler_set_5V_external(enable_5v);
	} else if (ACTUATOR_CANARD_LIPO_ON == actuator_id) {
		bool lipo_enable = (ACT_STATE_ON == cmd_state);
		status = power_handler_set_low_power_mode(!lipo_enable);

		if (W_SUCCESS == status) {
			if (W_SUCCESS != timer_get_ms(&timestamp)) {
				log_text(1,
						 "power_handler",
						 "WARNING: Failed to get timestamp for actuator status can msg.");
			}
			build_actuator_status_msg(PRIO_MEDIUM,
									  (uint16_t)timestamp,
									  ACTUATOR_CANARD_LIPO_ON,
									  cmd_state,
									  lipo_enable ? ACT_STATE_ON : ACT_STATE_OFF,
									  &response_msg);
			if (W_SUCCESS != can_handler_transmit(&response_msg)) {
				log_text(1,
						 "power_handler",
						 "WARNING: Can message failed to transmit for actruator status.");
			}
		}
	}

	return status;
}

/**
 * Handles RESET_CMD from RocketCAN.
 */
static w_status_t power_reset_callback(const can_msg_t *msg) {
	bool need_reset = true;

	if ((W_SUCCESS == check_board_need_reset(msg, &need_reset)) && need_reset) {
		log_text(0, "power_handler", "INFO: System reset initiated");
		NVIC_SystemReset();
	} else {
		log_text(1, "power_handler", "ERROR: failed to read reset command");
		return W_FAILURE;
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
	w_status_t gpio_status = W_SUCCESS;

	// Default: CHG_MUX_EN HIGH, external 5V ON, LiPo ON
	gpio_status |= gpio_write(GPIO_PIN_CHG_MUX_EN, GPIO_LEVEL_HIGH, 5);
	gpio_status |= gpio_write(GPIO_PIN_EN_EXT_5V, GPIO_LEVEL_HIGH, 5);
	gpio_status |= gpio_write(GPIO_PIN_PWR_EN, GPIO_LEVEL_HIGH, 5);

	if (W_SUCCESS != gpio_status) {
		log_text(5, "power_handler", "ERROR: Failed with gpio write during init.");
	}

	// Register callbacks
	cb_status |= can_handler_register_callback(MSG_ACTUATOR_CMD, power_actuator_callback);
	cb_status |= can_handler_register_callback(MSG_RESET_CMD, power_reset_callback);

	if (W_SUCCESS != cb_status) {
		log_text(1, "power_handler", "ERROR: Failed to register CAN callbacks during init.");
	}

	init_status = (cb_status & gpio_status);

	power_handler_status.initialized = true;
	power_handler_status.external_5v_enabled = true;
	power_handler_status.low_power_mode = false;

	return init_status;
}
