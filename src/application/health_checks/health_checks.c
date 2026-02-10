#include "health_checks.h"
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "can.h"
#include "drivers/adc/adc.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"
#include "message_types.h"
#include "printf.h"
#include "task.h"

#define TASK_DELAY_MS 3000
#define ADC_VREF 3.3f
#define R_SENSE 0.033f
#define INA180A3_GAIN 100.0f
#define MAX_CURRENT_mA 400
#define MAX_WATCHDOG_TASKS 10
#define CONV_ADC_COUNTS_TO_CURRENT_mA                                                              \
	((ADC_VREF * 1000.0f) / (ADC_MAX_COUNTS * INA180A3_GAIN * R_SENSE))

// struct for watchdog
typedef struct {
	TaskHandle_t task_handle;
	bool is_kicked;
	float last_kick_timestamp;
	uint32_t timeout_ticks;
} watchdog_task_t;

// watchdog initiailsations
static watchdog_task_t watchdog_tasks[MAX_WATCHDOG_TASKS] = {0};
static uint32_t num_watchdog_tasks = 0;

w_status_t get_adc_current(uint32_t *adc_current_mA) {
	w_status_t status = W_SUCCESS;
	uint32_t adc_value;

	status |= adc_get_value(PROCESSOR_BOARD_VOLTAGE, &adc_value, TASK_DELAY_MS);
	if (status != W_SUCCESS) {
		return status;
	}

	*adc_current_mA = (uint32_t)(adc_value * CONV_ADC_COUNTS_TO_CURRENT_mA);

	return W_SUCCESS;
}

uint32_t check_current(void) {
	uint32_t status = 0;
	uint32_t adc_current_mA;

	if (get_adc_current(&adc_current_mA) == W_SUCCESS) {
		float ms = 0;
		timer_get_ms(&ms);
		can_msg_t msg = {0};

		// always send current sense msg to can
		build_analog_data_msg(PRIO_LOW, (uint16_t)ms, SENSOR_5V_CURR, adc_current_mA, &msg);
		status |= can_handler_transmit(&msg);

		// send CAN err msg and log text if over current
		if (adc_current_mA > MAX_CURRENT_mA) {
			status |= 1 << E_5V_OVER_CURRENT_OFFSET;
			log_text(10, "health_checks", "5V overcurrent: %d mA", adc_current_mA);
		} else {
		}
	}

	return status;
}

w_status_t health_check_init(void) {
	num_watchdog_tasks = 0;
	return W_SUCCESS;
}

w_status_t watchdog_kick(void) {
	TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	float current_time = 0.0f;
	w_status_t status = W_SUCCESS;

	status |= timer_get_ms(&current_time);
	if (status != W_SUCCESS) {
		log_text(0, "health_checks", "timer_get_ms failure");
		return status;
	}

	// Check if the current task is registered
	for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
		if (watchdog_tasks[i].task_handle == current_task) {
			watchdog_tasks[i].is_kicked = true;
			watchdog_tasks[i].last_kick_timestamp = current_time;
			return W_SUCCESS;
		}
	}
	return W_FAILURE;
}

w_status_t watchdog_register_task(TaskHandle_t task_handle, uint32_t timeout_ticks) {
	if ((NULL == task_handle) || (0 == timeout_ticks)) {
		log_text(0, "health_checks", "invalid arguments into watchdog register");
		return W_INVALID_PARAM;
	}

	if (num_watchdog_tasks >= MAX_WATCHDOG_TASKS) {
		log_text(0, "health_checks", "max watchdog tasks reached");
		return W_FAILURE;
	}

	// Check if the task is already registered
	for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
		if (watchdog_tasks[i].task_handle == task_handle) {
			log_text(0, "health_checks", "duplicate task registration:%p", (void *)task_handle);
			return W_FAILURE;
		}
	}

	float current_time = 0.0f;
	w_status_t status = W_SUCCESS;
	status |= timer_get_ms(&current_time);

	watchdog_tasks[num_watchdog_tasks].task_handle = task_handle;
	watchdog_tasks[num_watchdog_tasks].is_kicked = true;
	watchdog_tasks[num_watchdog_tasks].last_kick_timestamp = current_time;
	watchdog_tasks[num_watchdog_tasks].timeout_ticks = timeout_ticks;

	num_watchdog_tasks++; // incriminent the watchdog task count for future ref

	return status;
}

/**
 * @brief Checks all registered tasks for watchdog timeouts
 *
 * Compares the current time with the last kick timestamp of each task.
 * If a task exceeds its timeout, sends a CAN status message indicating a timeout.
 * Should be called periodically (typically through health_check_exec).
 *
 * @return CAN board specific err bitfield
 */
uint32_t check_watchdog_tasks(void) {
	uint32_t status_bitfield = 0;
	float current_time = 0.0f;

	// failing to get time isn't a fatal err
	if (timer_get_ms(&current_time) != W_SUCCESS) {
		log_text(0, "health_checks", "timer_get_ms failure");
		status_bitfield |= 1 << E_IO_ERROR_OFFSET;
	}

	for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
		float time_elapsed = current_time - watchdog_tasks[i].last_kick_timestamp;
		uint32_t ticks_elapsed = pdMS_TO_TICKS((uint32_t)time_elapsed); // time to ticks

		if (watchdog_tasks[i].is_kicked || (ticks_elapsed <= watchdog_tasks[i].timeout_ticks)) {
			// do nothing if any one is true
		} else {
			// report watchdog timeout
			log_text(10, "health_checks", "task timeout: %d", i);
			status_bitfield |= 1 << E_WATCHDOG_TIMEOUT_OFFSET;
		}

		// resetting for next check
		watchdog_tasks[i].is_kicked = false;
	}
	return status_bitfield;
}

/**
 * @brief Checks the status of all known modules by directly calling their get_status functions
 *
 * Simply calls each module's status function to trigger status reporting
 */
static uint32_t check_modules_status(void) {
	// CAN error bitfield
	uint32_t status_bitfield = 0;

	// Call each module's get_status function
	// These functions handle their own status checking, logging, and CAN messaging

	status_bitfield |= i2c_get_status();
	status_bitfield |= adc_get_status();
	status_bitfield |= can_handler_get_status();
	status_bitfield |= estimator_get_status();
	status_bitfield |= controller_get_status();
	status_bitfield |= sd_card_get_status();
	status_bitfield |= timer_get_status();
	status_bitfield |= gpio_get_status();
	status_bitfield |= flight_phase_get_status();
	status_bitfield |= imu_handler_get_status();
	status_bitfield |= uart_get_status();

	if (logger_get_status() == W_FAILURE) {
		status_bitfield |= (1 << E_FS_ERROR_OFFSET);
		log_text(5, "health", "logger not init");
	}

	return status_bitfield;
}

w_status_t health_check_exec() {
	uint32_t status_bitfield = 0;

	status_bitfield |= check_current();
	status_bitfield |= check_watchdog_tasks();
	status_bitfield |= check_modules_status();

	// send status CAN msg
	can_msg_t msg = {0};
	if (build_general_board_status_msg(
			PRIO_LOW, xTaskGetTickCount(), status_bitfield, status_bitfield, &msg) == true) {
		if (can_handler_transmit(&msg) != W_SUCCESS) {
			log_text(0, "health_checks", "CAN send failure for status msg");
			return W_FAILURE;
		}
	} else {
		log_text(0, "health_checks", "build_general_board_status_msg failure");
		return W_FAILURE;
	}

	return W_SUCCESS;
}

void health_check_task(void *argument) {
	(void)argument;

	TickType_t lastWakeTime = xTaskGetTickCount();

	for (;;) {
		health_check_exec();
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_DELAY_MS));
	}
}
