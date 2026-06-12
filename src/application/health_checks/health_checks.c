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
#include "fdcan.h"
#include "message_types.h"
#include "printf.h"
#include "task.h"

#define TASK_DELAY_MS 1000
#define MAX_WATCHDOG_TASKS 10

// struct for watchdog
typedef struct {
	TaskHandle_t task_handle;
	bool is_kicked;
	uint32_t last_kick_timestamp_ms; // ms
	uint32_t timeout_ticks;
} watchdog_task_t;

// map for module IDs
typedef health_status_t (*get_module_status_t)(void);

// watchdog initiailsations
static watchdog_task_t watchdog_tasks[MAX_WATCHDOG_TASKS] = {0};
static uint32_t num_watchdog_tasks = 0;

w_status_t health_check_init(void) {
	num_watchdog_tasks = 0;
	return W_SUCCESS;
}

w_status_t watchdog_kick(void) {
	TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	uint32_t current_time = 0;
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
			watchdog_tasks[i].last_kick_timestamp_ms = current_time;
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

	uint32_t current_time = 0;
	w_status_t status = W_SUCCESS;
	status |= timer_get_ms(&current_time);

	watchdog_tasks[num_watchdog_tasks].task_handle = task_handle;
	watchdog_tasks[num_watchdog_tasks].is_kicked = true;
	watchdog_tasks[num_watchdog_tasks].last_kick_timestamp_ms = current_time;
	watchdog_tasks[num_watchdog_tasks].timeout_ticks = timeout_ticks;

	num_watchdog_tasks++; // increment the watchdog task count for future ref

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
	uint32_t current_time = 0;

	// failing to get time isn't a fatal err
	if (timer_get_ms(&current_time) != W_SUCCESS) {
		log_text(0, "health_checks", "timer_get_ms failure");
		status_bitfield |= 1 << E_IO_ERROR_OFFSET;
	}

	for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
		uint32_t time_elapsed = current_time - watchdog_tasks[i].last_kick_timestamp_ms;
		uint32_t ticks_elapsed = pdMS_TO_TICKS((uint32_t)time_elapsed); // time to ticks

		if (watchdog_tasks[i].is_kicked || (ticks_elapsed <= watchdog_tasks[i].timeout_ticks)) {
			// do nothing if any one is true
		} else {
			// report watchdog timeout
			can_msg_t msg = {0};
			char *task_name = pcTaskGetName(watchdog_tasks[i].task_handle);
			uint8_t data[6] = {0};
			strncpy((char *)data, task_name, sizeof(data));
			data[sizeof(data) - 1] = '\0'; // ensure null termination
			build_debug_raw_msg(PRIO_HIGH, xTaskGetTickCount(), data, &msg);
			if (can_handler_transmit(&msg) != W_SUCCESS) {
				log_text(0, "health", "CAN send failure for watchdog timeout msg");
			}
			log_text(0, "health_checks", "task timeout: %d", *task_name);
			status_bitfield |= 1 << E_WATCHDOG_TIMEOUT_OFFSET;
		}

		// resetting for next check
		watchdog_tasks[i].is_kicked = false;
	}
	return status_bitfield;
}

/**
 * @brief Processes the status a module. If severity is not HEALTH_OK,l send CAN canards firmware
 * error msg. If severity is HEALTH_FATAL, call fatal error handler
 *
 * @return W_SUCCESS if severity is HEALTH_OK. W_FAILURE if not
 */
static w_status_t process_module_status(health_status_t status) {
	if (status.severity != HEALTH_OK) {
		log_text(0,
				 "health",
				 "module=%d: sev=%d, err=%d",
				 status.module_id,
				 status.severity,
				 status.error_bitfield);

		uint8_t data[6] = {0};
		can_msg_t msg = {0};
		// build error msg in the form "module id:error bitfield:severity"
		snprintf_((char *)data, sizeof(data), "%d:%lu", status.module_id, status.error_bitfield);
		build_debug_raw_msg(PRIO_HIGH, xTaskGetTickCount(), data, &msg);
		if (can_handler_transmit(&msg) != W_SUCCESS) {
			log_text(0, "health", "CAN send failure for module status msg");
		}

		if (HEALTH_FATAL == status.severity) {
			proc_handle_fatal_error((char *)data);
		}
		return W_FAILURE;
	}
	return W_SUCCESS;
}

static const get_module_status_t module_get_status_fns[MODULE_COUNT] = {
	[MODULE_I2C] = i2c_get_status,
	[MODULE_ADC] = adc_get_status,
	[MODULE_CAN_HANDLER] = can_handler_get_status,
	[MODULE_ESTIMATOR] = estimator_get_status,
	[MODULE_CONTROLLER] = controller_get_status,
	[MODULE_SD_CARD] = sd_card_get_status,
	[MODULE_TIMER] = timer_get_status,
	[MODULE_GPIO] = gpio_get_status,
	[MODULE_FLIGHT_PHASE] = flight_phase_get_status,
	[MODULE_IMU_HANDLER] = imu_handler_get_status,
	[MODULE_UART] = uart_get_status,
	[MODULE_LOGGER] = logger_get_status,
};

/**
 * @brief Checks the status of all known modules by directly calling their get_status functions
 *
 * @return bitfield of all canard application modules' w_status_t
 * Calls each module's status function to trigger status reporting
 */

static uint32_t check_modules_status(void) {
	uint32_t status_bitfield = 0;
	w_status_t status = W_SUCCESS;

	for (int i = 0; i < MODULE_COUNT; i++) {
		status |= process_module_status(module_get_status_fns[i]());
	}

	if (status != W_SUCCESS) {
		// this bitfield error offset should later by replaced with E_CANARD_MODULE_FAILURE_OFFSET);
		status_bitfield |= (1 << E_WATCHDOG_TIMEOUT_OFFSET);
	}

	return status_bitfield;
}

// --- Fatal Error Handler Implementation ---

// Note: All IDs (Board Type, Message Type, Instance ID)
//       are used directly from canlib/message_types.h

void proc_handle_fatal_error(const char *errorMsg) {
	static bool can_initialized = false;
	// safe state - loop here forever and send CAN err msg repeatedly
	while (1) {
		can_initialized = can_initialized || stm32h7_can_init(&hfdcan3, can_handle_rx_message);
		__disable_irq();

		// let CAN still work
		HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);

		can_msg_t msg;
		uint8_t data[6] = {0}; // Data for the debug message (max 6 bytes)

		// Copy error message to the data buffer
		if (errorMsg != NULL) {
			strncpy((char *)data, errorMsg, sizeof(data));
			// Ensure null termination
			data[sizeof(data) - 1] = '\0';
		}

		// Use canlib's helper function to build the debug message
		// Set priority to high and timestamp to 0 (since we can't reliably get timestamp in error
		// state)
		build_debug_raw_msg(PRIO_HIGH, 0, data, &msg);
		if (can_initialized) {
			stm32h7_can_send(&msg);
		}

		// scream a few times then attempt to reset.
		// delay for ~1sec without using systick-based delays (no hal_delay)
		volatile int dummy;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 50000000; j++) {
				dummy++;
			}
		}

		dummy++;

		// resetting is always a safe state even in midflight, as flightphase starts IDLE
		NVIC_SystemReset();
	}
}

// --- End Fatal Error Handler ---

w_status_t health_check_exec() {
	uint32_t status_bitfield = 0;

	status_bitfield |= check_watchdog_tasks();
	status_bitfield |= check_modules_status();

	// send status CAN msg
	can_msg_t msg = {0};

	if (0 == status_bitfield) {
		build_general_board_status_msg(PRIO_LOW, xTaskGetTickCount(), 0, &msg);
	} else {
		build_general_board_status_msg(PRIO_HIGH, xTaskGetTickCount(), status_bitfield, &msg);
	}

	if (can_handler_transmit(&msg) != W_SUCCESS) {
		log_text(0, "health_checks", "CAN send failure for status msg");
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
