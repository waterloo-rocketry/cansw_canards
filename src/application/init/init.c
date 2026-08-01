// Add these includes for hardware handles
#include "FreeRTOS.h"
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1 and hfdcan3
#include "i2c.h" // For hi2c2, hi2c4
#include "stm32h7xx_hal.h"
#include "task.h"
#include "usart.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/init/init.h"
#include "application/logger/log.h"
#include "application/navigator/navigator.h"
#include "application/power_handler/power_handler.h"
#include "application/sensor_handler/sensor_handler.h"
#include "application/telemetry/telemetry.h"
#include "drivers/MS5611/MS5611.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/ad_breakout_board/ad_breakout_board.h"
#include "drivers/adc/adc.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/iis2mdc/IIS2MDC.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"

// Maximum number of initialization retries before giving up
#define MAX_INIT_RETRIES 1

// Delay between initialization retries in milliseconds
#define INIT_RETRY_DELAY_MS 1000

static const uint32_t MOTOR_INIT_TIMEOUT_MS = 10 * 1000; // 10 seconds

// Initialize task handles to NULL
TaskHandle_t log_task_handle = NULL;
TaskHandle_t fsm_task_handle = NULL;
TaskHandle_t can_handler_handle_tx = NULL;
TaskHandle_t can_handler_handle_rx = NULL;
TaskHandle_t health_checks_task_handle = NULL;
TaskHandle_t movella_task_handle = NULL;
TaskHandle_t ms5611_task_handle = NULL;
TaskHandle_t ad_breakout_task_handle = NULL;
TaskHandle_t telem_task_handle = NULL;
TaskHandle_t init_task_handle = NULL;

// Task priorities
// TODO: set fsm priority
const uint32_t fsm_task_priority = configMAX_PRIORITIES - 1;
// prioritize not missing injectorvalveopen msg
// TODO: could dynamically reduce this priority after flight starts?
const uint32_t can_handler_rx_priority = 45;
// in general, prioritize consumers (estimator) over producers (imus) to avoid congestion
const uint32_t can_handler_tx_priority = 40;
// TODO: update when sure (based on old imu handler priority)
const uint32_t movella_task_priority = 20;
const uint32_t ms5611_task_priority = 18;
const uint32_t ad_breakout_task_priority = 20;
const uint32_t log_task_priority = 15;
// should be lowest prio above default task
const uint32_t health_checks_task_priority = 10;

bool done_sys_init = false;
const uint32_t telem_task_priority = 10; // TODO: decide telem task priority

// Motor calibration callback.
// Run this in init to avoid blocking other tasks for too long, so blinky will survive.
static w_status_t ak45_motor_calibration(const can_msg_t *msg) {
	can_actuator_id_t msg_id;
	can_actuator_state_t msg_state;

	if ((get_actuator_id(msg, &msg_id) != W_SUCCESS) ||
		(get_cmd_actuator_state(msg, &msg_state) != W_SUCCESS)) {
		log_text(1, LOG_LVL_WARN, "ak45", "invalid actuator data");
		return W_FAILURE;
	}
	// make sure it is the correct message
	if ((ACTUATOR_CANARD_MOTOR_CALIBRATION == msg_id) && (ACT_STATE_ON == msg_state)) {
		xTaskNotifyGive(init_task_handle);
	}
	// default return
	return W_SUCCESS;
}

static void system_init_task(void *arg) {
	// hotfix: allow time for .... stuff ?? ... before init.
	// without this, the uart DMA change made proc freeze upon power cycle.
	// probably because movella triggers before its ready
	vTaskDelay(500);

	// initialize timer first to make sure other modules can use it
	if (W_SUCCESS != timer_init()) {
		proc_handle_fatal_error("timerinit");
	}

	// INIT NON-CRITICAL MODULES; try to do logger first
	w_status_t non_crit_status = sd_card_init();
	non_crit_status |= log_init();
	if (non_crit_status != W_SUCCESS) {
		// Log non-critical initialization failure
		log_text(10, LOG_LVL_WARN, "init", "Non-crit init fail 0x%lx (log)", non_crit_status);
	}

	if (telemetry_init() != W_SUCCESS) {
		log_text(10, LOG_LVL_FATAL, "init", "crit init fail (telem).");
		proc_handle_fatal_error("sysinit");
	}

	if (ak45_driver_init(&hfdcan1, MOTOR_INIT_TIMEOUT_MS) != W_SUCCESS) {
		log_text(10, LOG_LVL_WARN, "init", "Non-crit init fail (motor)", non_crit_status);
	}

	w_status_t status = W_SUCCESS;

	// INIT REQUIRED MODULES
	status |= gpio_init();
	status |= i2c_init(I2C_BUS_1, &hi2c1, 0); // ST IMU
	status |= i2c_init(I2C_BUS_4, &hi2c4, 0); // ST MAG
	status |= i2c_init(I2C_BUS_5, &hi2c5, 0); // MS BARO
	status |= i2c_init(I2C_BUS_2, &hi2c2, 0); // AD BREAKOUT
	status |= uart_init(UART_MOVELLA, &huart3, 100);
	status |= adc_init(&hadc1, &hadc2, &hadc3);
	status |= navigator_init();
	status |= health_check_init();
	status |= movella_init();
	status |= flight_phase_init();
	status |= sensor_handler_init();
	status |= can_handler_init(&hfdcan3);
	status |= controller_init();
	status |= fsm_init();
	status |= lsm6dsv32x_init();
	status |= ms5611_init();
	status |= power_handler_init();
	status |= iis2mdc_init();
	//status |= adxl380_init();
	//status |= adxrs649_init();

	// cannot continue if any of the above fail
	if (status != W_SUCCESS) {
		// Log critical initialization failure - specific modules should have logged details
		log_text(10, LOG_LVL_FATAL, "init", "crit init fail (status: 0x%lx).", status);
		// critical err
		proc_handle_fatal_error("sysinit");
	}

	done_sys_init = true;

	// Create FreeRTOS tasks
	BaseType_t task_status = pdTRUE;

	task_status &= xTaskCreate(fsm_task,
							   "fsm",
							   8192, // TODO: set the correct size
							   NULL,
							   fsm_task_priority,
							   &fsm_task_handle);

	task_status &= xTaskCreate(health_check_task,
							   "health",
							   512,
							   NULL,
							   health_checks_task_priority,
							   &health_checks_task_handle);

	task_status &= xTaskCreate(can_handler_task_rx,
							   "can handler rx",
							   256,
							   NULL,
							   can_handler_rx_priority,
							   &can_handler_handle_rx);

	task_status &= xTaskCreate(can_handler_task_tx,
							   "can handler tx",
							   256,
							   NULL,
							   can_handler_tx_priority,
							   &can_handler_handle_tx);

	task_status &= xTaskCreate(
		movella_task, "movella", 2560, NULL, movella_task_priority, &movella_task_handle);

	task_status &= xTaskCreate(ms5611_task,
							   "ms5611",
							   512,
							   NULL,
							   ms5611_task_priority,
							   &ms5611_task_handle); // TODO: set the correct size

	task_status &= xTaskCreate(log_task, "logger", 512, NULL, log_task_priority, &log_task_handle);

	task_status &= xTaskCreate(ad_breakout_board_task,
							   "ad board task",
							   2560, // TODO: set when sure of size
							   NULL,
							   ad_breakout_task_priority,
							   &ad_breakout_task_handle);

	task_status &= xTaskCreate(
		telemetry_task, "telem module", 512, NULL, telem_task_priority, &telem_task_handle);

	if (task_status != pdTRUE) {
		// Log critical task creation failure
		log_text(10,
				 LOG_LVL_FATAL,
				 "SystemInit",
				 "CRITICAL: Failed to create one or more FreeRTOS tasks.");
		proc_handle_fatal_error("tasks");
	}
	log_text(10, LOG_LVL_INFO, "SystemInit", "All tasks created successfully.");
	gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 0); // indicate init done
	gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 0); // indicate init done
	gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_HIGH, 0); // indicate init done

	// grab the task handle
	init_task_handle = xTaskGetCurrentTaskHandle();

	// register motor calibration
	if (can_handler_act_cmd_register_callback(ACTUATOR_CANARD_MOTOR_CALIBRATION,
											  &ak45_motor_calibration) != W_SUCCESS) {
		log_text(0, LOG_LVL_FATAL, "ak45", "failed to add calibration callback");
		ak45_send_disable_cmd();
	}
	// its blinky now
	ak45_hard_stop_calibrate(&ak45_calibration_config);
	while (1) {
		gpio_toggle(GPIO_PIN_GREEN_LED, 1);
		vTaskDelay(50);

		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(0)) != 0) {
			// TODO: TEST ONLY
			ak45_hard_stop_calibrate(&ak45_calibration_config);
		}
	}
}

w_status_t init_tasks(void) {
	// create first task that will run system_init_task
	BaseType_t task_status =
		xTaskCreate(system_init_task, "SysInit", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
	return (task_status == pdTRUE) ? W_SUCCESS : W_FAILURE;
}
