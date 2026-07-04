// Add these includes for hardware handles
#include "FreeRTOS.h"
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1 and hfdcan3
#include "i2c.h" // For hi2c2, hi2c4
#include "stm32h7xx_hal.h"
#include "task.h"
#include "usart.h"

#include "canlib.h"
#include "application/power_handler/power_handler.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/init/init.h"
#include "application/logger/log.h"
#include "application/navigator/navigator.h"
#include "application/sensor_handler/sensor_handler.h"
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
	non_crit_status |= ak45_driver_init(&hfdcan1, MOTOR_INIT_TIMEOUT_MS);
	if (non_crit_status != W_SUCCESS) {
		// Log non-critical initialization failure
		log_text(10, LOG_LVL_WARN, "init", "Non-crit init fail 0x%lx", non_crit_status);
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
	// status |= adxl380_init();
	status |= lsm6dsv32x_init();
	// status |= adxrs649_init();
	status |= ms5611_init();
	status |= iis2mdc_init();
	status |= power_handler_init();
	// status |= ekf_init();

	// cannot continue if any of the above fail
	if (status != W_SUCCESS) {
		// Log critical initialization failure - specific modules should have logged details
		log_text(10, LOG_LVL_FATAL, "init", "crit init fail (status: 0x%lx).", status);
		// critical err
		proc_handle_fatal_error("sysinit");
	}

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

	if (task_status != pdTRUE) {
		// Log critical task creation failure
		log_text(10,
				 LOG_LVL_FATAL,
				 "SystemInit",
				 "CRITICAL: Failed to create one or more FreeRTOS tasks.");
		proc_handle_fatal_error("tasks");
	}
	log_text(10, LOG_LVL_INFO, "SystemInit", "All tasks created successfully.");

	// its blinky now
	TickType_t last_wake_time = xTaskGetTickCount();

	gpio_toggle(GPIO_PIN_RED_LED, 1);
	gpio_toggle(GPIO_PIN_GREEN_LED, 1);
	gpio_toggle(GPIO_PIN_BLUE_LED, 1);
	can_msg_t msg = {0};
	uint32_t timestamp_ms = 0;
	uint16_t i = 0;

	while (1) {
		(void)timer_get_ms(&timestamp_ms);
		uint16_t ts = (uint16_t)timestamp_ms;

		// 10 Hz
		build_3d_analog_sensor_16bit_msg(
			PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_NAV_ORIENTATION_QUAT_QX_QY_QZ, 0, 0, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_3d_analog_sensor_16bit_msg(
			PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_NAV_ORIENTATION_QUAT_QW_ALT_VARNORM, 0, 0, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_CTRL_CMD_ANGLE, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_CTRL_CMD_ANGLE, 0, &msg);
		(void)can_handler_transmit(&msg); // #2
		vTaskDelay(1);

		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_CTRL_COEFF_LIFT, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_SERVO_ANGLE, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_3d_analog_sensor_16bit_msg(
			PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_LSM6DSV32X_ACCEL, 0, 0, 0, &msg);
		(void)can_handler_transmit(&msg);
		vTaskDelay(1);

		build_3d_analog_sensor_16bit_msg(
			PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_LSM6DSV32X_GYRO, 0, 0, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_3d_analog_sensor_16bit_msg(
			PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_ADXL380_ACCEL, 0, 0, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_analog_sensor_32bit_msg(PRIO_LOW, ts, SENSOR_CANARD_ADXRS649_GYRO, 0, &msg);
		(void)can_handler_transmit(&msg);

		vTaskDelay(1);
		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_SERVO_ANGLE, 0, &msg);
		(void)can_handler_transmit(&msg); // duplicate in list
		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_SERVO_CURR, 0, &msg);
		(void)can_handler_transmit(&msg);
		build_analog_sensor_16bit_msg(PRIO_LOW, ts, SENSOR_CANARD_SERVO_TEMP, 0, &msg);
		(void)can_handler_transmit(&msg);

		vTaskDelay(1);

		if (i % 2 == 0) { // 5 Hz
			build_2d_analog_sensor_24bit_msg(
				PRIO_LOW, ts, DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_X, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_2d_analog_sensor_24bit_msg(
				PRIO_LOW, ts, DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_Y, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_2d_analog_sensor_24bit_msg(
				PRIO_LOW, ts, DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_Z, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_2d_analog_sensor_24bit_msg(
				PRIO_LOW, ts, DEM_2D_SENSOR_CANARD_MS5611_BARO_TEMP, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
		}

		vTaskDelay(1);

		if (i % 5 == 0) { // 2 Hz
			build_3d_analog_sensor_16bit_msg(
				PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_IIS2MDC_MAG, 0, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_3d_analog_sensor_16bit_msg(
				PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_MTI630_ACCEL, 0, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_3d_analog_sensor_16bit_msg(
				PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_MTI630_GYRO, 0, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_3d_analog_sensor_16bit_msg(
				PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_MTI630_MAG, 0, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_analog_sensor_32bit_msg(PRIO_LOW, ts, SENSOR_CANARD_MTI630_BARO_0, 0, &msg);
			(void)can_handler_transmit(&msg);
		}

		vTaskDelay(1);

		if (i % 10 == 0) { // 1 Hz
			build_2d_analog_sensor_24bit_msg(
				PRIO_LOW, ts, DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_X, 0, 0, &msg);
			(void)can_handler_transmit(&msg); // #2
			build_2d_analog_sensor_24bit_msg(
				PRIO_LOW, ts, DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_X, 0, 0, &msg);
			(void)can_handler_transmit(&msg); // #3
			build_3d_analog_sensor_16bit_msg(
				PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_MTI630_EST_ANGLE_VEL, 0, 0, 0, &msg);
			(void)can_handler_transmit(&msg);
			build_3d_analog_sensor_16bit_msg(
				PRIO_LOW, ts, DEM_3D_SENSOR_CANARD_MTI630_EST_VEL, 0, 0, 0, &msg);
			(void)can_handler_transmit(&msg);

			gpio_toggle(GPIO_PIN_RED_LED, 1);
			gpio_toggle(GPIO_PIN_GREEN_LED, 1);
			gpio_toggle(GPIO_PIN_BLUE_LED, 1);
		}

		vTaskDelay(1);

		i = (i + 1) % 10;
		vTaskDelayUntil(&last_wake_time, 100);
	}
}

w_status_t init_tasks(void) {
	// create first task that will run system_init_task
	BaseType_t task_status =
		xTaskCreate(system_init_task, "SysInit", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
	return (task_status == pdTRUE) ? W_SUCCESS : W_FAILURE;
}
