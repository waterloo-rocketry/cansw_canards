#include "application/init/init.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/ekf.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"
#include "stm32h7xx_hal.h"
// Add these includes for hardware handles
#include "FreeRTOS.h"
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1
#include "i2c.h" // For hi2c2, hi2c4
#include "task.h"
#include "usart.h"
#include "sdmmc.h"

// Maximum number of initialization retries before giving up
#define MAX_INIT_RETRIES 1

// Delay between initialization retries in milliseconds
#define INIT_RETRY_DELAY_MS 1000

// Initialize task handles to NULL
TaskHandle_t log_task_handle = NULL;
TaskHandle_t estimator_task_handle = NULL;
TaskHandle_t can_handler_handle_tx = NULL;
TaskHandle_t can_handler_handle_rx = NULL;
TaskHandle_t health_checks_task_handle = NULL;
TaskHandle_t controller_task_handle = NULL;
TaskHandle_t flight_phase_task_handle = NULL;
TaskHandle_t imu_handler_task_handle = NULL;
TaskHandle_t movella_task_handle = NULL;

// Task priorities
// flight phase must have highest priority to preempt everything else
const uint32_t flight_phase_task_priority = configMAX_PRIORITIES - 1;
// prioritize not missing injectorvalveopen msg
// TODO: could dynamically reduce this priority after flight starts?
const uint32_t can_handler_rx_priority = 45;
// in general, prioritize consumers (estimator) over producers (imus) to avoid congestion
const uint32_t can_handler_tx_priority = 40;
const uint32_t controller_task_priority = 30;
const uint32_t estimator_task_priority = 25;
const uint32_t imu_handler_task_priority = 20;
const uint32_t movella_task_priority = 20;
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
	if (non_crit_status != W_SUCCESS) {
		// Log non-critical initialization failure
		log_text(10, "init", "Non-crit init fail 0x%lx", non_crit_status);
	}

	w_status_t status = W_SUCCESS;

	// INIT REQUIRED MODULES
	status |= gpio_init();
	status |= i2c_init(I2C_BUS_1, &hi2c1, 0); // ST IMU
	status |= i2c_init(I2C_BUS_5, &hi2c5, 0); // MS BARO
	status |= i2c_init(I2C_BUS_2, &hi2c2, 0); // AD BREAKOUT
	// status |= uart_init(UART_DEBUG_SERIAL, &huart4, 100);
	status |= uart_init(UART_MOVELLA, &huart3, 100);
	// status |= adc_init(&hadc1);
	// status |= estimator_init();
	// status |= health_check_init();
	status |= movella_init();
	status |= flight_phase_init();
	// status |= imu_handler_init();
	status |= can_handler_init(&hfdcan3);
	// status |= controller_init;
	// status |= ekf_init;

	// cannot continue if any of the above fail
	if (status != W_SUCCESS) {
		// Log critical initialization failure - specific modules should have logged details
		log_text(10, "init", "crit init fail (status: 0x%lx).", status);
		// critical err
		proc_handle_fatal_error("sysinit");
	}

	// Create FreeRTOS tasks
	BaseType_t task_status = pdTRUE;

	task_status &= xTaskCreate(flight_phase_task,
							   "flight phase",
							   256,
							   NULL,
							   flight_phase_task_priority,
							   &flight_phase_task_handle);

	// task_status &= xTaskCreate(health_check_task,
	//     "health",
	//     512,
	//     NULL,
	//     health_checks_task_priority,
	//     &health_checks_task_handle);

	// task_status &= xTaskCreate(imu_handler_task,
	//     "imu handler",
	//     512,
	//     NULL,
	//     imu_handler_task_priority,
	//     &imu_handler_task_handle);

	task_status &= xTaskCreate(can_handler_task_rx,
							   "can handler rx",
							   256,
							   NULL,
							   can_handler_rx_priority,
							   &can_handler_handle_rx);

	// task_status &= xTaskCreate(can_handler_task_tx,
	//     "can handler tx",
	//     256,
	//     NULL,
	//     can_handler_tx_priority,
	//     &can_handler_handle_tx);

	task_status &= xTaskCreate(
		movella_task, "movella", 2560, NULL, movella_task_priority, &movella_task_handle);

	task_status &= xTaskCreate(log_task, "logger", 512, NULL, log_task_priority, &log_task_handle);

	// task_status &= xTaskCreate(controller_task,
	//     "controller",
	//     512,
	//     NULL,
	//     controller_task_priority,
	//     &controller_task_handle);

	// task_status &= xTaskCreate(
	//     estimator_task, "estimator", 8192, NULL, estimator_task_priority,
	//     &estimator_task_handle);

	if (task_status != pdTRUE) {
		// Log critical task creation failure
		log_text(10, "SystemInit", "CRITICAL: Failed to create one or more FreeRTOS tasks.");
		proc_handle_fatal_error("tasks");
	}
	log_text(10, "SystemInit", "All tasks created successfully.");

	// test SD card time !!!! :(
	// HAL_StatusTypeDef sd_status  = HAL_SD_Init(&hsd2); 
	uint32_t err = HAL_SD_GetError(&hsd2);
	HAL_SD_StateTypeDef sd_state = HAL_SD_GetState(&hsd2);
	HAL_SD_CardStateTypeDef sdcard_state = HAL_SD_GetCardState(&hsd2);

	if (err != 0 && sd_state != 0 && sdcard_state != 0) {
		vTaskDelay(500);
	}
  	HAL_SD_CardInfoTypeDef info;
	HAL_StatusTypeDef sd_status  = HAL_SD_GetCardInfo(&hsd2, &info);
	err = HAL_SD_GetError(&hsd2);
	sd_state = HAL_SD_GetState(&hsd2);
	sdcard_state = HAL_SD_GetCardState(&hsd2);
	if (HAL_OK != sd_status && err != 0 && sd_state != 0 && sdcard_state != 0) {
		vTaskDelay(500);
	}


  	uint8_t rx[512];
	uint32_t test_lba = 1024;
	sd_status = HAL_SD_ReadBlocks(&hsd2, rx, test_lba, 1, 1000);
  	vTaskDelay(1000);
	sdcard_state = HAL_SD_GetCardState(&hsd2);
	if (HAL_OK != sd_status && err != 0 && sd_state != 0 && sdcard_state != 0) {
		vTaskDelay(500);
	}



	// its blinky now
	while (1) {
		gpio_toggle(GPIO_PIN_RED_LED, 1);
		vTaskDelay(500);
		gpio_toggle(GPIO_PIN_GREEN_LED, 1);
		vTaskDelay(500);
		gpio_toggle(GPIO_PIN_BLUE_LED, 1);
		vTaskDelay(500);
	}
}

w_status_t init_tasks(void) {
	// create first task that will run system_init_task
	BaseType_t task_status =
		xTaskCreate(system_init_task, "SysInit", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
	return (task_status == pdTRUE) ? W_SUCCESS : W_FAILURE;
}
