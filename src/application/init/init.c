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
#include "drivers/MX25L25645G/MX25L25645G.h"
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
#include "common/benchmark/benchmark.h"
#include "drivers/MX25L25645G/flash_logger.h"

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
// in general, prioritize consumers (estimator) over producers (imus) to avoid
// congestion
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
BENCHMARK_FILE_INIT(8,6000000);

// Motor calibration callback.
// Run this in init to avoid blocking other tasks for too long, so blinky will
// survive.
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

	w_status_t non_crit_status = W_SUCCESS;
	w_status_t crit_status = W_SUCCESS;

	// init logging and telem
	non_crit_status |= sd_card_init();
	non_crit_status |= log_init();
	crit_status |= telemetry_init();

	// motor init
	non_crit_status |= ak45_driver_init(&hfdcan1, MOTOR_INIT_TIMEOUT_MS);

	// init hardware drivers
	crit_status |= gpio_init();
	non_crit_status |= i2c_init(I2C_BUS_1, &hi2c1, 0); // ST IMU
	non_crit_status |= i2c_init(I2C_BUS_4, &hi2c4, 0); // ST MAG
	non_crit_status |= i2c_init(I2C_BUS_5, &hi2c5, 0); // MS BARO
	non_crit_status |= i2c_init(I2C_BUS_2, &hi2c2, 0); // AD BREAKOUT
	non_crit_status |= uart_init(UART_MOVELLA, &huart3, 100);
	non_crit_status |= adc_init(&hadc1, &hadc2, &hadc3);

	// init application modules
	crit_status |= navigator_init();
	crit_status |= health_check_init();
	non_crit_status |= movella_init();
	crit_status |= flight_phase_init();
	crit_status |= sensor_handler_init();
	crit_status |= can_handler_init(&hfdcan3);
	non_crit_status |= controller_init();
	crit_status |= fsm_init();

	// init non-critical sensors and power handler
	non_crit_status |= lsm6dsv32x_init();
	non_crit_status |= ms5611_init();
	non_crit_status |= power_handler_init();
	non_crit_status |= iis2mdc_init();
	non_crit_status |= adxl380_init();
	non_crit_status |= adxrs649_init();

	if (non_crit_status != W_SUCCESS) {
		// Log non-critical initialization failure
		log_text(10, LOG_LVL_WARN, "init", "Non-crit init fail 0x%lx (log)", non_crit_status);
	}

	// cannot continue if any of the critical inits fail
	if (crit_status != W_SUCCESS) {
		// Log critical initialization failure - specific modules should have logged
		// details
		log_text(10, LOG_LVL_FATAL, "init", "crit init fail (status: 0x%lx).", crit_status);
		// critical err
		proc_handle_fatal_error("sysinit");
	}

	done_sys_init = true;
	flash_file_t p_file;
	const char *message = "hello, how long does it take to write 53 characters?";
	char read_buffer[100];
	benchmark_data_t benchmark_0;
	benchmark_data_t benchmark_1;
	benchmark_data_t benchmark_2;
	benchmark_data_t benchmark_3;
	benchmark_data_t benchmark_4;
	benchmark_data_t benchmark_5;
	benchmark_data_t benchmark_6;
	benchmark_data_t benchmark_7;
	int counter=0;
	char filename[13];

	DWT_init();
	SINGLE_BENCHMARK_START(0);
	flash_init();
	SINGLE_BENCHMARK_END(0);

	for (int k=0; k<100;k++){
		if (0 == k%10){
			snprintf(filename, 13, "%07d.bin", counter);
		}
		else{
			snprintf(filename, 13, "%08d.bin", counter);
		}
		SINGLE_BENCHMARK_START(1);
		flash_file_open(&p_file, filename, LFS_O_RDWR| LFS_O_CREAT);
		SINGLE_BENCHMARK_END(1);
		SINGLE_BENCHMARK_START(2);
		flash_file_write(&p_file, message, 60, 5, 1);
		SINGLE_BENCHMARK_END(2);
		SINGLE_BENCHMARK_START(6);
		flash_file_sync(&p_file);
		SINGLE_BENCHMARK_END(6);
		SINGLE_BENCHMARK_START(3);
		flash_file_read(&p_file, read_buffer, 30, 5, 0, 30);
		SINGLE_BENCHMARK_END(3);
		SINGLE_BENCHMARK_START(4);
		file_flash_close(&p_file);
		SINGLE_BENCHMARK_END(4);
		counter+=1;
	}
	GET_BENCHMARK_STAT(0, &benchmark_0);
	GET_BENCHMARK_STAT(1, &benchmark_1);
	GET_BENCHMARK_STAT(2, &benchmark_2);
	GET_BENCHMARK_STAT(3, &benchmark_3);
	GET_BENCHMARK_STAT(4, &benchmark_4);
	GET_BENCHMARK_STAT(6, &benchmark_6);

	uint32_t init_time= BENCH_CYCLES_TO_US(benchmark_0.last);
	uint32_t open_time= BENCH_CYCLES_TO_US(benchmark_1.sum / benchmark_1.count);
	uint32_t write_time= BENCH_CYCLES_TO_US(benchmark_2.sum / benchmark_2.count);
	uint32_t read_time= BENCH_CYCLES_TO_US(benchmark_3.sum / benchmark_3.count);
	uint32_t close_time= BENCH_CYCLES_TO_US(benchmark_4.sum / benchmark_4.count);
	uint32_t sync_time= BENCH_CYCLES_TO_US(benchmark_6.sum/benchmark_6.count);

	SINGLE_BENCHMARK_START(5);
	flash_clear_logs();
	SINGLE_BENCHMARK_END(5);
	GET_BENCHMARK_STAT(5, &benchmark_5);
	uint32_t clear_logs_time= BENCH_CYCLES_TO_US(benchmark_5.last);
	SINGLE_BENCHMARK_START(7);
	flash_reformat(false);
	SINGLE_BENCHMARK_END(7);
	GET_BENCHMARK_STAT(7, &benchmark_7);
	uint32_t reformat_time= BENCH_CYCLES_TO_US(benchmark_7.last);
	uint32_t tot= init_time + open_time + write_time + sync_time + read_time + close_time + clear_logs_time + reformat_time;


	// Create FreeRTOS tasks
	BaseType_t task_status = pdTRUE;

	task_status &= xTaskCreate(fsm_task, "fsm",
	                           6144, // TODO: set the correct size
	                           NULL, fsm_task_priority, &fsm_task_handle);

	task_status &= xTaskCreate(health_check_task, "health", 384, NULL, health_checks_task_priority,
	                           &health_checks_task_handle);

	task_status &= xTaskCreate(can_handler_task_rx, "can handler rx", 256, NULL,
	                           can_handler_rx_priority, &can_handler_handle_rx);

	task_status &= xTaskCreate(can_handler_task_tx, "can handler tx", 256, NULL,
	                           can_handler_tx_priority, &can_handler_handle_tx);

	task_status &= xTaskCreate(movella_task, "movella", 512, NULL, movella_task_priority,
	                           &movella_task_handle);

	task_status &=
		xTaskCreate(ms5611_task, "ms5611", 256, NULL, ms5611_task_priority, &ms5611_task_handle);
	// TODO: set the correct size

	task_status &= xTaskCreate(log_task, "logger", 512, NULL, log_task_priority, &log_task_handle);

	task_status &= xTaskCreate(ad_breakout_board_task, "ad board task",
	                           256, // TODO: set when sure of size
	                           NULL, ad_breakout_task_priority, &ad_breakout_task_handle);

	task_status &= xTaskCreate(telemetry_task, "telem module", 336, NULL, telem_task_priority,
	                           &telem_task_handle);

	if (task_status != pdTRUE) {
		// Log critical task creation failure
		log_text(10, LOG_LVL_FATAL, "SystemInit",
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
		log_text(0, LOG_LVL_FATAL, "SystemInit", "failed to add calibration callback");
		ak45_send_disable_cmd();
	}
	// its blinky now
	while (1) {
		gpio_toggle(GPIO_PIN_GREEN_LED, 1);
		vTaskDelay(500);

		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(0)) != 0) {
			// TODO: TEST ONLY
			if (ak45_hard_stop_calibrate(&ak45_calibration_config) != W_SUCCESS) {
				log_text(0, LOG_LVL_FATAL, "SystemInit", "failed motor calibration");
			}
		}
	}
}

w_status_t init_tasks(void) {
	// create first task that will run system_init_task
	BaseType_t task_status =
		xTaskCreate(system_init_task, "SysInit", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
	return (task_status == pdTRUE) ? W_SUCCESS : W_FAILURE;
}
