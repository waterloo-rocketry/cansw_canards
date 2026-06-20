// Add these includes for hardware handles
#include "FreeRTOS.h"
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1 and hfdcan3
#include "i2c.h" // For hi2c2, hi2c4
#include "stm32h7xx_hal.h"
#include "task.h"
#include "usart.h"

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/ekf.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/imu_handler/imu_handler.h"
#include "application/init/init.h"
#include "application/logger/log.h"
#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/ad_breakout_board/ad_breakout_board.h"
#include "drivers/adc/adc.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"

// Maximum number of initialization retries before giving up
#define MAX_INIT_RETRIES 1

#define NUM_CMDS 14

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
const uint32_t ad_breakout_task_priority = 20;
const uint32_t log_task_priority = 1;
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
		log_text(10, "init", "Non-crit init fail 0x%lx", non_crit_status);
	}

	w_status_t aastatus = W_SUCCESS;

	// INIT REQUIRED MODULES
	aastatus |= gpio_init();
	// status |= i2c_init(I2C_BUS_1, &hi2c1, 0); // ST IMU
	// status |= i2c_init(I2C_BUS_5, &hi2c5, 0); // MS BARO
	// status |= i2c_init(I2C_BUS_2, &hi2c2, 0); // AD BREAKOUT
	// // status |= uart_init(UART_MOVELLA, &huart3, 100);
	// status |= adc_init(&hadc1, &hadc2, &hadc3);
	// status |= estimator_init();
	// status |= health_check_init();
	// status |= movella_init();
	// status |= flight_phase_init();
	// status |= imu_handler_init();
	// status |= can_handler_init(&hfdc?an3);
	// status |= controller_init();
	// status |= fsm_init();
	// status |= adxl380_init();
	// status |= lsm6dsv32x_init();
	// status |= adxrs649_init();
	// status |= iis2mdc_init();
	// status |= ekf_init();

	// cannot continue if any of the above fail
	if (aastatus != W_SUCCESS) {
		// Log critical initialization failure - specific modules should have logged details
		log_text(10, "init", "crit init fail (status: 0x%lx).", aastatus);
		// critical err
		proc_handle_fatal_error("sysinit");
	}

	// Create FreeRTOS tasks
	BaseType_t task_status = pdTRUE;

	// task_status &= xTaskCreate(fsm_task,
	// 						   "fsm",
	// 						   8192, // TODO: set the correct size
	// 						   NULL,
	// 						   fsm_task_priority,
	// 						   &fsm_task_handle);

	// task_status &= xTaskCreate(health_check_task,
	//     "health",
	//     512,
	//     NULL,
	//     health_checks_task_priority,
	//     &health_checks_task_handle);

	// task_status &= xTaskCreate(can_handler_task_rx,
	// 						   "can handler rx",
	// 						   256,
	// 						   NULL,
	// 						   can_handler_rx_priority,
	// 						   &can_handler_handle_rx);

	// task_status &= xTaskCreate(can_handler_task_tx,
	// 						   "can handler tx",
	// 						   256,
	// 						   NULL,
	// 						   can_handler_tx_priority,
	// 						   &can_handler_handle_tx);

	// task_status &= xTaskCreate(
	// 	movella_task, "movella", 2560, NULL, movella_task_priority, &movella_task_handle);

	task_status &= xTaskCreate(log_task, "logger", 512, NULL, log_task_priority, &log_task_handle);

	// task_status &= xTaskCreate(ad_breakout_board_task,
	// 						   "ad board task",
	// 						   2560, // TODO: set when sure of size
	// 						   NULL,
	// 						   ad_breakout_task_priority,
	// 						   &ad_breakout_task_handle);

	if (task_status != pdTRUE) {
		// Log critical task creation failure
		log_text(10, "SystemInit", "CRITICAL: Failed to create one or more FreeRTOS tasks.");
		proc_handle_fatal_error("tasks");
	}
	log_text(10, "SystemInit", "All tasks created successfully.");


    // vars for vtaskdelay
    TickType_t last_tick_count = xTaskGetTickCount();
            gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 1);
            gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 1);
            gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_HIGH, 1);

    // ------------ MOTOR TEST

        // start time is a few sec after startup
    uint32_t start_time_ms = 4000;

    // reset motor to 0
    w_status_t status = ak45_send_position_cmd(0);
    if (status != W_SUCCESS) {
        log_text(10, "test", "ak45_send_position_cmd(0) fail: %d", status);
    }


    // command sequence, run at intervals of 1sec
    float32_t cmd_angles[NUM_CMDS] = {
        40, 0, 20, 0, 10, 0, 5, 0, 3, 0, 2, 0, 1, 0
    };

    uint32_t last_cmd_time_ms = start_time_ms;
    uint32_t cmd_index = 0;
    

    // expected leds: toggle green per second for each cmd, then blinky red when done
	while (1) {
        if (cmd_index >= NUM_CMDS) {
            // done with test, just blink red
            gpio_toggle(GPIO_PIN_RED_LED, 1);
            gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_HIGH, 1);
            vTaskDelay(500);
        } else {
            uint32_t current_time_ms;
            timer_get_ms(&current_time_ms);
    
            // if 1 sec has passed since last cmd, send the next cmd in the sequence
            if (current_time_ms >= (last_cmd_time_ms + 1000)) {
                gpio_toggle(GPIO_PIN_GREEN_LED, 1);

                timer_get_ms(&last_cmd_time_ms);
    
                status = ak45_send_position_cmd(cmd_angles[cmd_index++]);
                 // log the result of the command
                if (status != W_SUCCESS) {
                    log_text(10, "t", "%fdeg fail: %d", cmd_angles[cmd_index - 1], status);
                } else {
                    log_text(10, "t", "%fdeg", cmd_angles[cmd_index - 1]);
                }
            }
        }

        // log encoder feedback
        ak45_feedback_t fb;
        status = ak45_get_latest_feedback(&fb);
        if (status == W_SUCCESS) {
            log_text(10, "t", "%fdeg, %ferpm, %fA, err %d",
                fb.position_deg, fb.speed_erpm, fb.current_a, fb.fault_code);
        } else {
            log_text(10, "t", "fb fail: %d", status);
        }

        // 1ms
        vTaskDelayUntil(&last_tick_count, 1);
	}
}

w_status_t init_tasks(void) {
	// create first task that will run system_init_task
	BaseType_t task_status =
		xTaskCreate(system_init_task, "SysInit", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
	return (task_status == pdTRUE) ? W_SUCCESS : W_FAILURE;
}
