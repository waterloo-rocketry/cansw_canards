// Add these includes for hardware handles
#include "FreeRTOS.h"
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1
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
#include "third_party/codegen/controller_codegen_entry.h"
#include "third_party/codegen/GNC_codegen_initialize.h"
#include "third_party/codegen/GNC_codegen_terminate.h"
#include "third_party/codegen/GNC_codegen_types.h"
#include "third_party/codegen/controller_codegen_entry.h"
#include "third_party/codegen/navigation_codegen_entry.h"
#include "third_party/codegen/rt_nonfinite.h"

/* Function Declarations */
// static void argInit_2x1_real_T(double result[2]);

static void argInit_3x1_real_T(double result[3]);

static bool argInit_boolean_T(void);

static double argInit_real_T(void);

static struct0_T argInit_struct0_T(void);

static struct1_T argInit_struct1_T(void);

/* Function Definitions */
/*
 * Arguments    : double result[2]
 * Return Type  : void
 */
// static void argInit_2x1_real_T(double result[2])
// {
//   int idx0;
//   /* Loop over the array to initialize each element. */
//   for (idx0 = 0; idx0 < 2; idx0++) {
//     /* Set the value of the array element.
// Change this value to the value that the application requires. */
//     result[idx0] = argInit_real_T();
//   }
// }

/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_3x1_real_T(double result[3])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : bool
 */
static bool argInit_boolean_T(void)
{
  return false;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : struct0_T
 */
static struct0_T argInit_struct0_T(void)
{
  struct0_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_3x1_real_T(result.meas);
  result.status = argInit_boolean_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct1_T
 */
static struct1_T argInit_struct1_T(void)
{
  struct1_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.meas = argInit_real_T();
  result.status = argInit_boolean_T();
  return result;
}

// Maximum number of initialization retries before giving up
#define MAX_INIT_RETRIES 1

// Delay between initialization retries in milliseconds
#define INIT_RETRY_DELAY_MS 1000

// Initialize task handles to NULL
TaskHandle_t log_task_handle = NULL;
TaskHandle_t fsm_task_handle = NULL;
TaskHandle_t can_handler_handle_tx = NULL;
TaskHandle_t can_handler_handle_rx = NULL;
TaskHandle_t health_checks_task_handle = NULL;
TaskHandle_t movella_task_handle = NULL;

// Task priorities
// TODO: set fsm priority
const uint32_t fsm_task_priority = configMAX_PRIORITIES - 1;
// prioritize not missing injectorvalveopen msg
// TODO: could dynamically reduce this priority after flight starts?
const uint32_t can_handler_rx_priority = 45;
// in general, prioritize consumers (estimator) over producers (imus) to avoid congestion
const uint32_t can_handler_tx_priority = 40;
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
	w_status_t non_crit_status = W_SUCCESS;//sd_card_init();
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
	status |= uart_init(UART_MOVELLA, &huart3, 100);
	// status |= adc_init(&hadc1);
	status |= estimator_init();
	// status |= health_check_init();
	status |= movella_init();
	status |= flight_phase_init();
	status |= imu_handler_init();
	status |= can_handler_init(&hfdcan3);
	status |= controller_init();
	status |= fsm_init();
	// status |= ekf_init();

	// cannot continue if any of the above fail
	if (status != W_SUCCESS) {
		// Log critical initialization failure - specific modules should have logged details
		log_text(10, "init", "crit init fail (status: 0x%lx).", status);
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

	// task_status &= xTaskCreate(health_check_task,
	//     "health",
	//     512,
	//     NULL,
	//     health_checks_task_priority,
	//     &health_checks_task_handle);

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

	task_status &= xTaskCreate(log_task, "logger", 512, NULL, log_task_priority, &log_task_handle);

	if (task_status != pdTRUE) {
		// Log critical task creation failure
		log_text(10, "SystemInit", "CRITICAL: Failed to create one or more FreeRTOS tasks.");
		proc_handle_fatal_error("tasks");
	}
	log_text(10, "SystemInit", "All tasks created successfully.");

	// its blinky now
	while (1) {
		gpio_toggle(GPIO_PIN_RED_LED, 1);
		vTaskDelay(500);
		gpio_toggle(GPIO_PIN_GREEN_LED, 1);
		vTaskDelay(500);
		gpio_toggle(GPIO_PIN_BLUE_LED, 1);
		vTaskDelay(500);

          struct0_T board_accel_tmp;
  struct1_T b_r;
  struct1_T r1;
  struct2_T state;
  struct3_T airdata;
  double roll_state[2];
  double cov_norm;
  /* Initialize function 'navigation_codegen_entry' input arguments. */
  /* Initialize function input argument 'board_accel'. */
  board_accel_tmp = argInit_struct0_T();
  /* Initialize function input argument 'board_gyro'. */
  /* Initialize function input argument 'mti_accel'. */
  /* Initialize function input argument 'mti_gyro'. */
  /* Initialize function input argument 'ad_accel'. */
  /* Initialize function input argument 'ad_gyro'. */
  /* Initialize function input argument 'board_baro'. */
  /* Initialize function input argument 'board_mag'. */
  /* Initialize function input argument 'mti_baro'. */
  /* Initialize function input argument 'mti_mag'. */
  /* Call the entry-point 'navigation_codegen_entry'. */
  b_r = argInit_struct1_T();
  r1 = argInit_struct1_T();
  volatile uint32_t timestart = HAL_GetTick();

  navigation_codegen_entry(argInit_real_T(), argInit_boolean_T(),
                           &board_accel_tmp, &board_accel_tmp, &board_accel_tmp,
                           &board_accel_tmp, &board_accel_tmp, &board_accel_tmp,
                           &b_r, &board_accel_tmp, &r1, &board_accel_tmp,
                           &state, &cov_norm, &airdata, roll_state);

    volatile uint32_t timeend = HAL_GetTick();

    volatile uint32_t timediff = timeend - timestart;
    (void)timediff; // Suppress unused variable warning
	}
}

w_status_t init_tasks(void) {
	// create first task that will run system_init_task
	BaseType_t task_status =
		xTaskCreate(system_init_task, "SysInit", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
	return (task_status == pdTRUE) ? W_SUCCESS : W_FAILURE;
}
