#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/can_handler/can_handler.h"
#include "application/estimator/estimator.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

#include "canlib.h"

// Period of IMU sampling in milliseconds
// slightly slower than 200 hz to always receive encoder which can be >5ms
#define IMU_SAMPLING_PERIOD_MS 6

// Timeout values for freshness check (in milliseconds)
#define GYRO_FRESHNESS_TIMEOUT_MS 5
#define MAG_FRESHNESS_TIMEOUT_MS 10
#define ACCEL_FRESHNESS_TIMEOUT_MS 5
#define BARO_FRESHNESS_TIMEOUT_MS 25
#define ERROR_THRESHOLD 10
#define MIN_SUCCESS_RATE 90.0f

// Rate limit CAN tx: only send data at 10Hz, every 100ms
#define IMU_HANDLER_CAN_TX_PERIOD_MS 100
#define IMU_HANDLER_CAN_TX_RATE (IMU_HANDLER_CAN_TX_PERIOD_MS / IMU_SAMPLING_PERIOD_MS)

// correct orientation from finn irl, may 4 2025
// S1 (movella)
static const matrix3d_t g_movella_upd_mat = {
	.array = {{0, 0, 1.000000000}, {1.0000000, 0, 0}, {0, 1.0000000000, 0}}};

// TODO provide orientation correction for rest of sensors

// Module state tracking
typedef struct {
	bool initialized;
	uint32_t sample_count;
	uint32_t error_count;

	// Per-IMU stats
	struct {
		uint32_t success_count;
		uint32_t failure_count;
	} pololu_stats, movella_stats;
} imu_handler_state_t;

static imu_handler_state_t imu_handler_state = {0};

// TODO: potentially implement for data transform log data to can

// TODO: implement read data from board, movella, and breakout board

/**
 * @brief Read data from the Movella MTi-630 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
// TODO: update to new standard
static w_status_t read_movella_imu(estimator_mti_meas_t *imu_data) {
	w_status_t status = W_SUCCESS;

	// Read all data from Movella in one call
	movella_data_t movella_data = {0}; // Initialize to zero

	// TODO: get data from movella once that is brought up
	//  status = movella_get_data(&movella_data, 1);

	if (W_SUCCESS == status) {
		// Copy data from Movella
		// Apply orientation correction
		// TODO: update all of the units to SI units as described in estimator.h
		imu_data->mti_accel = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.acc);
		imu_data->mti_gyro = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.gyr);
		imu_data->mti_mag = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.mag);

		imu_data->mti_baro = movella_data.pres;
		imu_data->is_dead = movella_data.is_dead;
		imu_handler_state.movella_stats.success_count++;
	} else {
		// Set is_dead flag to indicate IMU failure
		imu_data->is_dead = true;
		imu_handler_state.movella_stats.failure_count++;
	}

	return status;
}

/**
 * @brief Initialize the IMU handler module
 * @note This function is called before the scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void) {
	// TODO: poll all imus to make sure theyre initialized alr or smth

	// Set initialized flag directly here instead of calling initialize_all_imus()
	imu_handler_state.initialized = true;

	log_text(10, "IMUHandler", "IMU Handler Initialized.");
	return W_SUCCESS;
}

/**
 * @brief Execute one iteration of the IMU handler processing
 * Reads data from all IMUs and updates the estimator
 * @param loop_count Number of loops run, for CAN send rate limiting
 * @note This function is non-static to allow exposed to unit tests
 * @return Status of the execution
 */
w_status_t imu_handler_run(uint32_t loop_count) {
	estimator_all_imus_input_t imu_data = {0};
	uint32_t current_time_ms;
	w_status_t status = W_SUCCESS;

	// Get current timestamp
	if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
		current_time_ms = 0;
	}

	// TODO: get data from board, movella, and breakout board

	// Done just to use the movella imu function to make sure we can complile
	estimator_mti_meas_t movella_data = {0};
	status |= read_movella_imu(&movella_data);

	// TODO: logging for non-breakout board sensors

	// TODO: provide update for estimator

	// Return overall status
	return status;
}

/**
 * @brief IMU handler task function for FreeRTOS
 * @note This task will be created during system initialization
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument) {
	(void)argument; // Unused parameter

	// Variables for precise timing control
	TickType_t last_wake_time = xTaskGetTickCount();
	const TickType_t frequency = pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS);

	// track loop count for CAN tx rate limiting
	uint32_t loop_count = 0;

	// Main task loop
	log_text(10, "IMUHandlerTask", "IMU Handler task started.");
	while (1) {
		w_status_t run_status = imu_handler_run(loop_count++);
		if (W_SUCCESS != run_status) {
			// Log or handle run failures if needed
			imu_handler_state.error_count++;
			log_text(1, "IMUHandlerTask", "run failed (status: %d).", run_status);
		}

		// Wait for next sampling period with precise timing
		vTaskDelayUntil(&last_wake_time, frequency);
	}
}

uint32_t imu_handler_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log sampling statistics
	log_text(0,
			 "imu_handler",
			 "%s Sampling -Total: %lu, Errors: %lu",
			 imu_handler_state.initialized ? "INIT" : "NOT INIT",
			 imu_handler_state.sample_count,
			 imu_handler_state.error_count);

	// Log IMU statistics
	log_text(0,
			 "imu_handler",
			 "Polulu Success %lu, Failure %lu Movella - Success %lu, Failure %lu",
			 imu_handler_state.pololu_stats.success_count,
			 imu_handler_state.pololu_stats.failure_count,
			 imu_handler_state.movella_stats.success_count,
			 imu_handler_state.movella_stats.failure_count);

	return status_bitfield;
}
