
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/ad_breakout_board/ad_breakout_board.h"
#include "drivers/timer/timer.h"
#include "rocketlib/include/common.h"

// Units are as follows: m/s^2, rad/s, Pa, gauss.
typedef struct {
	float64_t meas;
	uint32_t timestamp_ms;
	w_status_t latest_status;
} ad_gyro_meas_t;

typedef struct {
	vector3d_t meas;
	uint32_t timestamp_ms;
	w_status_t latest_status;
} ad_accel_meas_t;

typedef enum {
	AD_WRITE_BUFFER = 0,
	AD_READ_BUFFER = 1
} ad_buffer_t;

typedef enum {
	AD_ACCEL_INDEX = 0,
	AD_GYRO_INDEX = 1
} ad_sensor_index_t;

// struct to hold task context
typedef struct {
	ad_gyro_meas_t gyro_dual_buffer[2];
	ad_accel_meas_t accel_dual_buffer[2];
} ad_task_ctx_t;

typedef struct {
	uint32_t recent_gyro_data_read_fails;
	uint32_t adxrs649_data_check_fails;
	uint32_t recent_accel_data_read_fails;
	uint32_t adxl380_data_check_fails;
	uint32_t ad_breakout_board_data_logging_fails;
} ad_breakout_board_health_t;

static const uint8_t AD_BREAKOUT_BOARD_PERIOD_MS = 2;
static const size_t AD_GYRO_MEASUREMENT_SIZE = sizeof(ad_gyro_meas_t);
static const size_t AD_GYRO_RAW_MEASUREMENT_SIZE = sizeof(int32_t);
static const size_t AD_ACCEL_MEASUREMENT_SIZE = sizeof(ad_accel_meas_t);
static const size_t AD_ACCEL_RAW_MEASUREMENT_SIZE = sizeof(adxl380_raw_accel_data_t);

static ad_task_ctx_t g_task_ctx = {};

static ad_breakout_board_health_t ad_breakout_board_health = {0};

static w_status_t ad_breakout_board_data_logging(uint32_t loop_count, const uint32_t raw_gyro,
												 const adxl380_raw_accel_data_t *g_raw_accel) {
	return W_SUCCESS;
}

/**
 * @brief the FreeRTOS Task for getting the sensor data
 */
void ad_breakout_board_task(void *argument) {
	(void)argument;

	// Variables for precise timing control
	TickType_t last_wake_time = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(AD_BREAKOUT_BOARD_PERIOD_MS);
	uint32_t loop_count = 0;

	uint32_t raw_gyro = 0;
	adxl380_raw_accel_data_t raw_accel = {0};
	uint32_t current_time_ms = 0;

	while (1) {
		// get current timestamp
		if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
			current_time_ms = 0;
		}

		// assume gyro not updated
		bool update_gyro_data = false;
		g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].latest_status = W_SUCCESS; // default success

		if (adxrs649_is_data_ready(&update_gyro_data) == W_SUCCESS) {
			// if no new data then we just skip getting data
			if (update_gyro_data) {
				if (adxrs649_get_gyro_data(&(g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].meas),
										   &raw_gyro) != W_SUCCESS) {
					g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].latest_status = W_IO_ERROR;
					ad_breakout_board_health.recent_gyro_data_read_fails++;
					log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Failed to read gyro.");
				}
			}

		} else {
			update_gyro_data = true;
			g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].latest_status = W_IO_ERROR;
			ad_breakout_board_health.adxrs649_data_check_fails++;
			log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Failed to read gyro drdy.");
		}

		// if new gyro data update the timestamp
		if (update_gyro_data) {
			g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].timestamp_ms = current_time_ms;
		}

		// assume gyro not updated
		bool update_accel_data = false;
		g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].latest_status = W_SUCCESS; // default success

		if (adxl380_is_data_ready(&update_accel_data) == W_SUCCESS) {
			// if no new data then we just skip getting data
			if (update_accel_data) {
				if (adxl380_get_accel_data(&(g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].meas),
										   &raw_accel) != W_SUCCESS) {
					g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].latest_status = W_IO_ERROR;
					ad_breakout_board_health.recent_accel_data_read_fails++;
					log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Failed to read accel.");
				}
			}

		} else {
			update_accel_data = true;
			g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].latest_status = W_IO_ERROR;
			ad_breakout_board_health.adxl380_data_check_fails++;
			log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Failed to read accel drdy.");
		}

		// if new gyro data update the timestamp
		if (update_accel_data) {
			g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].timestamp_ms = current_time_ms;
		}

		taskENTER_CRITICAL();
		// Gyro
		if (update_gyro_data) {
			g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER] =
				g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER];
		}

		// Accelerometer
		if (update_accel_data) {
			g_task_ctx.accel_dual_buffer[AD_READ_BUFFER] =
				g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER];
		}
		taskEXIT_CRITICAL();

		// LOG/TELEMETRY
		if (W_SUCCESS != ad_breakout_board_data_logging(loop_count, raw_gyro, &raw_accel)) {
			ad_breakout_board_health.ad_breakout_board_data_logging_fails++;
			log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Failed to complete data logging.");
		}

		loop_count++;

		vTaskDelayUntil(&last_wake_time, period);
	}
}

/**
 * @brief to read both the accelerometer data
 * @param p_accel_data this is a pointer to converted accelerometer data
 * @param timestamp_ms return pointer for the data timestamp
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_accel_data(vector3d_t *p_accel_data, uint32_t *timestamp_ms) {
	if ((NULL == p_accel_data) || (NULL == timestamp_ms)) {
		log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Invalid return ptrs.");
		return W_INVALID_PARAM;
	}

	w_status_t return_status = W_FAILURE;
	taskENTER_CRITICAL();
	*p_accel_data = g_task_ctx.accel_dual_buffer[AD_READ_BUFFER].meas;
	*timestamp_ms = g_task_ctx.accel_dual_buffer[AD_READ_BUFFER].timestamp_ms;
	return_status = g_task_ctx.accel_dual_buffer[AD_READ_BUFFER].latest_status;
	taskEXIT_CRITICAL();

	return return_status;
}

/**
 * @brief to read both the gyro data
 * @param p_gyro_data this is a pointer to converted gyro data
 * @param timestamp_ms return pointer for the data timestamp
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_gyro_data(float64_t *p_gyro_data, uint32_t *timestamp_ms) {
	if ((NULL == p_gyro_data) || (NULL == timestamp_ms)) {
		log_text(0, LOG_LVL_WARN, "AD BREAKBOARD TASK", "Invalid return ptrs.");
		return W_INVALID_PARAM;
	}

	w_status_t return_status = W_FAILURE;
	taskENTER_CRITICAL();
	*p_gyro_data = g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER].meas;
	*timestamp_ms = g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER].timestamp_ms;
	return_status = g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER].latest_status;
	taskEXIT_CRITICAL();

	return return_status;
}

health_status_t ad_breakout_board_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_AD_BREAKOUT_BOARD, .error_bitfield = 0};

	// Initialization failed if sending NULL exits early based on non-init (FAILURE) instead of
	// INVALID_PARAM
	bool adxl380_is_init = W_FAILURE == adxl380_is_data_ready(NULL);
	bool adxrs649_is_init = W_FAILURE == adxrs649_is_data_ready(NULL);

	log_text(10,
			 LOG_LVL_INFO,
			 "AD BREAKBOARD TASK",
			 "adxrs649_init=%d, adxrs649_read_fails=%lu, adxrs649_ready_check_fails=%lu",
			 adxrs649_is_init,
			 ad_breakout_board_health.recent_gyro_data_read_fails,
			 ad_breakout_board_health.adxrs649_data_check_fails);

	log_text(10,
			 LOG_LVL_INFO,
			 "AD BREAKBOARD TASK",
			 "adxl380_init=%d, adxl380_read_fails=%lu, adxl380_ready_check_fails=%lu",
			 adxl380_is_init,
			 ad_breakout_board_health.recent_accel_data_read_fails,
			 ad_breakout_board_health.adxl380_data_check_fails);

	log_text(10,
			 LOG_LVL_INFO,
			 "AD BREAKBOARD TASK",
			 "data_logging_fails=%lu, gyro_latest_status=%d, accel_latest_status=%d",
			 ad_breakout_board_health.ad_breakout_board_data_logging_fails,
			 g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER].latest_status,
			 g_task_ctx.accel_dual_buffer[AD_READ_BUFFER].latest_status);

	if (!adxl380_is_init || !adxrs649_is_init) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_NOT_INIT;
	}

	if (ad_breakout_board_health.recent_accel_data_read_fails ||
		ad_breakout_board_health.recent_gyro_data_read_fails) {
		ad_breakout_board_health.recent_accel_data_read_fails = 0;
		ad_breakout_board_health.recent_gyro_data_read_fails = 0;
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_I2C_FAIL;
	}

	return status;
}
