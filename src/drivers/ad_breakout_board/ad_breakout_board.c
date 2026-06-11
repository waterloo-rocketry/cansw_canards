
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
	navigator_1d_meas_t gyro_dual_buffer[2];
	navigator_3d_meas_t accel_dual_buffer[2];
	uint32_t timestamp_ms[2][2]; // not becasue it's not atomic but each repersent each buffer
} ad_task_ctx_t;

static const uint8_t AD_BREAKOUT_BOARD_PERIOD_MS = 2;
static const size_t AD_GYRO_MEASUREMENT_SIZE = sizeof(navigator_1d_meas_t);
static const size_t AD_GYRO_RAW_MEASUREMENT_SIZE = sizeof(int32_t);
static const size_t AD_ACCEL_MEASUREMENT_SIZE = sizeof(navigator_3d_meas_t);
static const size_t AD_ACCEL_RAW_MEASUREMENT_SIZE = sizeof(adxl380_raw_accel_data_t);

static ad_task_ctx_t g_task_ctx = {};

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
	uint32_t time = 0;

	while (1) {
		// get timestamp of task

		timer_get_tenth_ms(&time);
		log_text(0, "AD BREAKOUT TASK", "ENTER TASK %d", time);

		// get current timestamp
		if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
			current_time_ms = 0;
		}

		// assume gyro not updated
		bool update_gyro_data = false;

		if (adxrs649_is_data_ready(&update_gyro_data) == W_SUCCESS) {
			if (update_gyro_data) {
				if (adxrs649_get_gyro_data(&(g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].meas),
										   &raw_gyro) == W_SUCCESS) {
					g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].is_dead = false;
				} else {
					g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].is_dead = true;
					log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to read gyro.");
				}
			}
		} else {
			update_gyro_data = true;
			g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER].is_dead = true;
			log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to read gyro drdy.");
		}

		// if new gyro data update the timestamp
		if (update_gyro_data) {
			g_task_ctx.timestamp_ms[AD_WRITE_BUFFER][AD_GYRO_INDEX] = current_time_ms;
		}

		// assume gyro not updated
		bool update_accel_data = false;

		if (adxl380_is_data_ready(&update_accel_data) == W_SUCCESS) {
			if (update_accel_data) {
				if (adxl380_get_accel_data(
									&(g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].meas), &raw_accel) == W_SUCCESS) {
					g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].is_dead = false;
				} else {
					g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].is_dead = true;
					log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to read accel.");
				}
			}
		} else {
			update_accel_data = true;
			g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER].is_dead = true;
			log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to read accel drdy.");
		}

		// if new gyro data update the timestamp
		if (update_accel_data) {
			g_task_ctx.timestamp_ms[AD_WRITE_BUFFER][AD_ACCEL_INDEX] = current_time_ms;
		}

		taskENTER_CRITICAL();
		// Gyro
		if (update_gyro_data) {
			memcpy(&(g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER]),
				   &(g_task_ctx.gyro_dual_buffer[AD_WRITE_BUFFER]),
				   AD_GYRO_MEASUREMENT_SIZE);
			g_task_ctx.timestamp_ms[AD_READ_BUFFER][AD_GYRO_INDEX] = g_task_ctx.timestamp_ms[AD_WRITE_BUFFER][AD_GYRO_INDEX];
		}

		// Accelerometer
		if (update_accel_data) {
			memcpy(&(g_task_ctx.accel_dual_buffer[AD_READ_BUFFER]),
				&(g_task_ctx.accel_dual_buffer[AD_WRITE_BUFFER]),
				AD_ACCEL_MEASUREMENT_SIZE);
			g_task_ctx.timestamp_ms[AD_READ_BUFFER][AD_ACCEL_INDEX] = g_task_ctx.timestamp_ms[AD_WRITE_BUFFER][AD_ACCEL_INDEX];
		}
		taskEXIT_CRITICAL();

		// LOG/TELEMETRY
		if (W_SUCCESS != ad_breakout_board_data_logging(loop_count, raw_gyro, &raw_accel)) {
			log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to complete data logging.");
		}

		loop_count++;



		timer_get_tenth_ms(&time);
		log_text(0, "AD BREAKOUT TASK", "Exit TASK %d", time);

		vTaskDelayUntil(&last_wake_time, period);
	}
}

/**
 * @brief to read both the accelerometer and gyro data
 * @param p_gyro_data this is a pointer to converted gyro data
 * @param p_accel_data pointer to state of converted accel data
 * @param p_gyro_timestamp_ms the the timestamp at which the data was collected for gyro
 * @param p_accel_timestamp_ms the the timestamp at which the data was collected for accel
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(navigator_1d_meas_t *p_gyro_data,
									  navigator_3d_meas_t *p_accel_data,
									  uint32_t *p_gyro_timestamp_ms,
									  uint32_t *p_accel_timestamp_ms) {
	if ((NULL == p_gyro_data) || (NULL == p_accel_data) || (NULL == p_gyro_timestamp_ms) ||
		(NULL == p_accel_timestamp_ms)) {
		log_text(0, "AD BREAKBOARD TASK", "ERROR: Invalid return ptrs.");
		return W_INVALID_PARAM;
	}
	taskENTER_CRITICAL();
	memcpy(p_gyro_data, &(g_task_ctx.gyro_dual_buffer[AD_READ_BUFFER]), AD_GYRO_MEASUREMENT_SIZE);
	memcpy(
		p_accel_data, &(g_task_ctx.accel_dual_buffer[AD_READ_BUFFER]), AD_ACCEL_MEASUREMENT_SIZE);

	*p_gyro_timestamp_ms = g_task_ctx.timestamp_ms[AD_READ_BUFFER][AD_GYRO_INDEX];
	*p_accel_timestamp_ms = g_task_ctx.timestamp_ms[AD_READ_BUFFER][AD_ACCEL_INDEX];
	taskEXIT_CRITICAL();

	return W_SUCCESS;
}
