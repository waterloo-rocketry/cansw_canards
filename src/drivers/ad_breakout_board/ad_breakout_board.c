
#include <stdint.h>

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

// struct to hold task context
typedef struct {
	ad_gyro_mesurement_t gyro_dual_buffer[2];
	ad_accelerometer_mesurement_t accel_dual_buffer[2];
} ad_task_ctx_t;

static const uint8_t AD_BREAKOUT_BOARD_PERIOD_MS = 2;
static const size_t AD_GYRO_MEASUREMENT_SIZE = sizeof(ad_gyro_mesurement_t);
static const size_t AD_GYRO_RAW_MEASUREMENT_SIZE = sizeof(int32_t);
static const size_t AD_ACCEL_MEASUREMENT_SIZE = sizeof(ad_accelerometer_mesurement_t);
static const size_t AD_ACCEL_RAW_MEASUREMENT_SIZE = sizeof(adxl380_raw_accel_data_t);

/* ADXRS */
// SD Card Log
static const uint16_t IDLE_RECOVERY_ADXRS_SD_LOG_PERIOD_MS = 1000;
static const uint16_t PAD_ADXRS_SD_LOG_PERIOD_MS = 1000 / 20;
static const uint16_t FLIGHT_ADXRS_SD_LOG_PERIOD_MS = 1000 / 200;

static const uint16_t IDLE_RECOVERY_ADXRS_SD_LOG_RATE =
	IDLE_RECOVERY_ADXRS_SD_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXRS_SD_LOG_RATE =
	PAD_ADXRS_SD_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXRS_SD_LOG_RATE =
	FLIGHT_ADXRS_SD_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;

// Flash Log
static const uint16_t IDLE_RECOVERY_ADXRS_FLASH_LOG_PERIOD_MS = 1000;
static const uint16_t PAD_ADXRS_FLASH_LOG_PERIOD_MS = 1000 / 5;
static const uint16_t FLIGHT_ADXRS_FLASH_LOG_PERIOD_MS = 1000 / 20;

static const uint16_t IDLE_RECOVERY_ADXRS_FLASH_LOG_RATE =
	IDLE_RECOVERY_ADXRS_FLASH_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXRS_FLASH_LOG_RATE =
	PAD_ADXRS_FLASH_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXRS_FLASH_LOG_RATE =
	FLIGHT_ADXRS_FLASH_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;

// CAN (Telemetry)
static const uint16_t IDLE_ADXRS_CAN_LOG_PERIOD_MS = 1000;
static const uint16_t PAD_ADXRS_CAN_LOG_PERIOD_MS = 1000 / 10;
static const uint16_t FLIGHT_ADXRS_CAN_LOG_PERIOD_MS = 1000 / 10;

static const uint16_t IDLE_ADXRS_CAN_LOG_RATE =
	IDLE_ADXRS_CAN_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXRS_CAN_LOG_RATE =
	PAD_ADXRS_CAN_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXRS_CAN_LOG_RATE =
	FLIGHT_ADXRS_CAN_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;

/* ADXL */
// SD Card Log
static const uint16_t IDLE_RECOVERY_ADXL_SD_LOG_PERIOD_MS = 1000;
static const uint16_t PAD_ADXL_SD_LOG_PERIOD_MS = 1000 / 20;
static const uint16_t FLIGHT_ADXL_SD_LOG_PERIOD_MS = 1000 / 50;

static const uint16_t IDLE_RECOVERY_ADXL_SD_LOG_RATE =
	IDLE_RECOVERY_ADXL_SD_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXL_SD_LOG_RATE =
	PAD_ADXL_SD_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXL_SD_LOG_RATE =
	FLIGHT_ADXL_SD_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;

// Flash Log
static const uint16_t IDLE_RECOVERY_ADXL_FLASH_LOG_PERIOD_MS = 1000;
static const uint16_t PAD_ADXL_FLASH_LOG_PERIOD_MS = 1000 / 5;
static const uint16_t FLIGHT_ADXL_FLASH_LOG_PERIOD_MS = 1000 / 20;

static const uint16_t IDLE_RECOVERY_ADXL_FLASH_LOG_RATE =
	IDLE_RECOVERY_ADXL_FLASH_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXL_FLASH_LOG_RATE =
	PAD_ADXL_FLASH_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXL_FLASH_LOG_RATE =
	FLIGHT_ADXL_FLASH_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;

// CAN (Telemetry)
static const uint16_t IDLE_ADXL_CAN_LOG_PERIOD_MS = 1000;
static const uint16_t PAD_ADXL_CAN_LOG_PERIOD_MS = 1000 / 10;
static const uint16_t FLIGHT_ADXL_CAN_LOG_PERIOD_MS = 1000 / 10;

static const uint16_t IDLE_ADXL_CAN_LOG_RATE =
	IDLE_ADXL_CAN_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXL_CAN_LOG_RATE =
	PAD_ADXL_CAN_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXL_CAN_LOG_RATE =
	FLIGHT_ADXL_CAN_LOG_PERIOD_MS / AD_BREAKOUT_BOARD_PERIOD_MS;

static ad_task_ctx_t g_task_ctx = {};

/**
 * @brief initalize both the breakout board sensor drivers
 * @return the status of initalization
 */
w_status_t ad_beakout_board_init() {
	w_status_t status = W_SUCCESS;

	status |= adxrs649_init();
	status |= adxl380_init();

	if (W_SUCCESS != status) {
		log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to initalize the drivers.");
		return W_FAILURE;
	}
	return W_SUCCESS;
}

static w_status_t ad_breakout_board_data_logging(uint32_t loop_count, const int32_t raw_gyro,
												 const adxl380_raw_accel_data_t *g_raw_accel) {
	return W_FAILURE;
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
	adxl380_raw_accel_data_t raw_accel = {};
	uint32_t current_time_ms = 0;

	while (1) {
		// get current timestamp
		if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
			current_time_ms = 0;
		}

		g_task_ctx.gyro_dual_buffer[0].timestamp = current_time_ms;
		g_task_ctx.accel_dual_buffer[0].timestamp = current_time_ms;

		if (W_SUCCESS ==
			adxrs649_get_gyro_data(&(g_task_ctx.gyro_dual_buffer[0].z_rate), &raw_gyro)) {
			g_task_ctx.gyro_dual_buffer[0].is_dead = true;
		} else {
			g_task_ctx.gyro_dual_buffer[0].is_dead = false;
			log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to read gyro.");
		}

		if (W_SUCCESS ==
			adxl380_get_accel_data(&(g_task_ctx.accel_dual_buffer[0].accelerometer), &raw_accel)) {
			g_task_ctx.accel_dual_buffer[0].is_dead = true;
		} else {
			g_task_ctx.accel_dual_buffer[0].is_dead = false;
			log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to read gyro.");
		}

		taskENTER_CRITICAL();
		// Gyro
		memcpy(&(g_task_ctx.gyro_dual_buffer[1]),
			   &(g_task_ctx.gyro_dual_buffer[0]),
			   AD_GYRO_MEASUREMENT_SIZE);

		// Accelerometer
		memcpy(&(g_task_ctx.accel_dual_buffer[1]),
			   &(g_task_ctx.accel_dual_buffer[0]),
			   AD_ACCEL_MEASUREMENT_SIZE);
		taskEXIT_CRITICAL();

		// LOG/TELEMETRY
		if (W_SUCCESS != ad_breakout_board_data_logging(loop_count, raw_gyro, &raw_accel)) {
			log_text(0, "AD BREAKBOARD TASK", "ERROR: Failed to complete data logging.");
		}

		loop_count++;

		vTaskDelayUntil(&last_wake_time, period);
	}
}

/**
 * @brief to read both the accelerometer and gyro data
 * @param g_gyro_data this is a pointer to converted gyro data
 * @param g_accel_data pointer to state of converted accel data
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(ad_gyro_mesurement_t *g_gyro_data,
									  ad_accelerometer_mesurement_t *g_accel_data) {
	taskENTER_CRITICAL();
	memcpy(g_gyro_data, &(g_task_ctx.gyro_dual_buffer[1]), AD_GYRO_MEASUREMENT_SIZE);
	memcpy(g_accel_data, &(g_task_ctx.accel_dual_buffer[1]), AD_ACCEL_MEASUREMENT_SIZE);
	taskEXIT_CRITICAL();

	return W_SUCCESS;
}
