#include "drivers/ad_breakout_board/ad_breakout_board.h"
#include "application/flight_phase/flight_phase.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "rocketlib/include/common.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

// struct to hold task context
typedef struct {
	ad_gyro_mesurement_t gyro_dual_buffer[2];
	int32_t gyro_raw_dual_buffer_z_rate[2]; // TODO: ignored for since logging is done locally
	ad_accelerometer_mesurement_t accel_dual_buffer[2];
	altimu_raw_imu_data_t
		accel_raw_dual_buffer[2]; // TODO: Ignored for now since logging is done locally

} ad_task_ctx_t;

static const uint8_t AD_BREAKOUT_BOARD_PERIOD_MS = 2;
static const size_t AD_GYRO_MEASUREMENT_SIZE = sizeof(ad_gyro_mesurement_t);
static const size_t AD_GYRO_RAW_MEASUREMENT_SIZE = sizeof(int32_t);
static const size_t AD_ACCEL_MEASUREMENT_SIZE = sizeof(ad_accelerometer_mesurement_t);
static const size_t AD_ACCEL_RAW_MEASUREMENT_SIZE = sizeof(altimu_raw_imu_data_t);

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

static ad_task_ctx_t task_ctx = {};

/**
 * @brief initalize both the breakout board sensor drivers
 * @return the status of initalization
 */
w_status_t ad_beakout_board_init() {
	w_status_t status = W_SUCCESS;

	status |= adxrs649_init();
	// TODO: add ADXL init

	return status;
}

static w_status_t ad_breakout_board_data_logging(uint32_t loop_count, int32_t raw_gyro,
												 altimu_raw_imu_data_t raw_accel) {
	flight_phase_state_t flight_state = flight_phase_get_state();

	// TODO: logging function
	switch (flight_state) {
		case STATE_IDLE:
		case STATE_RECOVERY:

			if ((loop_count % IDLE_RECOVERY_ADXRS_SD_LOG_RATE) == 0) {}
			if ((loop_count % IDLE_RECOVERY_ADXRS_SD_LOG_RATE) == 0) {}
			if ((loop_count % IDLE_RECOVERY_ADXRS_SD_LOG_RATE) == 0) {}
			break;
		default:
			break;
	}

	return W_FAILURE; // TODO: change to something that actually makes sense
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

	int32_t raw_gyro = 0;
	altimu_raw_imu_data_t raw_accel = {};

	while (1) {
		if (W_SUCCESS ==
			adxrs649_get_gyro_data(&(task_ctx.gyro_dual_buffer[0].z_rate), &raw_gyro)) {
			task_ctx.gyro_dual_buffer[0].is_dead = true;
		} else {
			task_ctx.gyro_dual_buffer[0].is_dead = false;
		}
		// TODO: add ADXL get acceleration

		taskENTER_CRITICAL();
		// Gyro
		memcpy(&(task_ctx.gyro_dual_buffer[1]),
			   &(task_ctx.gyro_dual_buffer[0]),
			   AD_GYRO_MEASUREMENT_SIZE);

		// Accelerometer
		memcpy(&(task_ctx.accel_dual_buffer[1]),
			   &(task_ctx.accel_dual_buffer[0]),
			   AD_ACCEL_MEASUREMENT_SIZE);
		taskEXIT_CRITICAL();

		// LOG/TELEMETRY
		ad_breakout_board_data_logging(loop_count, raw_gyro, raw_accel);

		loop_count++;

		vTaskDelayUntil(&last_wake_time, period);
	}
}

/**
 * @brief to read both the accelerometer and gyro data
 * @param gyro_data this is a pointer to converted data
 * @param accel_data pointer to state of our data
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(ad_gyro_mesurement_t *gyro_data,
									  ad_accelerometer_mesurement_t *accel_data) {
	taskENTER_CRITICAL();
	memcpy(gyro_data, &(task_ctx.gyro_dual_buffer[1]), AD_GYRO_MEASUREMENT_SIZE);
	memcpy(accel_data, &(task_ctx.accel_dual_buffer[1]), AD_ACCEL_MEASUREMENT_SIZE);
	taskEXIT_CRITICAL();

	return W_SUCCESS; // TODO: change to something that actually makes sense
}

