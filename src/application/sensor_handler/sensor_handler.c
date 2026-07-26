#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "application/sensor_handler/sensor_handler.h"
#include "application/telemetry/telemetry.h"
#include "canlib.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/MS5611/MS5611.h"
#include "drivers/ad_breakout_board/ad_breakout_board.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "drivers/iis2mdc/IIS2MDC.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

// conversion factors
static const float64_t M_S2_PER_G = 9.81;
static const float64_t PA_PER_CENTIMBAR = 1;

// TODO: double check values with Tristan
// Timeout values for freshness check (in milliseconds)
static const int32_t ST_IMU_FRESHNESS_TIMEOUT_MS = 2;
static const int32_t AD_ACCEL_FRESHNESS_TIMEOUT_MS = 2;
static const int32_t AD_GYRO_FRESHNESS_TIMEOUT_MS = 2;
static const int32_t MAG_FRESHNESS_TIMEOUT_MS = 5;
static const int32_t BARO_FRESHNESS_TIMEOUT_MS = 5;
static const int32_t MOTOR_ENCODER_FRESHNESS_TIMEOUT_MS = 10;

// TODO: consider splitting to each sensor since the data is coming seperately
static const int32_t MTI_FRESHNESS_TIMEOUT_MS = 5;

// Rate limit CAN tx: only send data at 10Hz, every 100ms
// static const uint32_t IMU_HANDLER_CAN_TX_PERIOD_MS = 100;
// static const uint32_t IMU_HANDLER_CAN_TX_RATE =
// 	(IMU_HANDLER_CAN_TX_PERIOD_MS / ST_IMU_FRESHNESS_TIMEOUT_MS);
static const matrix3d_t g_mti_correction_matrix = {
	.array = {{0, 0, 1.0}, {1.0, 0, 0}, {0, 1.0, 0}}};
static const matrix3d_t g_board_imu_correction_matrix = {
	.array = {{0, 0, -1.0}, {1.0, 0, 0}, {0, -1.0, 0}}};
// TODO: Must be confirmed on July 11th
static const matrix3d_t g_board_mag_correction_matrix = {
	.array = {{0, 0, -1.0}, {0, -1.0, 0}, {1.0, 0, 0}}};
static const matrix3d_t g_ad_accel_correction_matrix = {
	.array = {{0, 0, 1.0}, {0, -1.0, 0}, {1.0, 0, 0}}};

// mag hard iron and soft iron calibration values
static const vector3d_t hard_iron_bias = {.x = 0, .y = 0, .z = 0};
static const matrix3d_t soft_iron_correction_matrix = {
	.array = {{1.0, 0, 0}, {0, 1.0, 0}, {0, 0, 1.0}}};

// ad accel null bias offsets
static const float64_t AD_ACCEL_X_NULL_BIAS_OFFSET = -0.61;
static const float64_t AD_ACCEL_Y_NULL_BIAS_OFFSET = -0.65;
static const float64_t AD_ACCEL_Z_NULL_BIAS_OFFSET = 0.15;

// set to true once calibrated, initialized to false to prevent use before calibration
static bool orientation_calibrated = false;

typedef struct {
	uint32_t success_count;
	uint32_t failure_count;
} sensor_health_state_t;

// Module state tracking
typedef struct {
	bool initialized;
	uint32_t sample_count;
	uint32_t error_count;

	// Per-IMU stats
	sensor_health_state_t board_imu_stats;
	sensor_health_state_t board_mag_stats;
	sensor_health_state_t board_baro_stats;
	sensor_health_state_t ad_accel_stats;
	sensor_health_state_t ad_gyro_stats;
	sensor_health_state_t mti_accel_stats;
	sensor_health_state_t mti_gyro_stats;
	sensor_health_state_t mti_mag_stats;
	sensor_health_state_t mti_baro_stats;
	sensor_health_state_t motor_encoder_stats;
} sensor_handler_state_t;

static sensor_handler_state_t sensor_handler_state = {0};

// All sensor values broadcast as telemetry, in the units/frames expected by the ground station.
// One flat snapshot so the telemetry task reads a single consistent set of values.
typedef struct {
	// LSM6DSV32X (board IMU)
	vector3d_t board_imu_accel; // m/s^2
	vector3d_t board_imu_gyro; // rad/2
	// MS5611 (board barometer)
	int32_t board_baro_pressure_pa;
	// TODO: add board barometer thermometer reading (board_baro_temp) once wired
	// LSM303AGR (board mag)
	vector3d_t board_mag;
	// MTi-630 (Movella)
	vector3d_t mti_accel; // m/s^2
	vector3d_t mti_gyro; // rad/s
	vector3d_t mti_mag; // gauss
	float32_t mti_baro_pressure; // Pa
	// ADXL380 (AD breakout accel)
	vector3d_t ad_accel; // m/s^2
	// ADXRS649 (AD high-rate gyro)
	float32_t ad_gyro; // rad/s, 1 axis
} sensor_can_telem_data_t;

// Length-1 mailbox holding the most recent telemetry snapshot. The producer
// (sensor_handler_get_fresh_meas, sensor task) overwrites it each cycle; the consumers (the
// *_telemetry() log functions, telemetry task) peek it on their own cadence. Using a mailbox
// keeps the cross-task hand-off atomic without a separate lock.
static QueueHandle_t g_sensor_data_queue = NULL;

/**
 * @brief Peek the latest telemetry snapshot from the mailbox.
 * @return W_SUCCESS if a snapshot was available, W_FAILURE otherwise.
 */
static w_status_t sensor_handler_get_latest(sensor_can_telem_data_t *out) {
	if ((NULL == g_sensor_data_queue) || (xQueuePeek(g_sensor_data_queue, out, 0) != pdPASS)) {
		return W_FAILURE;
	}
	return W_SUCCESS;
}

// ---------------------------------------------------------------------------
// Per-sensor telemetry log functions registered with the telemetry module.
// Board IMU / mag / baro send their raw register values and AD sends the float sensor_handler
// already has, all cast straight into the CAN field with no scaling. Only the MTI readings are
// scaled (real SCALE_MTI_* factors).
// ---------------------------------------------------------------------------

// LSM6DSV32X (board IMU): accelerometer + gyroscope sent as raw register counts.
static w_status_t board_imu_telemetry(void) {
	sensor_can_telem_data_t data;
	w_status_t status = W_SUCCESS;

	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	uint32_t ts_ms = 0;
	(void)timer_get_ms(&ts_ms);

	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;
	w_status_t accel_enc = W_SUCCESS;
	accel_enc |=
		can_encode_scaled_float(SCALE_BOARD_ACCEL, (float32_t)data.board_imu_accel.x, &accel_x);
	accel_enc |=
		can_encode_scaled_float(SCALE_BOARD_ACCEL, (float32_t)data.board_imu_accel.y, &accel_y);
	accel_enc |=
		can_encode_scaled_float(SCALE_BOARD_ACCEL, (float32_t)data.board_imu_accel.z, &accel_z);

	if (W_SUCCESS == accel_enc) {
		can_msg_t accel_msg = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_LSM6DSV32X_ACCEL,
										 (uint16_t)(accel_x + TELEM_INT16_OFFSET),
										 (uint16_t)(accel_y + TELEM_INT16_OFFSET),
										 (uint16_t)(accel_z + TELEM_INT16_OFFSET),
										 &accel_msg);
		status |= can_handler_transmit(&accel_msg);

	} else {
		status |= W_FAILURE;
	}

	int16_t gyro_x = 0;
	int16_t gyro_y = 0;
	int16_t gyro_z = 0;
	w_status_t gyro_enc = W_SUCCESS;
	gyro_enc |=
		can_encode_scaled_float(SCALE_BOARD_GYRO, (float32_t)data.board_imu_gyro.x, &gyro_x);
	gyro_enc |=
		can_encode_scaled_float(SCALE_BOARD_GYRO, (float32_t)data.board_imu_gyro.y, &gyro_y);
	gyro_enc |=
		can_encode_scaled_float(SCALE_BOARD_GYRO, (float32_t)data.board_imu_gyro.z, &gyro_z);

	if (W_SUCCESS == gyro_enc) {
		can_msg_t gyro_msg = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_LSM6DSV32X_GYRO,
										 (uint16_t)(gyro_x + TELEM_INT16_OFFSET),
										 (uint16_t)(gyro_y + TELEM_INT16_OFFSET),
										 (uint16_t)(gyro_z + TELEM_INT16_OFFSET),
										 &gyro_msg);
		status |= can_handler_transmit(&gyro_msg);

	} else {
		status |= W_FAILURE;
	}

	return status;
}

// MS5611 (board barometer): raw pressure (Pa). Thermometer half not wired yet, sending 0.
static w_status_t board_baro_telemetry(void) {
	sensor_can_telem_data_t data;
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	uint32_t ts_ms = 0;
	(void)timer_get_ms(&ts_ms);

	can_msg_t msg = {0};

	uint32_t baro_pres = 0;
	if (can_encode_scaled_int(
			SCALE_BOARD_PRESSURE, (int64_t)data.board_baro_pressure_pa, &baro_pres) != W_SUCCESS) {
		return W_FAILURE;
	}

	build_2d_analog_sensor_24bit_msg(PRIO_LOW,
									 (uint16_t)ts_ms,
									 DEM_2D_SENSOR_CANARD_MS5611_BARO_TEMP,
									 baro_pres,
									 0, // TODO:temp is being sent as zero now as a placeholder
									 &msg);
	return can_handler_transmit(&msg);
}

// MTi-630 (Movella) + IIS2MDC (board mag): both run at the same rate, sent together.
static w_status_t mti_board_mag_telemetry(void) {
	sensor_can_telem_data_t data;
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	uint32_t ts_ms = 0;
	(void)timer_get_ms(&ts_ms);
	w_status_t enc = W_SUCCESS;

	// IIS2MDC board magnetometer - raw register counts, unscaled
	int16_t board_mag_x = 0;
	int16_t board_mag_y = 0;
	int16_t board_mag_z = 0;

	enc |= can_encode_scaled_float(SCALE_BOARD_MAG, (float32_t)data.board_mag.x, &board_mag_x);
	enc |= can_encode_scaled_float(SCALE_BOARD_MAG, (float32_t)data.board_mag.y, &board_mag_y);
	enc |= can_encode_scaled_float(SCALE_BOARD_MAG, (float32_t)data.board_mag.z, &board_mag_z);
	can_msg_t board_mag_msg = {0};
	build_3d_analog_sensor_16bit_msg(PRIO_LOW,
									 (uint16_t)ts_ms,
									 DEM_3D_SENSOR_CANARD_IIS2MDC_MAG,
									 (uint16_t)(board_mag_x + TELEM_INT16_OFFSET),
									 (uint16_t)(board_mag_y + TELEM_INT16_OFFSET),
									 (uint16_t)(board_mag_z + TELEM_INT16_OFFSET),
									 &board_mag_msg);
	w_status_t status = can_handler_transmit(&board_mag_msg);

	// MTi-630 accel / gyro / mag: scale to int16, then bias into the unsigned field
	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;
	enc |= can_encode_scaled_float(SCALE_MTI_ACCEL, (float32_t)data.mti_accel.x, &accel_x);
	enc |= can_encode_scaled_float(SCALE_MTI_ACCEL, (float32_t)data.mti_accel.y, &accel_y);
	enc |= can_encode_scaled_float(SCALE_MTI_ACCEL, (float32_t)data.mti_accel.z, &accel_z);
	can_msg_t mti_accel_msg = {0};
	build_3d_analog_sensor_16bit_msg(PRIO_LOW,
									 (uint16_t)ts_ms,
									 DEM_3D_SENSOR_CANARD_MTI630_ACCEL,
									 (uint16_t)(accel_x + TELEM_INT16_OFFSET),
									 (uint16_t)(accel_y + TELEM_INT16_OFFSET),
									 (uint16_t)(accel_z + TELEM_INT16_OFFSET),
									 &mti_accel_msg);
	status |= can_handler_transmit(&mti_accel_msg);

	int16_t gyro_x = 0;
	int16_t gyro_y = 0;
	int16_t gyro_z = 0;
	enc |= can_encode_scaled_float(SCALE_MTI_GYRO, (float32_t)data.mti_gyro.x, &gyro_x);
	enc |= can_encode_scaled_float(SCALE_MTI_GYRO, (float32_t)data.mti_gyro.y, &gyro_y);
	enc |= can_encode_scaled_float(SCALE_MTI_GYRO, (float32_t)data.mti_gyro.z, &gyro_z);
	can_msg_t mti_gyro_msg = {0};
	build_3d_analog_sensor_16bit_msg(PRIO_LOW,
									 (uint16_t)ts_ms,
									 DEM_3D_SENSOR_CANARD_MTI630_GYRO,
									 (uint16_t)(gyro_x + TELEM_INT16_OFFSET),
									 (uint16_t)(gyro_y + TELEM_INT16_OFFSET),
									 (uint16_t)(gyro_z + TELEM_INT16_OFFSET),
									 &mti_gyro_msg);
	status |= can_handler_transmit(&mti_gyro_msg);

	int16_t mag_x = 0;
	int16_t mag_y = 0;
	int16_t mag_z = 0;
	enc |= can_encode_scaled_float(SCALE_MTI_MAG, (float32_t)data.mti_mag.x, &mag_x);
	enc |= can_encode_scaled_float(SCALE_MTI_MAG, (float32_t)data.mti_mag.y, &mag_y);
	enc |= can_encode_scaled_float(SCALE_MTI_MAG, (float32_t)data.mti_mag.z, &mag_z);
	can_msg_t mti_mag_msg = {0};
	build_3d_analog_sensor_16bit_msg(PRIO_LOW,
									 (uint16_t)ts_ms,
									 DEM_3D_SENSOR_CANARD_MTI630_MAG,
									 (uint16_t)(mag_x + TELEM_INT16_OFFSET),
									 (uint16_t)(mag_y + TELEM_INT16_OFFSET),
									 (uint16_t)(mag_z + TELEM_INT16_OFFSET),
									 &mti_mag_msg);
	status |= can_handler_transmit(&mti_mag_msg);

	// MTi-630 barometric pressure (scaled, uint32)
	uint32_t baro_scaled = 0;
	enc |= can_encode_scaled_float(SCALE_MTI_PRESSURE, data.mti_baro_pressure, &baro_scaled);
	can_msg_t mti_baro_msg = {0};
	build_analog_sensor_32bit_msg(
		PRIO_LOW, (uint16_t)ts_ms, SENSOR_CANARD_MTI630_BARO_0, baro_scaled, &mti_baro_msg);
	status |= can_handler_transmit(&mti_baro_msg);

	if (W_SUCCESS != enc) {
		log_text(0, LOG_LVL_WARN, "SensorHandler", "mti telem scale failed: %d", enc);
	}
	return ((W_SUCCESS == status) && (W_SUCCESS == enc)) ? W_SUCCESS : W_FAILURE;
}

// AD breakout: ADXL380 accelerometer + ADXRS649 high-rate 1-axis gyro sent together.
// The driver exposes only converted floats, so we send exactly what sensor_handler has, cast
// straight to the 16-bit field with no scaling applied.
static w_status_t ad_telemetry(void) {
	sensor_can_telem_data_t data;
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	uint32_t ts_ms = 0;
	(void)timer_get_ms(&ts_ms);

	can_msg_t accel_msg = {0};
	w_status_t enc = W_SUCCESS;

	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;
	enc |= can_encode_scaled_float(SCALE_ADXL380_ACCEL, (float32_t)data.ad_accel.x, &accel_x);
	enc |= can_encode_scaled_float(SCALE_ADXL380_ACCEL, (float32_t)data.ad_accel.y, &accel_y);
	enc |= can_encode_scaled_float(SCALE_ADXL380_ACCEL, (float32_t)data.ad_accel.z, &accel_z);

	build_3d_analog_sensor_16bit_msg(PRIO_LOW,
									 (uint16_t)ts_ms,
									 DEM_3D_SENSOR_CANARD_ADXL380_ACCEL,
									 (uint16_t)(accel_x + TELEM_INT16_OFFSET),
									 (uint16_t)(accel_y + TELEM_INT16_OFFSET),
									 (uint16_t)(accel_z + TELEM_INT16_OFFSET),
									 &accel_msg);
	w_status_t status = can_handler_transmit(&accel_msg);

	can_msg_t gyro_msg = {0};
	int32_t gyro_scaled = 0;
	enc |= can_encode_scaled_float(SCALE_ADXRS649_GYROSCOPE, data.ad_gyro, &gyro_scaled);
	build_analog_sensor_32bit_msg(
		PRIO_LOW,
		(uint16_t)ts_ms,
		SENSOR_CANARD_ADXRS649_GYRO,
		(uint32_t)(gyro_scaled + TELEM_INT32_OFFSET), // spans -+ 1000 so this is fine
		&gyro_msg);
	status |= can_handler_transmit(&gyro_msg);
	return status;
}

/**
 * @brief Read data from the board
 * @param ctx pointer to the ctx storing the previously updated times for the sensors
 * @param board_data Pointer to store the converted data
 * @param raw_data Pointer to store the raw data
 * @param curr_timestamp_ms the current time stamp for freshness calculations TODO
 * @return Status of the read operation
 */
static w_status_t read_board_meas(sensor_handler_ctx_t *ctx, navigator_board_meas_t *board_data,
								  raw_board_meas_t *raw_data, const uint32_t curr_timestamp_ms) {
	(void)curr_timestamp_ms;
	bool is_dead = true;

	w_status_t sensor_status = lsm6dsv32x_get_gyro_acc_data(&(board_data->board_imu.accel),
															&(board_data->board_imu.gyro),
															&(raw_data->raw_board_accel),
															&(raw_data->raw_board_gyro));

	// Read accelerometer and gyro data
	if (W_SUCCESS == sensor_status) {
		if ((raw_data->raw_board_accel.timestamp_ms) > (ctx->last_board_imu_timestamp_ms)) {
			board_data->board_imu.is_new = true;

			sensor_handler_state.board_imu_stats.success_count++;
		} else {
			board_data->board_imu.is_new = false;

			sensor_handler_state.board_imu_stats.failure_count++;
		}

		// update timestamp
		ctx->last_board_imu_timestamp_ms = (raw_data->raw_board_accel.timestamp_ms);
	} else {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Board IMU failed. CODE: %d", sensor_status);
		board_data->board_imu.is_new = false;

		sensor_handler_state.board_imu_stats.failure_count++;
	}

	// get mag.
	uint32_t mag_timestamp_ms = 0;

	sensor_status = iis2mdc_get_data(
		&(board_data->board_mag.meas), &(raw_data->raw_board_mag), &mag_timestamp_ms);
	if (W_SUCCESS == sensor_status) {
		if (mag_timestamp_ms > (ctx->last_mag_timestamp_ms)) {
			board_data->board_mag.is_new = true;

			sensor_handler_state.board_mag_stats.success_count++;
		} else {
			board_data->board_mag.is_new = false;

			sensor_handler_state.board_mag_stats.failure_count++;
		}

		ctx->last_mag_timestamp_ms = mag_timestamp_ms;
	} else {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Board Mag failed. CODE: %d", sensor_status);
		board_data->board_mag.is_new = false;

		sensor_handler_state.board_mag_stats.failure_count++;
	}

	// get baro
	uint32_t baro_timestamp_ms = 0;

	sensor_status = ms5611_get_raw_pressure(&(raw_data->raw_board_baro), &baro_timestamp_ms);
	if (W_SUCCESS == sensor_status) {
		if (baro_timestamp_ms > (ctx->last_baro_timestamp_ms)) {
			board_data->board_baro.is_new = true;

			sensor_handler_state.board_baro_stats.success_count++;
		} else {
			board_data->board_baro.is_new = false;

			sensor_handler_state.board_baro_stats.failure_count++;
		}

		ctx->last_baro_timestamp_ms = baro_timestamp_ms;
	} else {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Board Baro failed. CODE: %d", sensor_status);
		board_data->board_baro.is_new = false;

		sensor_handler_state.board_baro_stats.failure_count++;
	}

	// convert gyro from dps to rad/sec
	board_data->board_imu.gyro.x = (board_data->board_imu.gyro.x) * RAD_PER_DEG;
	board_data->board_imu.gyro.y = (board_data->board_imu.gyro.y) * RAD_PER_DEG;
	board_data->board_imu.gyro.z = (board_data->board_imu.gyro.z) * RAD_PER_DEG;

	// convert accel from g to m/s^2
	board_data->board_imu.accel.x = (board_data->board_imu.accel.x) * M_S2_PER_G;
	board_data->board_imu.accel.y = (board_data->board_imu.accel.y) * M_S2_PER_G;
	board_data->board_imu.accel.z = (board_data->board_imu.accel.z) * M_S2_PER_G;

	// mag data is already provided in Gauss

	// convert baro from mbar to Pascals
	board_data->board_baro.meas =
		((float64_t)(raw_data->raw_board_baro.pressure_centimbar)) * PA_PER_CENTIMBAR;

	// Apply orientation correction
	board_data->board_imu.accel =
		math_vector3d_rotate(&g_board_imu_correction_matrix, &(board_data->board_imu.accel));
	board_data->board_imu.gyro =
		math_vector3d_rotate(&g_board_imu_correction_matrix, &(board_data->board_imu.gyro));

	board_data->board_mag.meas =
		math_vector3d_rotate(&g_board_mag_correction_matrix, &(board_data->board_mag.meas));

	board_data->board_mag.meas = math_vector3d_subt(&(board_data->board_mag.meas), &hard_iron_bias);
	board_data->board_mag.meas =
		math_vector3d_rotate(&soft_iron_correction_matrix, &(board_data->board_mag.meas));

	// success is if at least one of the sensors updated
	if ((!board_data->board_mag.is_new) && (!board_data->board_imu.is_new) &&
		(!board_data->board_baro.is_new)) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Read data from the AD Breakoout
 * @param ctx pointer to the ctx storing the previously updated times for the sensors
 * @param ad_data Pointer to store the converted data
 * @param curr_timestamp_ms the current time stamp for freshness calculations TODO
 * @return Status of the read operation
 */
static w_status_t read_ad_meas(sensor_handler_ctx_t *ctx, navigator_ad_meas_t *ad_data,
							   const uint32_t curr_timestamp_ms) {
	(void)curr_timestamp_ms;
	bool is_dead = true;

	// get accel
	uint32_t accel_timestamp_ms = 0;
	w_status_t accel_status =
		ad_breakout_board_get_accel_data(&(ad_data->ad_accel.meas), &(accel_timestamp_ms));

	// Read accelerometer and gyro data
	if (W_SUCCESS == accel_status) {
		if ((accel_timestamp_ms) > (ctx->last_ad_accel_timestamp_ms)) {
			ad_data->ad_accel.is_new = true;

			sensor_handler_state.ad_accel_stats.success_count++;
		} else {
			ad_data->ad_accel.is_new = false;

			sensor_handler_state.ad_accel_stats.failure_count++;
		}

		// update timestamp
		ctx->last_ad_accel_timestamp_ms = accel_timestamp_ms;
	} else {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "AD380 failed. CODE: %d", accel_status);
		ad_data->ad_accel.is_new = false;
		sensor_handler_state.ad_accel_stats.failure_count++;
	}

	// get gyro
	uint32_t gyro_timestamp_ms = 0;
	w_status_t gyro_status =
		ad_breakout_board_get_gyro_data(&(ad_data->ad_gyro.meas), &(gyro_timestamp_ms));

	// Read accelerometer and gyro data
	if (W_SUCCESS == gyro_status) {
		if ((gyro_timestamp_ms) >
			(ctx->last_ad_gyro_timestamp_ms)) { // designed to make sure no overflow
			ad_data->ad_gyro.is_new = true;
			sensor_handler_state.ad_gyro_stats.success_count++;

		} else {
			ad_data->ad_gyro.is_new = false;
			sensor_handler_state.ad_gyro_stats.failure_count++;
		}

		// update timestamp
		ctx->last_ad_gyro_timestamp_ms = gyro_timestamp_ms;
	} else {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "ADXRS649 failed. CODE: %d", gyro_status);
		ad_data->ad_gyro.is_new = false;
		sensor_handler_state.ad_gyro_stats.failure_count++;
	}

	// convert gyro from dps to rad/sec
	ad_data->ad_gyro.meas = (ad_data->ad_gyro.meas) * RAD_PER_DEG;

	// convert accel from g to m/s^2
	ad_data->ad_accel.meas.x = (ad_data->ad_accel.meas.x) * M_S2_PER_G;
	ad_data->ad_accel.meas.y = (ad_data->ad_accel.meas.y) * M_S2_PER_G;
	ad_data->ad_accel.meas.z = (ad_data->ad_accel.meas.z) * M_S2_PER_G;

	// Apply orientation correction
	ad_data->ad_accel.meas =
		math_vector3d_rotate(&g_ad_accel_correction_matrix, &(ad_data->ad_accel.meas));

	// ad null bias offset
	ad_data->ad_accel.meas.x -= AD_ACCEL_X_NULL_BIAS_OFFSET;
	ad_data->ad_accel.meas.y -= AD_ACCEL_Y_NULL_BIAS_OFFSET;
	ad_data->ad_accel.meas.z -= AD_ACCEL_Z_NULL_BIAS_OFFSET;

	// success is if at least one of the sensors updated
	if ((!ad_data->ad_gyro.is_new) && (!ad_data->ad_accel.is_new)) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Read data from the Movella MTi-630 sensor
 * @param ctx pointer to the ctx storing the previously updated times for the sensors
 * @param imu_data Pointer to store the IMU data
 * @param curr_timestamp_ms the current time stamp for freshness calculations TODO
 * @return Status of the read operation
 */
static w_status_t read_movella_imu(sensor_handler_ctx_t *ctx, navigator_mti_meas_t *imu_data,
								   const uint32_t curr_timestamp_ms) {
	(void)curr_timestamp_ms;
	// Read all data from Movella in one call
	movella_data_t movella_data = {0}; // Initialize to zero

	w_status_t status = movella_get_data(&movella_data, 1);

	if (W_SUCCESS == status) {
		// Copy data from Movella
		// Apply orientation correction
		imu_data->mti_accel.meas =
			math_vector3d_rotate(&g_mti_correction_matrix, &movella_data.acc);
		imu_data->mti_gyro.meas = math_vector3d_rotate(&g_mti_correction_matrix, &movella_data.gyr);
		imu_data->mti_mag.meas = math_vector3d_rotate(&g_mti_correction_matrix, &movella_data.mag);

		imu_data->mti_baro.meas = movella_data.pres;

		// check freshness
		if ((movella_data.acc_timestamp_ms) > (ctx->last_mti_acc_timestamp_ms)) {
			imu_data->mti_accel.is_new = true;
			sensor_handler_state.mti_accel_stats.success_count++;

		} else {
			imu_data->mti_accel.is_new = false;
			sensor_handler_state.mti_accel_stats.failure_count++;
		}

		if ((movella_data.gyr_timestamp_ms) > (ctx->last_mti_gyr_timestamp_ms)) {
			imu_data->mti_gyro.is_new = true;
			sensor_handler_state.mti_gyro_stats.success_count++;

		} else {
			imu_data->mti_gyro.is_new = false;
			sensor_handler_state.mti_gyro_stats.failure_count++;
		}

		if ((movella_data.mag_timestamp_ms) > (ctx->last_mti_mag_timestamp_ms)) {
			imu_data->mti_mag.is_new = true;
			sensor_handler_state.mti_mag_stats.success_count++;

		} else {
			imu_data->mti_mag.is_new = false;
			sensor_handler_state.mti_mag_stats.failure_count++;
		}

		if ((movella_data.pres_timestamp_ms) > (ctx->last_mti_pres_timestamp_ms)) {
			imu_data->mti_baro.is_new = true;
			sensor_handler_state.mti_baro_stats.success_count++;

		} else {
			imu_data->mti_baro.is_new = false;
			sensor_handler_state.mti_baro_stats.failure_count++;
		}

		// update timestamps
		ctx->last_mti_acc_timestamp_ms = movella_data.acc_timestamp_ms;
		ctx->last_mti_gyr_timestamp_ms = movella_data.gyr_timestamp_ms;
		ctx->last_mti_mag_timestamp_ms = movella_data.mag_timestamp_ms;
		ctx->last_mti_pres_timestamp_ms = movella_data.pres_timestamp_ms;
	} else {
		log_text(
			1, LOG_LVL_WARN, "SensorHandler", "Movella get data read failed. CODE: %d", status);

		// Set is_new flag to indicate IMU failure
		imu_data->mti_accel.is_new = false;
		imu_data->mti_gyro.is_new = false;
		imu_data->mti_mag.is_new = false;
		imu_data->mti_baro.is_new = false;

		sensor_handler_state.mti_accel_stats.failure_count++;
		sensor_handler_state.mti_gyro_stats.failure_count++;
		sensor_handler_state.mti_mag_stats.failure_count++;
		sensor_handler_state.mti_baro_stats.failure_count++;
	}

	// if at least one sensor updated then it's successful
	if ((imu_data->mti_accel.is_new) || (imu_data->mti_gyro.is_new) || (imu_data->mti_mag.is_new) ||
		(imu_data->mti_baro.is_new)) {
		return W_SUCCESS;
	}
	return W_FAILURE;
}

/**
 * @brief Read data from the motor
 * @param ctx pointer to the ctx storing the previously updated times for the sensors
 * @param encoder_data Pointer to store the converted data
 * @param curr_timestamp_ms the current time stamp for freshness calculations TODO
 * @return Status of the read operation
 */
static w_status_t read_motor_meas(sensor_handler_ctx_t *ctx, navigator_1d_meas_t *encoder_data,
								  const uint32_t curr_timestamp_ms) {
	ak45_feedback_t motor_feedback = {0};
	w_status_t status = ak45_get_latest_feedback(&motor_feedback);

	if (W_SUCCESS == status) {
		if ((motor_feedback.timestamp_ms) - (ctx->last_motor_encoder_timestamp_ms) <=
			MOTOR_ENCODER_FRESHNESS_TIMEOUT_MS) {
			encoder_data->is_new = true;

			sensor_handler_state.motor_encoder_stats.success_count++;
		} else {
			encoder_data->is_new = false;

			sensor_handler_state.motor_encoder_stats.failure_count++;
		}

		// log any error codes
		if (motor_feedback.fault_code != AK45_FAULT_NONE) {
			log_text(1,
					 LOG_LVL_WARN,
					 "SensorHandler",
					 "Motor fault code: %d",
					 motor_feedback.fault_code);
		}

		// update timestamp
		ctx->last_motor_encoder_timestamp_ms = (motor_feedback.timestamp_ms);
	} else {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Motor Feedback failed. STATUS: %d", status);
		encoder_data->is_new = false;
		sensor_handler_state.motor_encoder_stats.failure_count++;
	}

	encoder_data->meas = (motor_feedback.position_deg) * RAD_PER_DEG;

	// success is if at least one of the sensors updated
	if ((!encoder_data->is_new)) {
		return W_FAILURE;
	}
	return W_SUCCESS;
}

/**
 * @brief Initialize the IMU handler module
 * @note This function is called before the scheduler starts
 * @return Status of initialization
 */
w_status_t sensor_handler_init(void) {
	// Set initialized flag directly here instead of calling initialize_all_imus()
	sensor_handler_state.initialized = true;

	if (orientation_calibrated != true) {
		log_text(1,
				 LOG_LVL_WARN,
				 "SensorHandler",
				 "Sensor orientation correction matrices not calibrated yet, using default "
				 "orientation.");
	}

	// Mailbox holding the latest telemetry snapshot for the telemetry task to peek.
	g_sensor_data_queue = xQueueCreate(1, sizeof(sensor_can_telem_data_t));
	if (NULL == g_sensor_data_queue) {
		log_text(1, LOG_LVL_FATAL, "SensorHandler", "Failed to create sensor data mailbox.");
		return W_FAILURE;
	}

	// Telemetry map: one entry per sensor reading to broadcast.
	// TODO: flight_phase_state and period_ms are placeholders — fill in the real phases and rates
	// (register a row per phase, as ak45_driver does).
	static const telemetry_source_config_t telemetry_sources[] = {
		// LSM6DSV32X (board IMU): accel + gyro together, 20Hz on pad filter, 10Hz in flight, 1hz
		// for idle
		{"Board IMU", board_imu_telemetry, STATE_IDLE, 1000 / 1},
		{"Board IMU", board_imu_telemetry, STATE_PAD_FILTER, 1000 / 20},
		{"Board IMU", board_imu_telemetry, STATE_PAD_NAV, 1000 / 10},
		{"Board IMU", board_imu_telemetry, STATE_BOOST, 1000 / 10},
		{"Board IMU", board_imu_telemetry, STATE_ACT_ALLOWED, 1000 / 10},

		// MS5611 (board barometer + thermometer)
		{"Board Baro", board_baro_telemetry, STATE_IDLE, 100},
		{"Board Baro", board_baro_telemetry, STATE_PAD_FILTER, 100},
		{"Board Baro", board_baro_telemetry, STATE_PAD_NAV, 100},
		{"Board Baro", board_baro_telemetry, STATE_ACT_ALLOWED, 100},
		{"Board Baro", board_baro_telemetry, STATE_BOOST, 100},

		// MTi-630 (Movella) accel+gyro+mag+baro plus LSM303AGR board mag (same rate),
		// 2Hz on pad/flight, 1Hz idle.
		{"MTI and board mag", mti_board_mag_telemetry, STATE_IDLE, 1000 / 1},
		{"MTI and board mag", mti_board_mag_telemetry, STATE_PAD_FILTER, 1000 / 2},
		{"MTI and board mag", mti_board_mag_telemetry, STATE_PAD_NAV, 1000 / 2},
		{"MTI and board mag", mti_board_mag_telemetry, STATE_BOOST, 1000 / 2},
		{"MTI and board mag", mti_board_mag_telemetry, STATE_ACT_ALLOWED, 1000 / 2},

		// AD breakout: ADXL380 accel + ADXRS649 gyro together, 10Hz pad/flight, 1Hz idle.
		{"AD", ad_telemetry, STATE_IDLE, 1000 / 1},
		{"AD", ad_telemetry, STATE_PAD_FILTER, 1000 / 10},
		{"AD", ad_telemetry, STATE_PAD_NAV, 1000 / 10},
		{"AD", ad_telemetry, STATE_BOOST, 1000 / 10},
		{"AD", ad_telemetry, STATE_ACT_ALLOWED, 1000 / 10},
	};

	static const size_t telemetry_source_count =
		sizeof(telemetry_sources) / sizeof(telemetry_source_config_t);
	w_status_t telemetry_register_status = W_SUCCESS;
	for (size_t i = 0; i < telemetry_source_count; i++) {
		telemetry_register_status |= telemetry_register(&telemetry_sources[i]);
	}
	if (W_SUCCESS != telemetry_register_status) {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Failed to register telemetry sources.");
		return W_FAILURE;
	}

	log_text(10, LOG_LVL_INFO, "SensorHandler", "Sensor Handler Initialized.");
	return W_SUCCESS;
}

w_status_t sensor_handler_get_fresh_meas(sensor_handler_ctx_t *ctx,
										 all_sensors_data_t *imu_output) {
	if ((NULL == imu_output) || (NULL == ctx)) {
		log_text(10, LOG_LVL_FATAL, "SensorHandler", "invalid ptrs.");
		return W_INVALID_PARAM;
	}

	// assume data are all dead until you read
	imu_output->ad_meas.ad_accel.is_new = false;
	imu_output->ad_meas.ad_gyro.is_new = false;

	imu_output->board_meas.board_baro.is_new = false;
	imu_output->board_meas.board_mag.is_new = false;
	imu_output->board_meas.board_imu.is_new = false;
	imu_output->motor_encoder_meas.is_new = false;

	// movella
	imu_output->mti_meas.mti_accel.is_new = false;
	imu_output->mti_meas.mti_gyro.is_new = false;
	imu_output->mti_meas.mti_mag.is_new = false;
	imu_output->mti_meas.mti_baro.is_new = false;

	// m/s^2, rad/s, pascals, mag is in gauss
	uint32_t current_time_ms;
	w_status_t status = W_SUCCESS;

	// raw data
	raw_board_meas_t raw_board_meas = {0};

	// Get current timestamp
	if (timer_get_ms(&current_time_ms) != W_SUCCESS) {
		current_time_ms = 0;
		log_text(1, LOG_LVL_FATAL, "SensorHandler", "Failed to get current time.");

		return W_FAILURE; // since without a timestamp the system will be unable to correctly judge
						  // any of the data therefore the results for all sensors are data
	}

	// Read from all IMUs and sensors
	w_status_t board_status =
		read_board_meas(ctx, &(imu_output->board_meas), &raw_board_meas, current_time_ms);
	w_status_t movella_status = read_movella_imu(ctx, &(imu_output->mti_meas), current_time_ms);
	w_status_t ad_status = read_ad_meas(ctx, &(imu_output->ad_meas), current_time_ms);
	w_status_t motor_status =
		read_motor_meas(ctx, &(imu_output->motor_encoder_meas), current_time_ms);

	// log system-level failures
	if (W_SUCCESS != movella_status) {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Read and Processing of Movella IMU failed.");
	}
	if (W_SUCCESS != board_status) {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Read and Processing of Board Sensors failed.");
	}
	if (W_SUCCESS != ad_status) {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Read and Processing of AD Sensors failed.");
	}
	if (W_SUCCESS != motor_status) {
		log_text(1, LOG_LVL_WARN, "SensorHandler", "Read and Processing of Motor Feedback failed.");
	}

	// TODO: add logging for board meas

	// Publish the latest telemetry snapshot to the mailbox for the telemetry task to broadcast.
	sensor_can_telem_data_t telem = {
		// board IMU / mag / baro are sent as raw register values (no scaling)
		.board_imu_accel = imu_output->board_meas.board_imu.accel,
		.board_imu_gyro = imu_output->board_meas.board_imu.gyro,
		.board_baro_pressure_pa = raw_board_meas.raw_board_baro.pressure_centimbar,
		// TODO: populate board barometer thermometer reading once wired
		.board_mag = imu_output->board_meas.board_mag.meas,
		.mti_accel = imu_output->mti_meas.mti_accel.meas,
		.mti_gyro = imu_output->mti_meas.mti_gyro.meas,
		.mti_mag = imu_output->mti_meas.mti_mag.meas,
		.mti_baro_pressure = (float32_t)imu_output->mti_meas.mti_baro.meas,
		.ad_accel = imu_output->ad_meas.ad_accel.meas,
		.ad_gyro = (float32_t)imu_output->ad_meas.ad_gyro.meas,
	};
	if (NULL != g_sensor_data_queue) {
		(void)xQueueOverwrite(g_sensor_data_queue, &telem);
	}

	// update queue with current IMU data for flight phase to read
	// now this is done by the updated output data

	sensor_handler_state.sample_count++;

	// Return overall status
	return status;
}

health_status_t sensor_handler_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log sampling statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "%s Sampling -Total: %lu, Errors: %lu",
			 sensor_handler_state.initialized ? "INIT" : "NOT INIT",
			 sensor_handler_state.sample_count,
			 sensor_handler_state.error_count);

	// Log IMU statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "Board IMU - Success %lu, Failure %lu Mag - Success %lu, Failure %lu Baro - Success "
			 "%lu, Failure %lu",
			 sensor_handler_state.board_imu_stats.success_count,
			 sensor_handler_state.board_imu_stats.failure_count,
			 sensor_handler_state.board_mag_stats.success_count,
			 sensor_handler_state.board_mag_stats.failure_count,
			 sensor_handler_state.board_baro_stats.success_count,
			 sensor_handler_state.board_baro_stats.failure_count);
	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "AD Accel - Success %lu, Failure %lu Gyro - Success %lu, Failure %lu",
			 sensor_handler_state.ad_accel_stats.success_count,
			 sensor_handler_state.ad_accel_stats.failure_count,
			 sensor_handler_state.ad_gyro_stats.success_count,
			 sensor_handler_state.ad_gyro_stats.failure_count);
	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "MTI Accel - Success %lu, Failure %lu Gyro - Success %lu, Failure %lu Mag - Success "
			 "%lu, Failure %lu Baro - Success %lu, Failure %lu",
			 sensor_handler_state.mti_accel_stats.success_count,
			 sensor_handler_state.mti_accel_stats.failure_count,
			 sensor_handler_state.mti_gyro_stats.success_count,
			 sensor_handler_state.mti_gyro_stats.failure_count,
			 sensor_handler_state.mti_mag_stats.success_count,
			 sensor_handler_state.mti_mag_stats.failure_count,
			 sensor_handler_state.mti_baro_stats.success_count,
			 sensor_handler_state.mti_baro_stats.failure_count);
	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "Motor Encoder - Success %lu, Failure %lu ",
			 sensor_handler_state.motor_encoder_stats.success_count,
			 sensor_handler_state.motor_encoder_stats.failure_count);

	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_SENSOR_HANDLER,
							  .error_bitfield = 0};

	return status;
}
