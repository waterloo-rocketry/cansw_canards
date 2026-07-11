#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "application/sensor_handler/sensor_handler.h"
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

	// latest motor feedback auxiliaries, kept for servo current/temp telemetry
	float32_t last_motor_current_a;
	int8_t last_motor_temp_c;
} sensor_handler_state_t;

static sensor_handler_state_t sensor_handler_state = {0};

// Scale one float into the 16-bit builder argument via can_encode_scaled_float. The encoder
// writes the native scaled integer into the temporary; reading it back and casting recovers
// the value for the canlib builder (which owns the big-endian packing). Only valid for
// 16-bit target tags (INT16/UINT16), which write exactly 2 bytes. On failure the status is
// recorded into *acc (so the caller can log once) and 0 is returned.
static uint16_t scale_to_u16(can_scaling_types_t sensor, float32_t value, w_status_t *acc) {
	uint16_t tmp = 0;
	w_status_t status = can_encode_scaled_float(sensor, value, &tmp);
	if (W_SUCCESS != status) {
		*acc = status;
	}
	return tmp;
}

// Scale one float into the 32-bit builder argument (see scale_to_u16). Only valid for 32-bit
// target tags (INT32/UINT32), which write exactly 4 bytes.
static uint32_t scale_to_u32(can_scaling_types_t sensor, float32_t value, w_status_t *acc) {
	uint32_t tmp = 0;
	w_status_t status = can_encode_scaled_float(sensor, value, &tmp);
	if (W_SUCCESS != status) {
		*acc = status;
	}
	return tmp;
}

/**
 * @brief Scale and broadcast the raw MTI + servo telemetry over CAN.
 * Each value is scaled via the can_telemetry_scaling table and packed into the
 * corresponding canlib canard telemetry message. Sends are gated on the per-sensor is_new
 * flags so stale data is not retransmitted.
 * @note MTI_EST (orientation/angular-velocity/velocity) is NOT broadcast: the Movella euler
 * estimate is not plumbed into all_sensors_data_t and the driver exposes no velocity. Board
 * (LSM6/IIS2MDC/ADXL380) and AD sensors are likewise not wired here yet.
 */
static w_status_t send_sensor_telemetry(const all_sensors_data_t *data) {
	uint32_t timestamp_ms = 0;
	(void)timer_get_ms(&timestamp_ms);
	uint16_t ts = (uint16_t)timestamp_ms;

	can_msg_t msg = {0};
	w_status_t enc = W_SUCCESS;
	w_status_t tx = W_SUCCESS;

	// MTI raw accel / gyro / mag (3D int16)
	if (data->mti_meas.mti_accel.is_new) {
		const vector3d_t *v = &data->mti_meas.mti_accel.meas;
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 ts,
										 DEM_3D_SENSOR_CANARD_MTI630_ACCEL,
										 scale_to_u16(SCALE_MTI_ACCEL, v->x, &enc),
										 scale_to_u16(SCALE_MTI_ACCEL, v->y, &enc),
										 scale_to_u16(SCALE_MTI_ACCEL, v->z, &enc),
										 &msg);
		tx |= can_handler_transmit(&msg);
	}
	if (data->mti_meas.mti_gyro.is_new) {
		const vector3d_t *v = &data->mti_meas.mti_gyro.meas;
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 ts,
										 DEM_3D_SENSOR_CANARD_MTI630_GYRO,
										 scale_to_u16(SCALE_MTI_GYRO, v->x, &enc),
										 scale_to_u16(SCALE_MTI_GYRO, v->y, &enc),
										 scale_to_u16(SCALE_MTI_GYRO, v->z, &enc),
										 &msg);
		tx |= can_handler_transmit(&msg);
	}
	if (data->mti_meas.mti_mag.is_new) {
		const vector3d_t *v = &data->mti_meas.mti_mag.meas;
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 ts,
										 DEM_3D_SENSOR_CANARD_MTI630_MAG,
										 scale_to_u16(SCALE_MTI_MAG, v->x, &enc),
										 scale_to_u16(SCALE_MTI_MAG, v->y, &enc),
										 scale_to_u16(SCALE_MTI_MAG, v->z, &enc),
										 &msg);
		tx |= can_handler_transmit(&msg);
	}

	// MTI barometric pressure (uint32)
	if (data->mti_meas.mti_baro.is_new) {
		build_analog_sensor_32bit_msg(PRIO_LOW,
									  ts,
									  SENSOR_CANARD_MTI630_BARO_0,
									  scale_to_u32(SCALE_MTI_PRESSURE,
												   (float32_t)data->mti_meas.mti_baro.meas,
												   &enc),
									  &msg);
		tx |= can_handler_transmit(&msg);
	}

	// Servo angle / current / temperature (int16 / uint16)
	if (data->motor_encoder_meas.is_new) {
		build_analog_sensor_16bit_msg(PRIO_LOW,
									  ts,
									  SENSOR_CANARD_SERVO_ANGLE,
									  scale_to_u16(SCALE_SERVO_ANGLE,
												   (float32_t)data->motor_encoder_meas.meas,
												   &enc),
									  &msg);
		tx |= can_handler_transmit(&msg);

		build_analog_sensor_16bit_msg(
			PRIO_LOW,
			ts,
			SENSOR_CANARD_SERVO_CURR,
			scale_to_u16(SCALE_SERVO_CURRENT, sensor_handler_state.last_motor_current_a, &enc),
			&msg);
		tx |= can_handler_transmit(&msg);

		build_analog_sensor_16bit_msg(
			PRIO_LOW,
			ts,
			SENSOR_CANARD_SERVO_TEMP,
			scale_to_u16(SCALE_SERVO_TEMP, (float32_t)sensor_handler_state.last_motor_temp_c, &enc),
			&msg);
		tx |= can_handler_transmit(&msg);
	}

	if (W_SUCCESS != enc) {
		log_text(0, LOG_LVL_WARN, "SensorHandler", "telemetry scale/encode failed: %d", enc);
	}
	if (W_SUCCESS != tx) {
		log_text(0, LOG_LVL_WARN, "SensorHandler", "telemetry tx failed");
		sensor_handler_state.error_count++;
	}
	return ((W_SUCCESS == enc) && (W_SUCCESS == tx)) ? W_SUCCESS : W_FAILURE;
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
		if ((motor_feedback.timestamp_ms) > (ctx->last_motor_encoder_timestamp_ms)) {
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

	// stash current/temperature for servo telemetry (not part of the 1D encoder meas)
	sensor_handler_state.last_motor_current_a = motor_feedback.current_a;
	sensor_handler_state.last_motor_temp_c = motor_feedback.temperature_c;

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

	// scale and broadcast the raw MTI + servo telemetry over CAN
	(void)send_sensor_telemetry(imu_output);

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

	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_SENSOR_HANDLER, .error_bitfield = 0};

	return status;
}
