#include <string.h>

#include "application/can_handler/can_handler.h"
#include "application/estimator/estimator_types.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

// conversion factors
static const float64_t M_S2_PER_G = 9.81;

// TODO: double check values with Tristan

// Timeout values for freshness check (in milliseconds)
static const int32_t ST_IMU_FRESHNESS_TIMEOUT_MS = 2;
static const int32_t AD_GYRO_FRESHNESS_TIMEOUT_MS = 2;
static const int32_t MAG_FRESHNESS_TIMEOUT_MS = 5;
static const int32_t AD_ACCEL_FRESHNESS_TIMEOUT_MS = 5;
static const int32_t BARO_FRESHNESS_TIMEOUT_MS = 5;
static const int32_t ERROR_THRESHOLD = 10;

// Rate limit CAN tx: only send data at 10Hz, every 100ms
static const uint32_t IMU_HANDLER_CAN_TX_PERIOD_MS = 100;
static const uint32_t IMU_HANDLER_CAN_TX_RATE =
	(IMU_HANDLER_CAN_TX_PERIOD_MS / ST_IMU_FRESHNESS_TIMEOUT_MS);

// TODO: add calibration matrix for this year
static const matrix3d_t g_mti_correction_matrix = {
	.array = {{0, 0, 1.000000000}, {1.0000000, 0, 0}, {0, 1.0000000000, 0}}};
static const matrix3d_t g_board_imu_correction_matrix = {
	.array = {{0, 0, -1.00000000}, {-1.00000000000, 0, 0}, {0, 1.00000000000, 0}}};
static const matrix3d_t g_board_mag_correction_matrix = {
	.array = {{0, 0, -1.00000000}, {-1.00000000000, 0, 0}, {0, 1.00000000000, 0}}};
static const matrix3d_t g_ad_accel_correction_matrix = {
	.array = {{0, 0, -1.00000000}, {-1.00000000000, 0, 0}, {0, 1.00000000000, 0}}};
static const matrix3d_t g_ad_gyro_correction_matrix = {
	.array = {{0, 0, -1.00000000}, {-1.00000000000, 0, 0}, {0, 1.00000000000, 0}}};

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
	sensor_health_state_t board_stats;
	sensor_health_state_t ad_stats;
	sensor_health_state_t movella_stats;
} imu_handler_state_t;

static imu_handler_state_t imu_handler_state = {0};

// static w_status_t log_raw_to_can(raw_pololu_data_t *raw_data) {
// 	// Log raw data to CAN
// 	can_msg_t msg;
// 	uint32_t timestamp = 0;
// 	timer_get_ms(&timestamp);
// 	w_status_t encode_status = W_SUCCESS;
// 	w_status_t can_tx_status = W_SUCCESS;

// 	// TODO: Currently using incorrect sensor for testing
// 	// Encode messages
// 	int16_t acc_x = 0, acc_y = 0, acc_z = 0;
// 	int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
// 	int32_t mag_x = 0, mag_y = 0, mag_z = 0;

// 	// TODO: do CAN scaling and sending

// 	can_tx_status |= can_handler_transmit(&msg);

// 	// Error handling
// 	if (encode_status == W_MATH_ERROR) {
// 		log_text(0, "IMUHandler", "IMU raw msg encode math error (NaN or Inf)");
// 	} else if (encode_status != W_SUCCESS) {
// 		log_text(0, "IMUHandler", "IMU raw msg scale / encode failed");
// 	}

// 	if (can_tx_status != W_SUCCESS) {
// 		log_text(0, "IMUHandler", "IMU raw msg tx failed");
// 	}

// 	if ((can_tx_status != W_SUCCESS) || (encode_status != W_SUCCESS)) {
// 		imu_handler_state.error_count++;
// 		return W_FAILURE;
// 	}
// 	return W_SUCCESS;
// }

/**
 * @brief Read data from the board
 * @param board_data Pointer to store the converted data
 * @param raw_data Pointer to store the raw data
 * @param curr_timestamp_ms the current time stamp for freshness calculations
 * @return Status of the read operation
 */
static w_status_t read_board_meas(imu_handler_ctx_t *ctx, navigator_board_meas_t *board_data,
								  raw_board_meas_t *raw_data, const uint32_t curr_timestamp_ms) {
	bool is_dead = true;

	// Read accelerometer and gyro data
	if (lsm6dsv32x_get_gyro_acc_data(&(board_data->board_imu.accel),
									 &(board_data->board_imu.gyro),
									 &(raw_data->raw_board_accel),
									 &(raw_data->raw_board_gyro),
									 &is_dead) == W_SUCCESS) {
		// check if dead
		if (is_dead) {
			if ((raw_data->raw_board_accel.timestamp_ms) >
				(ctx->last_board_imu_timestamp_ms)) { // designed to make sure no overflow
				board_data->board_imu.is_new = true;
			} else {
				board_data->board_imu.is_new = false;
			}

		} else {
			log_text(1, "IMUHandler", "WARN: Board IMU failed.");
			board_data->board_imu.is_new = false;
		}

		// update timestamp
		ctx->last_board_imu_timestamp_ms = raw_data->raw_board_accel.timestamp_ms;
	} else {
		log_text(1, "IMUHandler", "WARN: Board IMU read failed.");
		board_data->board_imu.is_new = false;
	}

	// get mag.
	uint32_t mag_timestamp_ms = 0;
	if (iis2mdc_get_data(
			&(board_data->board_mag.meas), &(raw_data->board_mag), &mag_timestamp_ms, &is_dead) ==
		W_SUCCESS) {
		// check if dead
		if (is_dead) {
			if (mag_timestamp_ms > (ctx->last_mag_timestamp_ms)) {
				board_data->board_mag.is_new = true;
			} else {
				board_data->board_mag.is_new = false;
			}

		} else {
			log_text(1, "IMUHandler", "WARN: Board Mag failed.");
			board_data->board_mag.is_new = false;
		}

		ctx->last_mag_timestamp_ms = mag_timestamp_ms;
	} else {
		log_text(1, "IMUHandler", "WARN: Board Mag read failed.");
		board_data->board_mag.is_new = false;
	}

	// get baro
	// TODO: once baro implemented

	// convert gyro from dps to rad/sec
	board_data->board_imu.gyro.x = board_data->board_imu.gyro.x * RAD_PER_DEG;
	board_data->board_imu.gyro.y = board_data->board_imu.gyro.y * RAD_PER_DEG;
	board_data->board_imu.gyro.z = board_data->board_imu.gyro.z * RAD_PER_DEG;

	// convert accel from g to m/s^2
	board_data->board_imu.accel.x = board_data->board_imu.accel.x * M_S2_PER_G;
	board_data->board_imu.accel.y = board_data->board_imu.accel.y * M_S2_PER_G;
	board_data->board_imu.accel.z = board_data->board_imu.accel.z * M_S2_PER_G;

	// mag data is already provided in Gauss

	// Apply orientation correction
	board_data->board_imu.accel =
		math_vector3d_rotate(&g_board_imu_correction_matrix, &(board_data->board_imu.accel));
	board_data->board_imu.gyro =
		math_vector3d_rotate(&g_board_imu_correction_matrix, &(board_data->board_imu.gyro));

	board_data->board_mag.meas =
		math_vector3d_rotate(&g_board_mag_correction_matrix, &(board_data->board_mag.meas));

	// TODO: baro

	// success is if at least one of the sensors updated
	if ((board_data->board_mag.is_new) || (board_data->board_imu.is_new)) {
		imu_handler_state.board_stats.success_count++;
	} else {
		imu_handler_state.board_stats.failure_count++;
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Read data from the Movella MTi-630 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_movella_imu(imu_handler_ctx_t *ctx, navigator_mti_meas_t *imu_data) {
	// Read all data from Movella in one call
	movella_data_t movella_data = {0}; // Initialize to zero
	w_status_t status = movella_get_data(&movella_data, 1);

	if ((W_SUCCESS == status) || (movella_data.is_dead)) {
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
		} else {
			imu_data->mti_accel.is_new = false;
		}

		if ((movella_data.gyr_timestamp_ms) > (ctx->last_mti_gyr_timestamp_ms)) {
			imu_data->mti_gyro.is_new = true;
		} else {
			imu_data->mti_gyro.is_new = false;
		}

		if ((movella_data.mag_timestamp_ms) > (ctx->last_mti_mag_timestamp_ms)) {
			imu_data->mti_mag.is_new = true;
		} else {
			imu_data->mti_mag.is_new = false;
		}

		if ((movella_data.pres_timestamp_ms) > (ctx->last_mti_pres_timestamp_ms)) {
			imu_data->mti_baro.is_new = true;
		} else {
			imu_data->mti_baro.is_new = false;
		}

		// update timestamps
		ctx->last_mti_acc_timestamp_ms = movella_data.acc_timestamp_ms;
		ctx->last_mti_gyr_timestamp_ms = movella_data.gyr_timestamp_ms;
		ctx->last_mti_mag_timestamp_ms = movella_data.mag_timestamp_ms;
		ctx->last_mti_pres_timestamp_ms = movella_data.pres_timestamp_ms;

		imu_handler_state.movella_stats.success_count++;
	} else {
		if (W_SUCCESS != status) {
			log_text(1, "IMUHandler", "WARN: Movella get data read failed.");
		} else {
			log_text(1, "IMUHandler", "WARN: Movella communication failed.");
		}

		// Set is_new flag to indicate IMU failure
		imu_data->mti_accel.is_new = false;
		imu_data->mti_gyro.is_new = false;
		imu_data->mti_mag.is_new = false;
		imu_data->mti_baro.is_new = false;
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

	if (orientation_calibrated != true) {
		log_text(1,
				 "IMUHandler",
				 "WARN: IMU orientation correction matrices not calibrated yet, using default "
				 "orientation.");
	}

	log_text(10, "IMUHandler", "INFO: IMU Handler Initialized.");
	return W_SUCCESS;
}

w_status_t imu_handler_get_fresh_meas(imu_handler_ctx_t *ctx, all_sensors_data_t *imu_output) {
	if (NULL == imu_output) {
		log_text(10, "IMUHandler", "ERROR: get fresh meas invalid output ptr.");
		return W_INVALID_PARAM;
	}

	// assume data are all dead until you read
	imu_output->ad_meas.ad_accel.is_new = false;
	imu_output->ad_meas.ad_gyro.is_new = false;
	imu_output->board_meas.board_baro.is_new = false;
	imu_output->board_meas.board_mag.is_new = false;
	imu_output->board_meas.board_imu.is_new = false;

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
	if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
		current_time_ms = 0;
	}

	// Read from all IMUs and sensors
	w_status_t board_status =
		read_board_meas(ctx, &(imu_output->board_meas), &raw_board_meas, current_time_ms);
	w_status_t movella_status = read_movella_imu(ctx, &(imu_output->mti_meas));
	// TODO: add AD data

	// log system-level failures
	if (W_SUCCESS != movella_status) {
		log_text(1, "IMUHandler", "WARN: Movella IMU read failed.");
	}
	if (W_SUCCESS != board_status) {
		log_text(1, "IMUHandler", "WARN: Board Sensors read failed.");
	}

	// TODO: add logging for board meas

	// update queue with current IMU data for flight phase to read
	// now this is done by the updated output data

	imu_handler_state.sample_count++;

	// Return overall status
	return status;
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
			 "Board Sensor Success %lu, Failure %lu Movella - Success %lu, Failure %lu",
			 imu_handler_state.board_stats.success_count,
			 imu_handler_state.board_stats.failure_count,
			 imu_handler_state.movella_stats.success_count,
			 imu_handler_state.movella_stats.failure_count);

	return status_bitfield;
}
