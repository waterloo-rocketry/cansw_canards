#include <string.h>

#include "application/can_handler/can_handler.h"
#include "application/estimator/estimator_types.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

// Period of IMU sampling in milliseconds
// slightly slower than 200 hz to always receive encoder which can be >5ms
static const int IMU_SAMPLING_PERIOD_MS = 6;

// Timeout values for freshness check (in milliseconds)
static const int GYRO_FRESHNESS_TIMEOUT_MS = 5;
static const int MAG_FRESHNESS_TIMEOUT_MS = 10;
static const int ACCEL_FRESHNESS_TIMEOUT_MS = 5;
static const int BARO_FRESHNESS_TIMEOUT_MS = 25;
static const int ERROR_THRESHOLD = 10;

// Rate limit CAN tx: only send data at 10Hz, every 100ms
static const int IMU_HANDLER_CAN_TX_PERIOD_MS = 100;
static const int IMU_HANDLER_CAN_TX_RATE = (IMU_HANDLER_CAN_TX_PERIOD_MS / IMU_SAMPLING_PERIOD_MS);

// correct orientation from finn irl, may 4 2025
// also default uncalibrated orientation until calibration module sets these
// S1 (movella)
static const matrix3d_t g_movella_upd_mat = {
	.array = {{0, 0, 1.000000000}, {1.0000000, 0, 0}, {0, 1.0000000000, 0}}};
// S2 (pololu)
static const matrix3d_t g_pololu_upd_mat = {
	.array = {{0, 0, -1.00000000}, {-1.00000000000, 0, 0}, {0, 1.00000000000, 0}}};

// set to true once calibrated, initialized to false to prevent use before calibration
static bool orientation_calibrated = false;

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
 * @brief Read data from the pololu AltIMU-10 sensor
 * @param imu_data Pointer to store the converted data
 * @param raw_data Pointer to store the raw data
 * @return Status of the read operation
 */
static w_status_t read_pololu_imu(estimator_imu_measurement_t *imu_data,
								  raw_pololu_data_t *raw_data) {
	w_status_t status = W_SUCCESS;

	// Read accelerometer, gyro, and magnetometer data
	status |= altimu_get_gyro_acc_data(
		&imu_data->accelerometer, &imu_data->gyroscope, &raw_data->raw_acc, &raw_data->raw_gyro);
	status |= altimu_get_mag_data(&imu_data->magnetometer, &raw_data->raw_mag);

	// Read barometer data
	altimu_barometer_data_t baro_data;
	status |= altimu_get_baro_data(&baro_data, &raw_data->raw_baro);

	if (W_SUCCESS == status) {
		// convert gyro from dps to rad/sec
		imu_data->gyroscope.x = imu_data->gyroscope.x * RAD_PER_DEG;
		imu_data->gyroscope.y = imu_data->gyroscope.y * RAD_PER_DEG;
		imu_data->gyroscope.z = imu_data->gyroscope.z * RAD_PER_DEG;

		// convert accel from g to m/s^2
		imu_data->accelerometer.x = imu_data->accelerometer.x * 9.81;
		imu_data->accelerometer.y = imu_data->accelerometer.y * 9.81;
		imu_data->accelerometer.z = imu_data->accelerometer.z * 9.81;

		// Apply orientation correction
		imu_data->accelerometer =
			math_vector3d_rotate(&g_pololu_upd_mat, &(imu_data->accelerometer));
		imu_data->gyroscope = math_vector3d_rotate(&g_pololu_upd_mat, &(imu_data->gyroscope));
		imu_data->magnetometer = math_vector3d_rotate(&g_pololu_upd_mat, &(imu_data->magnetometer));

		imu_data->barometer = baro_data.pressure;
		imu_data->is_dead = false;
		imu_handler_state.pololu_stats.success_count++;
	} else {
		// Set is_dead flag to indicate IMU failure
		imu_data->is_dead = true;
		imu_handler_state.pololu_stats.failure_count++;
	}

	return status;
}

/**
 * @brief Read data from the Movella MTi-630 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_movella_imu(estimator_imu_measurement_t *imu_data) {
	w_status_t status;

	// Read all data from Movella in one call
	movella_data_t movella_data = {0}; // Initialize to zero
	status = movella_get_data(&movella_data, 1);

	if (W_SUCCESS == status) {
		// Copy data from Movella
		// Apply orientation correction
		imu_data->accelerometer = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.acc);
		imu_data->gyroscope = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.gyr);
		imu_data->magnetometer = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.mag);

		imu_data->barometer = movella_data.pres;
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

	if (orientation_calibrated != true) {
		log_text(1,
				 "IMUHandler",
				 "WARN: IMU orientation correction matrices not calibrated yet, using default "
				 "orientation.");
	}

	log_text(10, "IMUHandler", "INFO: IMU Handler Initialized.");
	return W_SUCCESS;
}

w_status_t imu_handler_get_fresh_meas(all_sensors_data_t *imu_output) {
	if (NULL == imu_output) {
		log_text(10, "IMUHandler", "ERROR: get fresh meas invalid output ptr.");
		return W_INVALID_PARAM;
	}

	// is this even necessary at all, since this assumes success before any process
	// replacing original declaration
	imu_output->movella.is_dead = false;
	imu_output->pololu.is_dead = false;

	raw_pololu_data_t raw_pololu_data = {0};
	uint32_t current_time_ms;
	w_status_t status = W_SUCCESS;

	// Get current timestamp
	if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
		current_time_ms = 0;
	}
	uint32_t now_ms = (uint32_t)current_time_ms;

	// TODO: update this with new IMU for correct behaviour
	// Set timestamps for all IMUs
	// Note: All IMUs get the same timestamp intentionally for synchronization
	imu_output->pololu.timestamp_imu_sec = ((float64_t)now_ms) / 1000.0;
	imu_output->movella.timestamp_imu_sec = ((float64_t)now_ms) / 1000.0;

	// Read from all IMUs, including orientation correction
	w_status_t pololu_status = read_pololu_imu(&(imu_output->pololu), &raw_pololu_data);
	w_status_t movella_status = read_movella_imu(&(imu_output->movella));

	// If both IMUs fail, consider it a system-level failure
	if ((W_FAILURE == pololu_status) && (W_FAILURE == movella_status)) {
		log_text(1, "IMUHandler", "ERROR: Both pololu and Movella IMU reads failed.");
		status = W_FAILURE;
	} else if (W_FAILURE == pololu_status) {
		log_text(1, "IMUHandler", "WARN: pololu IMU read failed.");
	} else if (W_FAILURE == movella_status) {
		log_text(1, "IMUHandler", "WARN: Movella IMU read failed.");
	}

	// Log movella data as seperate messages

	log_data_container_t log_payload = {0}; //{.imu_reading = imu_data.movella};

	log_payload.imu_reading_pt1.accelerometer.x = (float)imu_output->movella.accelerometer.x;
	log_payload.imu_reading_pt1.accelerometer.y = (float)imu_output->movella.accelerometer.y;
	log_payload.imu_reading_pt1.accelerometer.z = (float)imu_output->movella.accelerometer.z;
	log_data(1, LOG_TYPE_MOVELLA_READING_PT1, &log_payload);

	log_payload.imu_reading_pt2.gyroscope.x = (float)imu_output->movella.gyroscope.x;
	log_payload.imu_reading_pt2.gyroscope.y = (float)imu_output->movella.gyroscope.y;
	log_payload.imu_reading_pt2.gyroscope.z = (float)imu_output->movella.gyroscope.z;
	log_data(1, LOG_TYPE_MOVELLA_READING_PT2, &log_payload);

	log_payload.imu_reading_pt3.magnetometer.x = (float)imu_output->movella.magnetometer.x;
	log_payload.imu_reading_pt3.magnetometer.y = (float)imu_output->movella.magnetometer.y;
	log_payload.imu_reading_pt3.magnetometer.z = (float)imu_output->movella.magnetometer.z;

	log_payload.imu_reading_pt3.barometer = imu_output->movella.barometer;
	log_payload.imu_reading_pt3.timestamp_imu_ms =
		(uint32_t)(imu_output->movella.timestamp_imu_sec * 1000);
	log_payload.imu_reading_pt3.is_dead = imu_output->movella.is_dead;
	log_data(1, LOG_TYPE_MOVELLA_READING_PT3, &log_payload);

	// Log polulu data as seperate messages

	log_payload.imu_reading_pt1.accelerometer.x = (float)imu_output->pololu.accelerometer.x;
	log_payload.imu_reading_pt1.accelerometer.y = (float)imu_output->pololu.accelerometer.y;
	log_payload.imu_reading_pt1.accelerometer.z = (float)imu_output->pololu.accelerometer.z;
	log_data(1, LOG_TYPE_POLOLU_READING_PT1, &log_payload);

	log_payload.imu_reading_pt2.gyroscope.x = (float)imu_output->pololu.gyroscope.x;
	log_payload.imu_reading_pt2.gyroscope.y = (float)imu_output->pololu.gyroscope.y;
	log_payload.imu_reading_pt2.gyroscope.z = (float)imu_output->pololu.gyroscope.z;
	log_data(1, LOG_TYPE_POLOLU_READING_PT2, &log_payload);

	log_payload.imu_reading_pt3.magnetometer.x = (float)imu_output->pololu.magnetometer.x;
	log_payload.imu_reading_pt3.magnetometer.y = (float)imu_output->pololu.magnetometer.y;
	log_payload.imu_reading_pt3.magnetometer.z = (float)imu_output->pololu.magnetometer.z;

	log_payload.imu_reading_pt3.barometer = imu_output->pololu.barometer;
	log_payload.imu_reading_pt3.timestamp_imu_ms =
		(uint32_t)(imu_output->pololu.timestamp_imu_sec * 1000);
	log_payload.imu_reading_pt3.is_dead = imu_output->pololu.is_dead;
	log_data(1, LOG_TYPE_POLOLU_READING_PT3, &log_payload);

	// Log raw pololu data

	log_payload.raw_pololu_data_pt1.raw_acc = raw_pololu_data.raw_acc;
	log_payload.raw_pololu_data_pt1.raw_gyro = raw_pololu_data.raw_gyro;
	log_data(1, LOG_TYPE_POLOLU_RAW_PT1, &log_payload);

	log_payload.raw_pololu_data_pt2.raw_mag = raw_pololu_data.raw_mag;
	log_payload.raw_pololu_data_pt2.raw_baro = raw_pololu_data.raw_baro;
	log_data(1, LOG_TYPE_POLOLU_RAW_PT2, &log_payload);

	// update queue with current IMU data for flight phase to read
	// now this is done by the updated output data

	imu_handler_state.sample_count++;

	// Return overall status
	return status;
}

health_status_t imu_handler_get_status(void) {
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

	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_IMU_HANDLER, .error_bitfield = 0};

	return status;
}
