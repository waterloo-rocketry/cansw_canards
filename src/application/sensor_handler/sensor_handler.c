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
static const vector3d_t hard_iron_bias_board = {.x = 0.0, .y = 0.0, .z = 0.0};
static const vector3d_t hard_iron_bias_mti = {.x = 0.0, .y = 0.0, .z = 0.0};
static const matrix3d_t soft_iron_correction_matrix = {
	.array = {{1.00, 0.00, 0.00}, {0.00, 1.00, 0.00}, {0.00, 0.00, 1.00}}};

#ifdef ADBREAKOUT_09590
// TODO: Add actual calibration values for AD gyro
static const float64_t AD_GYRO_MV_OFFSET = 0.0;
static const float64_t AD_GYRO_RAD_PER_MV = RAD_PER_DEG * 10.0;

#elif defined(ADBREAKOUT_05127)

static const float64_t AD_GYRO_MV_OFFSET = 0.0;
static const float64_t AD_GYRO_RAD_PER_MV = RAD_PER_DEG * 10.0;

#else
static const float64_t AD_GYRO_MV_OFFSET = 0.0;
static const float64_t AD_GYRO_RAD_PER_MV = RAD_PER_DEG * 10.0;
#endif

// set to true once calibrated, initialized to false to prevent use before calibration
static bool orientation_calibrated = false;

// Timeout for log_data() calls from telemetry callbacks: never block the telemetry task.
#define SENSOR_LOG_TIMEOUT_MS 0

typedef struct {
	uint32_t success_count;
	uint32_t failure_count;
} sensor_health_state_t;

// Module state tracking
typedef struct {
	bool initialized;
	uint32_t sample_count;
	uint32_t log_data_fail_count;

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

	ak45_fault_code_t motor_fault_code;
} sensor_handler_state_t;

static sensor_handler_state_t sensor_handler_state = {0};

// All sensor values broadcast as telemetry, in the units/frames expected by the ground station.
// One flat snapshot so the telemetry task reads a single consistent set of values.
typedef struct {
	// LSM6DSV32X (board IMU)
	vector3d_t board_imu_accel; // m/s^2
	vector3d_t board_imu_gyro; // rad/s
	// MS5611 (board barometer)
	int32_t board_baro_pressure_centimbar;
	// TODO: add board barometer thermometer reading (board_baro_temp) once wired
	// LSM303AGR (board mag)
	vector3d_t board_mag;
	// MTi-630 (Movella)
	vector3d_t mti_accel; // m/s^2
	vector3d_t mti_gyro; // rad/s
	vector3d_t mti_mag; // gauss
	float32_t mti_baro_pressure; // Pa
	quaternion_f32_t mti_quaternion;

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
// Values are scaled/encoded via can_encode_scaled_* with the relevant SCALE_* factor.
// Encoded signed values are then biased into unsigned CAN payload fields using telemetry offsets.
// (Board + AD are scaled too; MTI uses SCALE_MTI_* as well.)
// ---------------------------------------------------------------------------

// LSM6DSV32X (board IMU): accelerometer + gyroscope
static w_status_t board_imu_ad_can_telemetry(void) {
	sensor_can_telem_data_t data = {0};
	w_status_t status = W_SUCCESS;

	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	uint32_t ts_ms = 0;
	if (timer_get_ms(&ts_ms) != W_SUCCESS) {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to get timestamp for can msg tx");
		return W_FAILURE;
	}

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
										 (uint16_t)(accel_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(accel_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(accel_z + TELEMETRY_INT16_OFFSET),
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
										 (uint16_t)(gyro_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(gyro_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(gyro_z + TELEMETRY_INT16_OFFSET),
										 &gyro_msg);
		status |= can_handler_transmit(&gyro_msg);

	} else {
		status |= W_FAILURE;
	}

	can_msg_t accel_msg = {0};
	w_status_t ad_accel_enc = W_SUCCESS;

	ad_accel_enc |=
		can_encode_scaled_float(SCALE_ADXL380_ACCEL, (float32_t)data.ad_accel.x, &accel_x);
	ad_accel_enc |=
		can_encode_scaled_float(SCALE_ADXL380_ACCEL, (float32_t)data.ad_accel.y, &accel_y);
	ad_accel_enc |=
		can_encode_scaled_float(SCALE_ADXL380_ACCEL, (float32_t)data.ad_accel.z, &accel_z);
	if (W_SUCCESS == ad_accel_enc) {
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_ADXL380_ACCEL,
										 (uint16_t)(accel_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(accel_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(accel_z + TELEMETRY_INT16_OFFSET),
										 &accel_msg);
		status |= can_handler_transmit(&accel_msg);
	} else {
		status |= W_FAILURE;
	}

	can_msg_t gyro_msg = {0};
	int32_t gyro_scaled = 0;
	w_status_t ad_gyro_enc =
		can_encode_scaled_float(SCALE_ADXRS649_GYROSCOPE, data.ad_gyro, &gyro_scaled);

	if (W_SUCCESS == ad_gyro_enc) {
		build_analog_sensor_32bit_msg(
			PRIO_LOW,
			(uint16_t)ts_ms,
			SENSOR_CANARD_ADXRS649_GYRO,
			(uint32_t)(gyro_scaled + TELEMETRY_INT32_OFFSET), // spans -+ 1000 so this is fine
			&gyro_msg);
		status |= can_handler_transmit(&gyro_msg);
	} else {
		status |= W_FAILURE;
	}

	return status;
}

// MS5611 (board barometer): raw pressure (Pa). Thermometer half not wired yet, sending 0.
static w_status_t board_baro_can_telemetry(void) {
	sensor_can_telem_data_t data = {0};
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	uint32_t ts_ms = 0;
	if (timer_get_ms(&ts_ms) != W_SUCCESS) {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to get timestamp for can msg tx");
		return W_FAILURE;
	}

	can_msg_t msg = {0};

	uint32_t baro_pres = 0;
	if (can_encode_scaled_int(SCALE_BOARD_PRESSURE,
							  (int64_t)data.board_baro_pressure_centimbar,
							  &baro_pres) != W_SUCCESS) {
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
static w_status_t mti_board_mag_can_telemetry(void) {
	sensor_can_telem_data_t data = {0};
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	w_status_t status = W_SUCCESS;

	uint32_t ts_ms = 0;
	if (timer_get_ms(&ts_ms) != W_SUCCESS) {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to get timestamp for can msg tx");
		return W_FAILURE;
	}

	w_status_t board_mag_enc = W_SUCCESS;

	// IIS2MDC board magnetometer - raw register counts, unscaled
	int16_t board_mag_x = 0;
	int16_t board_mag_y = 0;
	int16_t board_mag_z = 0;

	board_mag_enc |=
		can_encode_scaled_float(SCALE_BOARD_MAG, (float32_t)data.board_mag.x, &board_mag_x);
	board_mag_enc |=
		can_encode_scaled_float(SCALE_BOARD_MAG, (float32_t)data.board_mag.y, &board_mag_y);
	board_mag_enc |=
		can_encode_scaled_float(SCALE_BOARD_MAG, (float32_t)data.board_mag.z, &board_mag_z);
	if (W_SUCCESS == board_mag_enc) {
		can_msg_t board_mag_msg = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_IIS2MDC_MAG,
										 (uint16_t)(board_mag_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(board_mag_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(board_mag_z + TELEMETRY_INT16_OFFSET),
										 &board_mag_msg);
		status |= can_handler_transmit(&board_mag_msg);
	} else {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to encode board mag.");
		status |= W_FAILURE;
	}

	// MTi-630 accel / gyro / mag: scale to int16, then bias into the unsigned field
	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;
	w_status_t mti_accel_enc = W_SUCCESS;
	mti_accel_enc |=
		can_encode_scaled_float(SCALE_MTI_ACCEL, (float32_t)data.mti_accel.x, &accel_x);
	mti_accel_enc |=
		can_encode_scaled_float(SCALE_MTI_ACCEL, (float32_t)data.mti_accel.y, &accel_y);
	mti_accel_enc |=
		can_encode_scaled_float(SCALE_MTI_ACCEL, (float32_t)data.mti_accel.z, &accel_z);
	if (W_SUCCESS == mti_accel_enc) {
		can_msg_t mti_accel_msg = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_MTI630_ACCEL,
										 (uint16_t)(accel_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(accel_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(accel_z + TELEMETRY_INT16_OFFSET),
										 &mti_accel_msg);
		status |= can_handler_transmit(&mti_accel_msg);
	} else {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to encode MTI accel.");
		status |= W_FAILURE;
	}

	int16_t gyro_x = 0;
	int16_t gyro_y = 0;
	int16_t gyro_z = 0;
	w_status_t mti_gyro_enc = W_SUCCESS;
	mti_gyro_enc |= can_encode_scaled_float(SCALE_MTI_GYRO, (float32_t)data.mti_gyro.x, &gyro_x);
	mti_gyro_enc |= can_encode_scaled_float(SCALE_MTI_GYRO, (float32_t)data.mti_gyro.y, &gyro_y);
	mti_gyro_enc |= can_encode_scaled_float(SCALE_MTI_GYRO, (float32_t)data.mti_gyro.z, &gyro_z);
	if (W_SUCCESS == mti_gyro_enc) {
		can_msg_t mti_gyro_msg = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_MTI630_GYRO,
										 (uint16_t)(gyro_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(gyro_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(gyro_z + TELEMETRY_INT16_OFFSET),
										 &mti_gyro_msg);
		status |= can_handler_transmit(&mti_gyro_msg);
	} else {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to encode MTI gyro.");
		status |= W_FAILURE;
	}

	int16_t mag_x = 0;
	int16_t mag_y = 0;
	int16_t mag_z = 0;
	w_status_t mti_mag_enc = W_SUCCESS;
	mti_mag_enc |= can_encode_scaled_float(SCALE_MTI_MAG, (float32_t)data.mti_mag.x, &mag_x);
	mti_mag_enc |= can_encode_scaled_float(SCALE_MTI_MAG, (float32_t)data.mti_mag.y, &mag_y);
	mti_mag_enc |= can_encode_scaled_float(SCALE_MTI_MAG, (float32_t)data.mti_mag.z, &mag_z);
	if (W_SUCCESS == mti_mag_enc) {
		can_msg_t mti_mag_msg = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)ts_ms,
										 DEM_3D_SENSOR_CANARD_MTI630_MAG,
										 (uint16_t)(mag_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(mag_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(mag_z + TELEMETRY_INT16_OFFSET),
										 &mti_mag_msg);
		status |= can_handler_transmit(&mti_mag_msg);
	} else {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to encode MTI mag.");
		status |= W_FAILURE;
	}

	// MTi-630 barometric pressure (scaled, uint32)
	uint32_t baro_scaled = 0;
	w_status_t mti_baro_enc =
		can_encode_scaled_float(SCALE_MTI_PRESSURE, data.mti_baro_pressure, &baro_scaled);
	if (W_SUCCESS == mti_baro_enc) {
		can_msg_t mti_baro_msg = {0};
		build_analog_sensor_32bit_msg(
			PRIO_LOW, (uint16_t)ts_ms, SENSOR_CANARD_MTI630_BARO_0, baro_scaled, &mti_baro_msg);
		status |= can_handler_transmit(&mti_baro_msg);
	} else {
		log_text(0, LOG_LVL_WARN, "Sensor Handler", "Failed to encode MTI baro.");
		status |= W_FAILURE;
	}

	return status;
}

// ---------------------------------------------------------------------------
// Per-sensor SD data-log functions registered with the telemetry module.
// These write the same snapshot the CAN telemetry functions read,via log_data(). Timeout is 0 so
// they stay non-blocking as telemetry callbacks require.
// ---------------------------------------------------------------------------

// This will be all of the high rate logging sensors
// LSM6 Accel + Gyro
// AD Gyro
static w_status_t sensor_high_rate_sd_log(void) {
	sensor_can_telem_data_t data = {0};
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	// LSM6
	log_data_container_t container = {0};
	container.board_imu.accelerometer.x = (float32_t)data.board_imu_accel.x;
	container.board_imu.accelerometer.y = (float32_t)data.board_imu_accel.y;
	container.board_imu.accelerometer.z = (float32_t)data.board_imu_accel.z;
	container.board_imu.gyroscope.x = (float32_t)data.board_imu_gyro.x;
	container.board_imu.gyroscope.y = (float32_t)data.board_imu_gyro.y;
	container.board_imu.gyroscope.z = (float32_t)data.board_imu_gyro.z;

	w_status_t log_data_result = W_SUCCESS;

	if (log_data(SENSOR_LOG_TIMEOUT_MS, LOG_TYPE_BOARD_IMU, &container) != W_SUCCESS) {
		sensor_handler_state.log_data_fail_count++;
		log_data_result |= W_FAILURE;
	}

	// AD Gyro
	container.ad_breakout.accelerometer.x = (float32_t)data.ad_accel.x;
	container.ad_breakout.accelerometer.y = (float32_t)data.ad_accel.y;
	container.ad_breakout.accelerometer.z = (float32_t)data.ad_accel.z;
	container.ad_breakout.gyroscope = data.ad_gyro;

	if (log_data(SENSOR_LOG_TIMEOUT_MS, LOG_TYPE_AD_BREAKOUT, &container) != W_SUCCESS) {
		sensor_handler_state.log_data_fail_count++;
		log_data_result |= W_FAILURE;
	}

	// MTI Accel + Gyro
	container.movella_pt1.accelerometer.x = (float32_t)data.mti_accel.x;
	container.movella_pt1.accelerometer.y = (float32_t)data.mti_accel.y;
	container.movella_pt1.accelerometer.z = (float32_t)data.mti_accel.z;
	container.movella_pt1.gyroscope.x = (float32_t)data.mti_gyro.x;
	container.movella_pt1.gyroscope.y = (float32_t)data.mti_gyro.y;
	container.movella_pt1.gyroscope.z = (float32_t)data.mti_gyro.z;

	if (log_data(SENSOR_LOG_TIMEOUT_MS, LOG_TYPE_MOVELLA_PT1, &container) != W_SUCCESS) {
		sensor_handler_state.log_data_fail_count++;
		log_data_result |= W_FAILURE;
	}

	return log_data_result;
}

// Low rate data logging
// Board/MTI Baro/Mag
static w_status_t sensor_low_rate_sd_log(void) {
	sensor_can_telem_data_t data;
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	log_data_container_t container = {0};
	container.board_mag_baro.magnetometer.x = (float32_t)data.board_mag.x;
	container.board_mag_baro.magnetometer.y = (float32_t)data.board_mag.y;
	container.board_mag_baro.magnetometer.z = (float32_t)data.board_mag.z;
	container.board_mag_baro.barometer =
		(float32_t)(data.board_baro_pressure_centimbar * PA_PER_CENTIMBAR);
	container.board_mag_baro.thermometer = 0.0f;

	w_status_t log_data_result = W_SUCCESS;

	if (log_data(SENSOR_LOG_TIMEOUT_MS, LOG_TYPE_BOARD_MAG_BARO, &container) != W_SUCCESS) {
		sensor_handler_state.log_data_fail_count++;
		log_data_result |= W_FAILURE;
	}

	container.movella_pt2.magnetometer.x = (float32_t)data.mti_mag.x;
	container.movella_pt2.magnetometer.y = (float32_t)data.mti_mag.y;
	container.movella_pt2.magnetometer.z = (float32_t)data.mti_mag.z;
	container.movella_pt2.barometer = (float32_t)data.mti_baro_pressure;

	if (log_data(SENSOR_LOG_TIMEOUT_MS, LOG_TYPE_MOVELLA_PT2, &container) != W_SUCCESS) {
		sensor_handler_state.log_data_fail_count++;
		log_data_result |= W_FAILURE;
	}

	return log_data_result;
}

static w_status_t movella_state_sd_log(void) {
	sensor_can_telem_data_t data;
	if (W_SUCCESS != sensor_handler_get_latest(&data)) {
		return W_FAILURE;
	}

	log_data_container_t container = {0};
	container.movella_pt3.orient_w = (float32_t)data.mti_quaternion.w;
	container.movella_pt3.orient_x = (float32_t)data.mti_quaternion.x;
	container.movella_pt3.orient_y = (float32_t)data.mti_quaternion.y;
	container.movella_pt3.orient_z = (float32_t)data.mti_quaternion.z;

	w_status_t status = W_SUCCESS;
	if (log_data(SENSOR_LOG_TIMEOUT_MS, LOG_TYPE_MOVELLA_PT3, &container) != W_SUCCESS) {
		sensor_handler_state.log_data_fail_count++;
		status |= W_FAILURE;
	}

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

	w_status_t sensor_status = lsm6dsv32x_get_gyro_acc_data(&(board_data->board_imu.accel),
															&(board_data->board_imu.gyro),
															&(raw_data->raw_board_accel),
															&(raw_data->raw_board_gyro));

	// Read accelerometer and gyro data
	if (W_SUCCESS == sensor_status) {
		if ((raw_data->raw_board_accel.timestamp_ms) > (ctx->last_board_imu_timestamp_ms)) {
			board_data->board_imu.is_new = true;

			// convert gyro from dps to rad/sec
			board_data->board_imu.gyro.x = (board_data->board_imu.gyro.x) * RAD_PER_DEG;
			board_data->board_imu.gyro.y = (board_data->board_imu.gyro.y) * RAD_PER_DEG;
			board_data->board_imu.gyro.z = (board_data->board_imu.gyro.z) * RAD_PER_DEG;

			// convert accel from g to m/s^2
			board_data->board_imu.accel.x = (board_data->board_imu.accel.x) * M_S2_PER_G;
			board_data->board_imu.accel.y = (board_data->board_imu.accel.y) * M_S2_PER_G;
			board_data->board_imu.accel.z = (board_data->board_imu.accel.z) * M_S2_PER_G;

			// Apply orientation correction
			board_data->board_imu.accel = math_vector3d_rotate(&g_board_imu_correction_matrix,
															   &(board_data->board_imu.accel));
			board_data->board_imu.gyro =
				math_vector3d_rotate(&g_board_imu_correction_matrix, &(board_data->board_imu.gyro));

			sensor_handler_state.board_imu_stats.success_count++;
		} else {
			board_data->board_imu.is_new = false;

			sensor_handler_state.board_imu_stats.failure_count++;
		}

		// update timestamp
		ctx->last_board_imu_timestamp_ms = (raw_data->raw_board_accel.timestamp_ms);
	} else {
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

			// mag data is already provided in Gauss

			board_data->board_mag.meas =
				math_vector3d_rotate(&g_board_mag_correction_matrix, &(board_data->board_mag.meas));

			board_data->board_mag.meas =
				math_vector3d_subt(&(board_data->board_mag.meas), &hard_iron_bias_board);
			board_data->board_mag.meas =
				math_vector3d_rotate(&soft_iron_correction_matrix, &(board_data->board_mag.meas));

			sensor_handler_state.board_mag_stats.success_count++;
		} else {
			board_data->board_mag.is_new = false;

			sensor_handler_state.board_mag_stats.failure_count++;
		}

		ctx->last_mag_timestamp_ms = mag_timestamp_ms;
	} else {
		board_data->board_mag.is_new = false;

		sensor_handler_state.board_mag_stats.failure_count++;
	}

	// get baro
	uint32_t baro_timestamp_ms = 0;

	sensor_status = ms5611_get_raw_pressure(&(raw_data->raw_board_baro), &baro_timestamp_ms);
	if (W_SUCCESS == sensor_status) {
		if (baro_timestamp_ms > (ctx->last_baro_timestamp_ms)) {
			board_data->board_baro.is_new = true;

			// convert baro from mbar to Pascals
			board_data->board_baro.meas =
				((float64_t)(raw_data->raw_board_baro.pressure_centimbar)) * PA_PER_CENTIMBAR;
			sensor_handler_state.board_baro_stats.success_count++;
		} else {
			board_data->board_baro.is_new = false;

			sensor_handler_state.board_baro_stats.failure_count++;
		}

		ctx->last_baro_timestamp_ms = baro_timestamp_ms;
	} else {
		board_data->board_baro.is_new = false;

		sensor_handler_state.board_baro_stats.failure_count++;
	}

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

	// get accel
	uint32_t accel_timestamp_ms = 0;
	w_status_t accel_status =
		ad_breakout_board_get_accel_data(&(ad_data->ad_accel.meas), &(accel_timestamp_ms));

	// Read accelerometer and gyro data
	if (W_SUCCESS == accel_status) {
		if ((accel_timestamp_ms) > (ctx->last_ad_accel_timestamp_ms)) {
			ad_data->ad_accel.is_new = true;

			// convert accel from g to m/s^2
			ad_data->ad_accel.meas.x = (ad_data->ad_accel.meas.x) * M_S2_PER_G;
			ad_data->ad_accel.meas.y = (ad_data->ad_accel.meas.y) * M_S2_PER_G;
			ad_data->ad_accel.meas.z = (ad_data->ad_accel.meas.z) * M_S2_PER_G;

			// Apply orientation correction
			ad_data->ad_accel.meas =
				math_vector3d_rotate(&g_ad_accel_correction_matrix, &(ad_data->ad_accel.meas));

			sensor_handler_state.ad_accel_stats.success_count++;
		} else {
			ad_data->ad_accel.is_new = false;

			sensor_handler_state.ad_accel_stats.failure_count++;
		}

		// update timestamp
		ctx->last_ad_accel_timestamp_ms = accel_timestamp_ms;
	} else {
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

			// Apply gyro calibration
			ad_data->ad_gyro.meas =
				((ad_data->ad_gyro.meas) - AD_GYRO_MV_OFFSET) * AD_GYRO_RAD_PER_MV;

			sensor_handler_state.ad_gyro_stats.success_count++;

		} else {
			ad_data->ad_gyro.is_new = false;
			sensor_handler_state.ad_gyro_stats.failure_count++;
		}

		// update timestamp
		ctx->last_ad_gyro_timestamp_ms = gyro_timestamp_ms;
	} else {
		ad_data->ad_gyro.is_new = false;
		sensor_handler_state.ad_gyro_stats.failure_count++;
	}

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
 * @param quaternion_output pointer to store quaternion output for logging
 * @param curr_timestamp_ms the current time stamp for freshness calculations TODO
 * @return Status of the read operation
 */
static w_status_t read_movella_imu(sensor_handler_ctx_t *ctx, navigator_mti_meas_t *imu_data,
								   quaternion_f32_t *quaternion_output,
								   const uint32_t curr_timestamp_ms) {
	(void)curr_timestamp_ms;
	// Read all data from Movella in one call
	movella_data_t movella_data = {0}; // Initialize to zero

	w_status_t status = movella_get_data(&movella_data, 1);

	if (W_SUCCESS == status) {
		// Copy data from Movella
		memcpy(quaternion_output, &movella_data.quaternion, sizeof(*quaternion_output));
		// Apply orientation correction

		imu_data->mti_baro.meas = movella_data.pres;

		// check freshness
		if ((movella_data.acc_timestamp_ms) > (ctx->last_mti_acc_timestamp_ms)) {
			imu_data->mti_accel.meas =
				math_vector3d_rotate(&g_mti_correction_matrix, &movella_data.acc);
			imu_data->mti_accel.is_new = true;
			sensor_handler_state.mti_accel_stats.success_count++;

		} else {
			imu_data->mti_accel.is_new = false;
			sensor_handler_state.mti_accel_stats.failure_count++;
		}

		if ((movella_data.gyr_timestamp_ms) > (ctx->last_mti_gyr_timestamp_ms)) {
			imu_data->mti_gyro.is_new = true;
			imu_data->mti_gyro.meas =
				math_vector3d_rotate(&g_mti_correction_matrix, &movella_data.gyr);

			sensor_handler_state.mti_gyro_stats.success_count++;

		} else {
			imu_data->mti_gyro.is_new = false;
			sensor_handler_state.mti_gyro_stats.failure_count++;
		}

		if ((movella_data.mag_timestamp_ms) > (ctx->last_mti_mag_timestamp_ms)) {
			imu_data->mti_mag.is_new = true;
			imu_data->mti_mag.meas =
				math_vector3d_rotate(&g_mti_correction_matrix, &movella_data.mag);

			imu_data->mti_mag.meas =
				math_vector3d_subt(&(imu_data->mti_mag.meas), &hard_iron_bias_board);
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

			encoder_data->meas = (motor_feedback.position_deg) * RAD_PER_DEG;
			sensor_handler_state.motor_encoder_stats.success_count++;
		} else {
			encoder_data->is_new = false;

			sensor_handler_state.motor_encoder_stats.failure_count++;
		}

		// check for motor fault code
		if (motor_feedback.fault_code != AK45_FAULT_NONE) {
			sensor_handler_state.motor_encoder_stats.failure_count++;
			sensor_handler_state.motor_fault_code = motor_feedback.fault_code;
		}

		// update timestamp
		ctx->last_motor_encoder_timestamp_ms = (motor_feedback.timestamp_ms);
	} else {
		encoder_data->is_new = false;
		sensor_handler_state.motor_encoder_stats.failure_count++;
	}

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
		// LSM6DSV32X (board IMU)
		// for idle
		{"Board IMU", board_imu_ad_can_telemetry, STATE_IDLE, 1000 / 1},
		{"Board IMU", board_imu_ad_can_telemetry, STATE_PAD_FILTER, 1000 / 20},
		{"Board IMU", board_imu_ad_can_telemetry, STATE_PAD_NAV, 1000 / 10},
		{"Board IMU", board_imu_ad_can_telemetry, STATE_BOOST, 1000 / 10},
		{"Board IMU", board_imu_ad_can_telemetry, STATE_ACT_ALLOWED, 1000 / 10},

		// MS5611 (board barometer + thermometer)
		{"Board Baro", board_baro_can_telemetry, STATE_IDLE, 100},
		{"Board Baro", board_baro_can_telemetry, STATE_PAD_FILTER, 100},
		{"Board Baro", board_baro_can_telemetry, STATE_PAD_NAV, 100},
		{"Board Baro", board_baro_can_telemetry, STATE_ACT_ALLOWED, 100},
		{"Board Baro", board_baro_can_telemetry, STATE_BOOST, 100},

		// MTi-630 (Movella)
		{"MTI and board mag", mti_board_mag_can_telemetry, STATE_IDLE, 1000 / 1},
		{"MTI and board mag", mti_board_mag_can_telemetry, STATE_PAD_FILTER, 1000 / 2},
		{"MTI and board mag", mti_board_mag_can_telemetry, STATE_PAD_NAV, 1000 / 2},
		{"MTI and board mag", mti_board_mag_can_telemetry, STATE_BOOST, 1000 / 2},
		{"MTI and board mag", mti_board_mag_can_telemetry, STATE_ACT_ALLOWED, 1000 / 2},

		// --- SD log group(High Rate) ---
		{"Sensor High Rate", sensor_high_rate_sd_log, STATE_IDLE, 1000 / 1},
		{"Sensor High Rate", sensor_high_rate_sd_log, STATE_SLEEPY, 1000 / 1},
		{"Sensor High Rate", sensor_high_rate_sd_log, STATE_RECOVERY, 1000 / 20},
		{"Sensor High Rate", sensor_high_rate_sd_log, STATE_PAD_FILTER, 1000 / 20},
		{"Sensor High Rate", sensor_high_rate_sd_log, STATE_PAD_NAV, 1000 / MAX_LOGGING_RATE_HZ},
		{"Sensor High Rate", sensor_high_rate_sd_log, STATE_BOOST, 1000 / MAX_LOGGING_RATE_HZ},
		{"Sensor High Rate",
		 sensor_high_rate_sd_log,
		 STATE_ACT_ALLOWED,
		 1000 / MAX_LOGGING_RATE_HZ},

		// --- SD log group (Low Rate) ---
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_IDLE, 1000 / 1},
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_SLEEPY, 1000 / 1},
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_RECOVERY, 1000 / 20},
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_PAD_FILTER, 1000 / 20},
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_PAD_NAV, 1000 / 50},
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_BOOST, 1000 / 50},
		{"Sensor Low Rate", sensor_low_rate_sd_log, STATE_ACT_ALLOWED, 1000 / 50},

		// --- SD log group (Movella State) ---
		{"Movella State", movella_state_sd_log, STATE_IDLE, 1000 / 1},
		{"Movella State", movella_state_sd_log, STATE_SLEEPY, 1000 / 1},
		{"Movella State", movella_state_sd_log, STATE_RECOVERY, 1000 / 20},
		{"Movella State", movella_state_sd_log, STATE_PAD_FILTER, 1000 / 20},
		{"Movella State", movella_state_sd_log, STATE_PAD_NAV, 1000 / 20},
		{"Movella State", movella_state_sd_log, STATE_BOOST, 1000 / 20},
		{"Movella State", movella_state_sd_log, STATE_ACT_ALLOWED, 1000 / 20},
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
	quaternion_f32_t mti_quaternion = {0};
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
	w_status_t movella_status =
		read_movella_imu(ctx, &(imu_output->mti_meas), &mti_quaternion, current_time_ms);
	w_status_t ad_status = read_ad_meas(ctx, &(imu_output->ad_meas), current_time_ms);
	w_status_t motor_status =
		read_motor_meas(ctx, &(imu_output->motor_encoder_meas), current_time_ms);

	if ((movella_status != W_SUCCESS) && (board_status != W_SUCCESS) && (ad_status != W_SUCCESS) &&
		(motor_status != W_SUCCESS)) {
		status = W_FAILURE;
	} else {
		status = W_SUCCESS;
	}

	// Publish the latest telemetry snapshot to the mailbox for the telemetry task to broadcast.
	sensor_can_telem_data_t telem = {
		// board IMU / mag / baro are sent as converted values
		.board_imu_accel = imu_output->board_meas.board_imu.accel,
		.board_imu_gyro = imu_output->board_meas.board_imu.gyro,
		.board_baro_pressure_centimbar = raw_board_meas.raw_board_baro.pressure_centimbar,
		// TODO: populate board barometer thermometer reading once wired
		.board_mag = imu_output->board_meas.board_mag.meas,

		.mti_accel = imu_output->mti_meas.mti_accel.meas,
		.mti_gyro = imu_output->mti_meas.mti_gyro.meas,
		.mti_mag = imu_output->mti_meas.mti_mag.meas,
		.mti_baro_pressure = (float32_t)imu_output->mti_meas.mti_baro.meas,
		.mti_quaternion = mti_quaternion,

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
			 "%s Sampling -Total: %lu",
			 sensor_handler_state.initialized ? "INIT" : "NOT INIT",
			 sensor_handler_state.sample_count);

	// Log sensor statistics

	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "Board IMU fail: %lu, "
			 "Board Mag fail: %lu, "
			 "Board Baro fail: %lu",
			 sensor_handler_state.board_imu_stats.failure_count,
			 sensor_handler_state.board_mag_stats.failure_count,
			 sensor_handler_state.board_baro_stats.failure_count);

	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "AD Accel fail: %lu, "
			 "AD Gyro fail: %lu",
			 sensor_handler_state.ad_accel_stats.failure_count,
			 sensor_handler_state.ad_gyro_stats.failure_count);

	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "MTI Accel fail: %lu, "
			 "MTI Gyro fail: %lu, "
			 "MTI Mag fail: %lu, "
			 "MTI Baro fail: %lu",
			 sensor_handler_state.mti_accel_stats.failure_count,
			 sensor_handler_state.mti_gyro_stats.failure_count,
			 sensor_handler_state.mti_mag_stats.failure_count,
			 sensor_handler_state.mti_baro_stats.failure_count);

	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "Motor fail: %lu",
			 sensor_handler_state.motor_encoder_stats.failure_count);

	log_text(0,
			 LOG_LVL_INFO,
			 "SensorHandler",
			 "Log data fail: %lu",
			 sensor_handler_state.log_data_fail_count);

	if (sensor_handler_state.motor_fault_code != AK45_FAULT_NONE) {
		log_text(0,
				 LOG_LVL_WARN,
				 "SensorHandler",
				 "Motor fault code: %d",
				 sensor_handler_state.motor_fault_code);
	}

	sensor_handler_state.motor_fault_code = AK45_FAULT_NONE;

	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_SENSOR_HANDLER,
							  .error_bitfield = 0};

	return status;
}
