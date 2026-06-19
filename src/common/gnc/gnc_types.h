// TODO: this file should be cleaned up during gnc impl this year

/**
 * Navigator and controller types used by the whole gnc system
 */
#ifndef GNC_TYPES_H
#define GNC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "application/controller/gain_table.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"

// ---------- SENSOR TYPES ----------

// measurement data from 1 arbitrary imu
// TODO: remove old impl of `estimator_imu_measurement_t`
typedef struct {
	float64_t timestamp_imu_sec;
	vector3d_t accelerometer; // gravities
	vector3d_t gyroscope; // rad/sec
	vector3d_t magnetometer; // mgauss (pololu) or arbitrary units (movella)
	float32_t barometer; // Pa
	bool is_dead;
} estimator_imu_measurement_t;

// Units are as follows: m/s^2, rad/s, Pa, gauss.
typedef struct {
	float64_t meas;
	bool is_dead;
} navigator_1d_meas_t;

typedef struct {
	vector3d_t meas;
	bool is_dead;
} navigator_3d_meas_t;

typedef struct {
	vector3d_t accel;
	vector3d_t gyro;
	bool is_dead;
} navigator_board_imu_meas_t;

/**
 * @brief LSM6DSV32X (32G IMU), MS5611 (High-alt Barometer), and LSM303AGR (Compass) measurements
 */
typedef struct {
	navigator_board_imu_meas_t board_imu; // m/s^2 and rad/s
	navigator_1d_meas_t board_baro; // Pa
	navigator_3d_meas_t board_mag; // gauss
} navigator_board_meas_t;

/**
 * @brief MTi-630 (Movella) measurements
 */
typedef struct {
	vector3d_t mti_accel; // m/s^2
	vector3d_t mti_gyro; // rad/s
	float64_t mti_baro; // Pa
	vector3d_t mti_mag; // gauss
	bool is_dead;
} navigator_mti_meas_t;

/**
 * @brief ADXL380 (AD Breakout Accel) and ADXRS649 (AD high-rate gyro) measurements
 */
typedef struct {
	navigator_3d_meas_t ad_accel; // m/s^2
	navigator_1d_meas_t ad_gyro; // rad/s, 1 axis gyro
} navigator_ad_meas_t;

// measurements from all imus together
typedef struct {
	navigator_board_meas_t board_meas;
	navigator_mti_meas_t mti_meas;
	navigator_ad_meas_t ad_meas;

	// TODO: remove old impl below
	estimator_imu_measurement_t movella; // raw movella data
	estimator_imu_measurement_t pololu; // raw pololu data
} all_sensors_data_t;

// ---------- NAVIGATOR TYPES ----------
typedef struct {
	// TODO
} navigator_input_t;

typedef struct {
	// TODO
} navigator_output_t;

// ---------- CONTROLLER TYPES ----------
#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1) // subtract 1 for the pre-gain
#define ROLL_STATE_NUM (FEEDBACK_GAIN_NUM)
#define MIN_COOR_BOUND 0

typedef union {
	double roll_state_arr[ROLL_STATE_NUM];

	struct {
		double roll_angle;
		double roll_rate;
		double canard_angle;
	};
} roll_state_t;

// output from navigator aka input to controller
typedef struct {
	// Roll state
	roll_state_t roll_state;
	// Scheduling variables (flight condition)
	double pressure_dynamic;
	double canard_coeff;
} controller_input_t;

// Output of controller: latest commanded canard angle
typedef struct {
	double commanded_angle; // radians
	uint32_t timestamp; // ms
} controller_output_t;

#endif
