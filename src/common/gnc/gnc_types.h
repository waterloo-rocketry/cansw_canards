// TODO: this file should be cleaned up during gnc impl this year

/**
 * Navigator and controller types used by the whole gnc system
 */
#ifndef GNC_TYPES_H
#define GNC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "application/fsm/fsm.h"
#include "common/math/math.h"
#include "navigation_codegen_entry.h"
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

// All of the units are follows m/s^2, rad/s, Pa, gauss
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
} navigator_imu_meas_t;

/**
 * @brief LSM6DSV32X (32G IMU), MS5611 (High-alt Barometer), and LSM303AGR (Compass) measurements
 */
typedef struct {
	navigator_imu_meas_t board_imu; // m/s^2 and rad/s
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

// Codegen Type Rename
typedef struct1_T navigator_codegen_bias_t;
typedef struct2_T navigator_codegen_sensor_filter_t;
typedef struct3_T navigator_codegen_sensor_input_t;

typedef struct4_T navigator_codegen_sensor_3D_t;
typedef struct5_T navigator_codegen_sensor_1D_t;

typedef struct6_T airdata_codegen_t;

// ---------- NAVIGATOR TYPES ----------
typedef struct {
	uint32_t curr_timestamp_tenth_ms;
	fsm_state_t fsm_state;
} navigator_input_t;

typedef struct {
	uint32_t timestamp_tenth_ms;
	double cov_norm;
	// TODO: update after Tristan update codegen
	airdata_codegen_t airdata;
	double roll_state[2];
} navigator_output_t;

// ---------- CONTROLLER TYPES ----------

// output from navigator aka input to controller
typedef struct {
	float64_t motor_angle_rad; /// delta
	float64_t xR[2];
	float64_t pdyn;
	uint32_t curr_timestamp_ms;
	uint32_t launch_timestamp_ms;
} controller_input_t;

// Output of controller: latest commanded canard angle
typedef struct {
	float64_t motor_command_angle_rad; // radians
	float64_t ref_roll_angle_rad;
	uint32_t timestamp_ms; // ms
} controller_output_t;

typedef struct {
	navigator_codegen_bias_t bias;
	navigator_codegen_sensor_filter_t sensor_filter;
	float64_t x[11];
	float64_t P[121];
} navigator_codegen_ctx_t;

// codegen controller
typedef struct0_T controller_codegen_ctx_t;

#endif
