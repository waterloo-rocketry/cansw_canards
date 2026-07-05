/**
 * Navigator and controller types used by the whole gnc system
 */
#ifndef GNC_TYPES_H
#define GNC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "GNC_codegen_types.h"
#include "application/fsm/fsm.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"

// ---------- SENSOR TYPES ----------

// Units are as follows: m/s^2, rad/s, Pa, gauss.
typedef struct {
	float64_t meas;
	bool is_new;
} navigator_1d_meas_t;

typedef struct {
	vector3d_t meas;
	bool is_new;
} navigator_3d_meas_t;

typedef struct {
	vector3d_t accel;
	vector3d_t gyro;
	bool is_new;
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
	navigator_3d_meas_t mti_accel; // m/s^2
	navigator_3d_meas_t mti_gyro; // rad/s
	navigator_1d_meas_t mti_baro; // Pa
	navigator_3d_meas_t mti_mag; // gauss
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
	navigator_1d_meas_t motor_encoder_meas;
} all_sensors_data_t;

// Codegen Type Rename
typedef struct1_T gnc_navigator_bias_t;
typedef struct2_T gnc_navigator_sensor_filter_t;
typedef struct3_T gnc_navigator_sensor_input_t;

typedef struct4_T gnc_navigator_sensor_3D_t;
typedef struct5_T gnc_navigator_sensor_1D_t;

// ---------- NAVIGATOR TYPES ----------
typedef struct {
	all_sensors_data_t *sensor_data;
	fsm_state_t fsm_state;
} navigator_input_t;

typedef struct {
	uint32_t timestamp_tenth_ms;
	double cov_norm;
	double roll_state[2];
	double dynamic_pressure; // dynamic pressure
} navigator_output_t;

// ---------- CONTROLLER TYPES ----------

// output from navigator aka input to controller
typedef struct {
	float64_t canard_angle_rad; /// delta
	float64_t xR[2];
	float64_t dynamic_pressure;
	uint32_t curr_timestamp_tenth_ms;
	uint32_t launch_timestamp_ms;
} controller_input_t;

// Output of controller: latest commanded canard angle
typedef struct {
	float64_t canard_command_angle_rad; // radians
	float64_t ref_roll[2]; // {roll angle (rad), roll rate (rad/s)}
	uint32_t timestamp_tenth_ms;
} controller_output_t;

typedef union {
	float64_t arr[11];

	struct {
		quaternion_t q;
		vector3d_t ang_rate;
		vector3d_t vel;
		float64_t altitude;
	};
} gnc_x_state_t;

typedef struct {
	gnc_navigator_bias_t bias;
	gnc_navigator_sensor_filter_t sensor_filter;
	gnc_x_state_t x;
	float64_t P[121];
} gnc_navigator_ctx_t;

// codegen controller
typedef struct0_T gnc_controller_ctx_t;

#endif
