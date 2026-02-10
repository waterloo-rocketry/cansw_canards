/**
 * Types used by the Estimator internally
 */
#ifndef EST_TYPE_H
#define EST_TYPE_H

#include "common/math/math.h"

// size of the y_imu_t array
#define SIZE_IMU_ALL 10
// size of the x_state_t array (new name)
#define SIZE_STATE 13
// size of imu struct without accelerometer data, used during meas model before predictions
#define SIZE_IMU_MEAS 7

/*
 * Double state
 */

typedef union {
	double array[SIZE_STATE];

	struct {
		quaternion_t attitude;
		vector3d_t rates;
		vector3d_t velocity;
		double altitude;
		double CL;
		double delta;
	};
} x_state_t;

/*
 * Float state
 */
typedef union {
	float array[SIZE_STATE];

	struct {
		quaternion_f32_t attitude;
		vector3d_f32_t rates;
		vector3d_f32_t velocity;
		float altitude;
		float CL;
		float delta;
	};
} x_state_f32_t;

/*
 * Input signal
 */
typedef struct {
	double cmd;
	vector3d_t acceleration;
} u_dynamics_t;

/*
 * Measurement signals
 */
// sensor data from 1 arbitrary imu
// Pad filter: both filtered_i and bias_i can be of this type
// EKF: bias_i, y_i, and h_x_i are of this type
// IMU measurement model: the return is of this type
typedef union {
	double array[SIZE_IMU_ALL];

	struct {
		vector3d_t accelerometer;
		vector3d_t gyroscope;
		vector3d_t magnetometer;
		double barometer;
	};
} y_imu_t;

#endif