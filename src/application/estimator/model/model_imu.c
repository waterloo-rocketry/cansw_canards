// include paths
#include "application/estimator/model/model_imu.h"
#include "application/estimator/estimator_types.h"

#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"

// computes measurement prediction using current state and sensror biases
y_imu_t model_measurement_imu(const x_state_t *state, const y_imu_t *imu_bias) {
	y_imu_t measurement_prediction = {0};

	// calculate rates
	const vector3d_t W = math_vector3d_add(&state->rates, &imu_bias->gyroscope); // W = w + b_W;

	// magnetic field model
	const matrix3d_t S = quaternion_rotmatrix(&state->attitude);
	const vector3d_t M = math_vector3d_rotate(&S, &imu_bias->magnetometer);

	// atmosphere model
	const estimator_airdata_t airdata = model_airdata(state->altitude);
	const double P = airdata.pressure + imu_bias->barometer;

	// final measurement prediction
	measurement_prediction.gyroscope = W;
	measurement_prediction.magnetometer = M;
	measurement_prediction.barometer = P;

	return measurement_prediction;
}

// jacobian of the measurement model
void model_measurement_imu_jacobian(double pData_imu_jacobian[SIZE_IMU_MEAS * SIZE_STATE],
									const x_state_t *state, const y_imu_t *imu_bias) {
	// rates
	const matrix3d_t W_w = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

	// magnetic field model
	rotation_jacobian_t M_q = {0};
	quaternion_rotate_jacobian(&M_q.flat[0], &state->attitude, &imu_bias->magnetometer);

	// atmosphere model
	const double P_alt = model_airdata_jacobian(state->altitude);

	// measurement prediction
	write_pData(pData_imu_jacobian,
				0,
				4,
				SIDE_MATRIX_3D,
				SIDE_MATRIX_3D,
				&W_w.flat[0]); // J(1:3, 5:7) = W_w;
	write_pData(
		pData_imu_jacobian, 3, 0, SIZE_VECTOR_3D, SIZE_QUAT, &M_q.flat[0]); // J(4:6, 1:4) = M_q;
	write_pData(pData_imu_jacobian, 6, 10, SIZE_1D, SIZE_1D, &P_alt); // J(7, 11) = P_alt;
}
