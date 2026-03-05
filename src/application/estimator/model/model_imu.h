#ifndef MODEL_IMU_H
#define MODEL_IMU_H

#include "application/estimator/estimator_types.h"

/**
 * @brief Computes measurement prediction using current state and sensor biases
 * @param x_state_t pointer to estimator state
 * @param y_imu_t pointer to sensor biases
 * @return y_imu_t measurement prediction
 */
y_imu_t model_measurement_imu(const x_state_t *state, const y_imu_t *imu_bias);

/**
 * @brief jacobian of the measurement model
 * @param imu_jacobian 7x13 jacobian matrix flat data to write to
 * @param x_state_t pointer to estimator state
 * @param y_imu_t pointer to sensor biases
 */
void model_measurement_imu_jacobian(
    double imu_jacobian[SIZE_IMU_MEAS * SIZE_STATE], const x_state_t *state, const y_imu_t *imu_bias
);

#endif
