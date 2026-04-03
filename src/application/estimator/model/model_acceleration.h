#ifndef MODEL_ACCELERATION_H
#define MODEL_ACCELERATION_H

#include "application/estimator/estimator_types.h"
#include "common/math/math.h"

/**
 * @brief Computes the average acceleration from two IMUs, accounting for their deadness.
 * @param state input
 * @param IMU_1 movella imu input
 * @param is_dead_1 true if movella is dead
 * @param IMU_2 altimu input
 * @param is_dead_2 true if altimu is dead
 * @return average acceleration vector
 */
vector3d_t model_acceleration(const x_state_t *state, const y_imu_t *IMU_1, const bool is_dead_1,
							  const y_imu_t *IMU_2, const bool is_dead_2);

#endif // MODEL_ACCELERATION_H
