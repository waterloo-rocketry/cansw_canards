/**
 * Quaternion math operations
 */
#ifndef QUATERNION_H
#define QUATERNION_H

#include "common/math/math.h"

// Norm of a quaternion
double quaternion_norm(const quaternion_t *q);

// Normalize quaternion
quaternion_t quaternion_normalize(const quaternion_t *q);

// Invert attitude quaternion
quaternion_t quaternion_invert(const quaternion_t *q);

// Quaternion multiplication
quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2);

// Compute rotation matrix from a quaternion
matrix3d_t quaternion_rotmatrix(const quaternion_t *q);

// Quaternion time update, using the derivative + explicit euler
quaternion_t quaternion_update(const quaternion_t *q, const vector3d_t *rates, double dt);

// inverse of quaternion
quaternion_t quaternion_inverse(const quaternion_t *q);

/**
 * jacobians of quaternion operations
 */

// Jacobian of the rotation wrt to the quaternion
// Rotation: vector3d_t rotated = math_vector3d_rotate(quaternion_rotmatrix(quaternion_t),
// vector3d_t) output is a flattened array with 3 rows, 4 cols
void quaternion_rotate_jacobian(double R_q[SIZE_VECTOR_3D * SIZE_QUAT], const quaternion_t *q,
								const vector3d_t *vector);

// Jacobian of the time update wrt to the quaternion q_q, and wrt to the rates q_w
// quaternion_t qnew = quaternion_update(quaternion_t *q, vector3d_t *w);
// Output are flattened arrays with: q_new_q 4 rows and 4 cols, q_new_w 4 rows and 3 cols
void quaternion_update_jacobian(double q_new_q[SIZE_QUAT * SIZE_QUAT],
								double q_new_w[SIZE_QUAT * SIZE_VECTOR_3D], const quaternion_t *q,
								const vector3d_t *rates, const double dt);

/**
 * Discrete update of quaternion differential equation
 */

// quaternion_t quaternion_increment(const quaternion_t *q, const vector3d_t *omega, double deltaT);
// Compute Euler angles from a quaternion
vector3d_t quaternion_to_euler(const quaternion_t *q);

// Compute Euler angle roll from a quaternion
double quaternion_to_roll(const quaternion_t *q);

#endif
