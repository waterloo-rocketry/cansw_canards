/**
 * Quaternion math operations
 */
#include "application/estimator/model/quaternion.h"
#include "common/math/math.h"
#include <math.h>

// Norm of a quaternion
double quaternion_norm(const quaternion_t *q) {
	if ((q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z) < 0.0000001) {
		return 0;
	} else {
		return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	}
}

// Normalize quaternion
quaternion_t quaternion_normalize(const quaternion_t *q) {
	double norm = quaternion_norm(q);
	quaternion_t result = {0};
	if (norm > 0.0000001) {
		result.w = q->w / norm;
		result.x = q->x / norm;
		result.y = q->y / norm;
		result.z = q->z / norm;
	} else {
	}

	return result;
}

// Quaternion multiplication
quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2) {
	quaternion_t result;

	result.w = q2->w * q1->w - q2->x * q1->x - q2->y * q1->y - q2->z * q1->z;
	result.x = q2->w * q1->x + q2->x * q1->w - q2->y * q1->z + q2->z * q1->y;
	result.y = q2->w * q1->y + q2->x * q1->z + q2->y * q1->w - q2->z * q1->x;
	result.z = q2->w * q1->z - q2->x * q1->y + q2->y * q1->x + q2->z * q1->w;

	return result;
}

// Rotation matrix from quaternion: point rotation/active transformation
matrix3d_t quaternion_rotmatrix(const quaternion_t *q_unnormed) {
	quaternion_t q = quaternion_normalize(q_unnormed);

	matrix3d_t S;

	// top row
	S.s11 = 1 - 2 * (q.y * q.y + q.z * q.z);
	S.s12 = 2 * (q.x * q.y + q.w * q.z);
	S.s13 = 2 * (q.x * q.z - q.w * q.y);
	// middle row
	S.s21 = 2 * (q.x * q.y - q.w * q.z);
	S.s22 = 1 - 2 * (q.x * q.x + q.z * q.z);
	S.s23 = 2 * (q.y * q.z + q.w * q.x);
	// bottom row
	S.s31 = 2 * (q.x * q.z + q.w * q.y);
	S.s32 = 2 * (q.y * q.z - q.w * q.x);
	S.s33 = 1 - 2 * (q.x * q.x + q.y * q.y);

	return S;
}

// Quaternion time update, using the derivative + explicit euler
// combination of quaternion derivative, scale, add, normalize
quaternion_t quaternion_update(const quaternion_t *q, const vector3d_t *omega, double dt) {
	// quaternion derivative
	quaternion_t q_normed = quaternion_normalize(q);

	quaternion_t omega_q = {.array = {0, 0.5 * omega->x, 0.5 * omega->y, 0.5 * omega->z}};

	quaternion_t q_dot = quaternion_multiply(&q_normed, &omega_q);

	// scale and add
	quaternion_t q_new;
	q_new.w = q_normed.w + (dt)*q_dot.w;
	q_new.x = q_normed.x + (dt)*q_dot.x;
	q_new.y = q_normed.y + (dt)*q_dot.y;
	q_new.z = q_normed.z + (dt)*q_dot.z;

	q_normed = quaternion_normalize(&q_new);
	return q_normed;
}

// Jacobian of the rotation wrt to the quaternion
// Rotation: vector3d_t rotated = math_vector3d_rotate(quaternion_rotmatrix(quaternion_t),
// vector3d_t) output is a flattened array with 3 rows, 4 cols
// 2D coor (x, y) flattening to 1D coor (index): index = x * num_col + y (all zero-indexed)
void quaternion_rotate_jacobian(double R_q[SIZE_VECTOR_3D * SIZE_QUAT],
								const quaternion_t *q_unnormed, const vector3d_t *v) {
	quaternion_t q = quaternion_normalize(q_unnormed);

	// R_q = 2 * [qw*v - qv x v, qv'*v*I + qv*v' - v*qv' + qw*v_tilde]
	R_q[0] = 2 * (q.w * v->x - q.y * v->z + q.z * v->y);
	R_q[1] = 2 * (q.x * v->x + q.y * v->y + q.z * v->z);
	R_q[2] = 2 * (q.x * v->y - q.w * v->z - q.y * v->x);
	R_q[3] = 2 * (q.w * v->y + q.x * v->z - q.z * v->x);

	R_q[4] = 2 * (q.w * v->y + q.x * v->z - q.z * v->x);
	R_q[5] = 2 * (q.w * v->z - q.x * v->y + q.y * v->x);
	R_q[6] = 2 * (q.x * v->x + q.y * v->y + q.z * v->z);
	R_q[7] = 2 * (q.y * v->z - q.w * v->x - q.z * v->y);

	R_q[8] = 2 * (q.w * v->z - q.x * v->y + q.y * v->x);
	R_q[9] = 2 * (q.z * v->x - q.x * v->z - q.w * v->y);
	R_q[10] = 2 * (q.w * v->x - q.y * v->z + q.z * v->y);
	R_q[11] = 2 * (q.x * v->x + q.y * v->y + q.z * v->z);

	return;
}

// Jacobian of the time update wrt to the quaternion q_q, and wrt to the rates q_w
// quaternion_t qnew = quaternion_update(quaternion_t *q, vector3d_t *w);
// Output are flattened arrays with: q_new_q 4 rows and 4 cols, q_new_w 4 rows and 3 cols in row
// major order
// 2D coor (x, y) flattening to 1D coor (index): index = x * num_col + y (all zero-indexed)
void quaternion_update_jacobian(double q_new_q[SIZE_QUAT * SIZE_QUAT],
								double q_new_w[SIZE_QUAT * SIZE_VECTOR_3D],
								const quaternion_t *q_un, const vector3d_t *w, double dt) {
	quaternion_t q = quaternion_normalize(q_un);

	// q_new partial q (4x4)
	q_new_q[0] = 1;
	q_new_q[1] = -0.5 * (dt)*w->x;
	q_new_q[2] = -0.5 * (dt)*w->y;
	q_new_q[3] = -0.5 * (dt)*w->z;
	q_new_q[4] = 0.5 * (dt)*w->x;
	q_new_q[5] = 1;
	q_new_q[6] = 0.5 * (dt)*w->z;
	q_new_q[7] = -0.5 * (dt)*w->y;
	q_new_q[8] = 0.5 * (dt)*w->y;
	q_new_q[9] = -0.5 * (dt)*w->z;
	q_new_q[10] = 1;
	q_new_q[11] = 0.5 * (dt)*w->x;
	q_new_q[12] = 0.5 * (dt)*w->z;
	q_new_q[13] = 0.5 * (dt)*w->y;
	q_new_q[14] = -0.5 * (dt)*w->x;
	q_new_q[15] = 1;

	// q_new partial rates (4x3)
	q_new_w[0] = -0.5 * (dt)*q.x;
	q_new_w[1] = -0.5 * (dt)*q.y;
	q_new_w[2] = -0.5 * (dt)*q.z;
	q_new_w[3] = 0.5 * (dt)*q.w;
	q_new_w[4] = -0.5 * (dt)*q.z;
	q_new_w[5] = 0.5 * (dt)*q.y;
	q_new_w[6] = 0.5 * (dt)*q.z;
	q_new_w[7] = 0.5 * (dt)*q.w;
	q_new_w[8] = -0.5 * (dt)*q.x;
	q_new_w[9] = -0.5 * (dt)*q.y;
	q_new_w[10] = 0.5 * (dt)*q.x;
	q_new_w[11] = 0.5 * (dt)*q.w;

	return;
}

// !! this is possibly incorrect !!
// // Approximate solution of quaternion differential equation (truncation of Taylor expansion)
// quaternion_t quaternion_increment(const quaternion_t *q, const vector3d_t *omega, double deltaT)
// {
//     quaternion_t q_normed = quaternion_normalize(q);

//     quaternion_t omega_q = {.array = {0, omega->x, omega->y, omega->z}};
//     double omega_norm = quaternion_norm(&omega_q);

//     // incremental quaternion difference
//     double dphi = 0.5 * omega_norm * deltaT;
//     quaternion_t dq = quaternion_normalize(&omega_q);
//     dq.w = cos(dphi);
//     dq.x = omega_q.x * sin(dphi);
//     dq.y = omega_q.y * sin(dphi);
//     dq.z = omega_q.z * sin(dphi);

//     // update quaternion attitude
//     quaternion_t q_new = quaternion_multiply(&q_normed, &dq);

//     return q_new;
// }

// Compute Euler angles from a quaternion
vector3d_t quaternion_to_euler(const quaternion_t *q) {
	vector3d_t euler = {.array = {0, 0, 0}};

	// yaw angle
	euler.z = atan2(2 * (q->x * q->y + q->w * q->z),
					(q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z));
	// pitch angle
	euler.y = asin(-2 * (q->x * q->z - q->w * q->y));
	// roll angle

	euler.x = atan2(2 * (q->y * q->z + q->w * q->x),
					(q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z));

	return euler;
}

// Compute Euler angle roll from a quaternion
double quaternion_to_roll(const quaternion_t *q) {
	double roll;
	roll = atan2(2 * (q->y * q->z + q->w * q->x),
				 (q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z));
	return roll;
}

// // inverse of quaternion
quaternion_t quaternion_inverse(const quaternion_t *q) {
	quaternion_t result = {0};
	result.w = q->w;
	result.x = -q->x;
	result.y = -q->y;
	result.z = -q->z;
	return result;
}

