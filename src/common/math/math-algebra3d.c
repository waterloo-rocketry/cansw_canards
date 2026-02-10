/**
 * Special operations in R3, as they are used often
 * Cross product only exists in R3
 * Rotation matrix transpose is inverse rotation
 */
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>

// vector * scalar // vector scaling
vector3d_t math_vector3d_scale(double scalar, const vector3d_t *vector) {
	vector3d_t result;
	result.x = scalar * vector->x;
	result.y = scalar * vector->y;
	result.z = scalar * vector->z;
	return result;
}

// ||vector|| // norm of a vector
double math_vector3d_norm(const vector3d_t *vector) {
	// use math.h hypot cuz it handles overflow better than raw sqrt
	return hypot(hypot(vector->x, vector->y), vector->z);
}

// vector + vector // vector addition
vector3d_t math_vector3d_add(const vector3d_t *vector1, const vector3d_t *vector2) {
	vector3d_t result;
	result.x = vector1->x + vector2->x;
	result.y = vector1->y + vector2->y;
	result.z = vector1->z + vector2->z;
	return result;
}

// vector - vector // vector subtraction
vector3d_t math_vector3d_subt(const vector3d_t *vector_left, const vector3d_t *vector_right) {
	vector3d_t result;
	result.x = vector_left->x - vector_right->x;
	result.y = vector_left->y - vector_right->y;
	result.z = vector_left->z - vector_right->z;
	return result;
}

// vector * vector // vector dot product
float math_vector3d_dot(const vector3d_t *vector1, const vector3d_t *vector2) {
	float result;
	result = (vector1->x * vector2->x) + (vector1->y * vector2->y) + (vector1->z * vector2->z);
	return result;
}

// vector x vector // vector cross product, only in R3
vector3d_t math_vector3d_cross(const vector3d_t *vector_left, const vector3d_t *vector_right) {
	vector3d_t result;
	result.x = vector_left->y * vector_right->z - vector_left->z * vector_right->y;
	result.y = vector_left->z * vector_right->x - vector_left->x * vector_right->z;
	result.z = vector_left->x * vector_right->y - vector_left->y * vector_right->x;
	return result;
}

// matrix * vector // matrix vector multiplication, for rotating vectors
vector3d_t math_vector3d_rotate(const matrix3d_t *matrix, const vector3d_t *vector) {
	vector3d_t result;
	result.x = matrix->s11 * vector->x + matrix->s12 * vector->y + matrix->s13 * vector->z;
	result.y = matrix->s21 * vector->x + matrix->s22 * vector->y + matrix->s23 * vector->z;
	result.z = matrix->s31 * vector->x + matrix->s32 * vector->y + matrix->s33 * vector->z;
	return result;
}

// matrix ^T // transpose, for inverting rotation matrices
matrix3d_t math_matrix3d_transp(const matrix3d_t *input) {
	matrix3d_t result;
	result.s11 = input->s11;
	result.s12 = input->s21;
	result.s13 = input->s31;
	result.s21 = input->s12;
	result.s22 = input->s22;
	result.s23 = input->s32;
	result.s31 = input->s13;
	result.s32 = input->s23;
	result.s33 = input->s33;
	return result;
}

void math_init_matrix_identity(arm_matrix_instance_f64 *I, uint16_t size) {
	I->numCols = size;
	I->numRows = size;

	double *data = I->pData;
	uint32_t n = size * size;

	// Clear all values
	for (uint32_t i = 0; i < n; i++) {
		data[i] = 0.0;
	}

	// Set diagonal to 1.0
	for (uint16_t i = 0; i < size; i++) {
		data[i * size + i] = 1.0;
	}
}

void math_init_matrix_diag(arm_matrix_instance_f64 *matrix, const uint16_t size,
						   const double *vector) {
	matrix->numCols = size;
	matrix->numRows = size;

	for (uint16_t i = 0; i < size; i++) {
		for (uint16_t j = 0; j < size; j++) {
			matrix->pData[i * size + j] = (i == j) ? vector[i] : 0.0f;
		}
	}
}

matrix3d_t math_matrix3d_add(const matrix3d_t *a, const matrix3d_t *b) {
	matrix3d_t result;
	result.s11 = a->s11 + b->s11;
	result.s12 = a->s12 + b->s12;
	result.s13 = a->s13 + b->s13;
	result.s21 = a->s21 + b->s21;
	result.s22 = a->s22 + b->s22;
	result.s23 = a->s23 + b->s23;
	result.s31 = a->s31 + b->s31;
	result.s32 = a->s32 + b->s32;
	result.s33 = a->s33 + b->s33;
	return result;
}

matrix3d_t math_matrix3d_mult(const matrix3d_t *a, const matrix3d_t *b) {
	matrix3d_t result;
	result.s11 = a->s11 * b->s11 + a->s12 * b->s21 + a->s13 * b->s31;
	result.s12 = a->s11 * b->s12 + a->s12 * b->s22 + a->s13 * b->s32;
	result.s13 = a->s11 * b->s13 + a->s12 * b->s23 + a->s13 * b->s33;
	result.s21 = a->s21 * b->s11 + a->s22 * b->s21 + a->s23 * b->s31;
	result.s22 = a->s21 * b->s12 + a->s22 * b->s22 + a->s23 * b->s32;
	result.s23 = a->s21 * b->s13 + a->s22 * b->s23 + a->s23 * b->s33;
	result.s31 = a->s31 * b->s11 + a->s32 * b->s21 + a->s33 * b->s31;
	result.s32 = a->s31 * b->s12 + a->s32 * b->s22 + a->s33 * b->s32;
	result.s33 = a->s31 * b->s13 + a->s32 * b->s23 + a->s33 * b->s33;
	return result;
}

