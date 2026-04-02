#ifndef JACOBIANS_H
#define JACOBIANS_H

// JACOBIANS HELPER FUNCTIONS

#include "application/estimator/estimator_types.h"

/**
 * @brief jacobians matrix definitions
 * @example rotation matrix 3x4: SIZE_VECTOR_3D * SIZE_QUAT
 * @example quaternion update matrix q_q 4x4: SIZE_QUAT * SIZE_QUAT
 * @example quaternion update matrix q_w 4x3: SIZE_QUAT * SIZE_VECTOR_3D
 * @example matrix3d_t 3x3: SIDE_MATRIX_3D * SIDE_MATRIX_3D
 * @example vector3d_t 3x1: SIZE_VECTOR_3D * SIZE_1D
 * @example 7x13 matrix: SIZE_IMU_MEAS * SIZE_STATE
 */

// vector flat dimension
#define SIZE_1D 1

// MATRIX DEFS
// rotation matrix
typedef union {
	double flat[SIZE_VECTOR_3D * SIZE_QUAT];

	struct {
		double s11, s12, s13, s14;
		double s21, s22, s23, s24;
		double s31, s32, s33, s34;
	};
} rotation_jacobian_t;

// quaternion update 4x4 matrix
typedef union {
	double flat[SIZE_QUAT * SIZE_QUAT];

	struct {
		double q11, q12, q13, q14;
		double q21, q22, q23, q24;
		double q31, q32, q33, q34;
		double q41, q42, q43, q44;
	};
} quaternion_update_matrix_q_t;

// quaternion update 4x3 matrix
typedef union {
	double flat[SIZE_QUAT * SIZE_VECTOR_3D];

	struct {
		double q11, q12, q13;
		double q21, q22, q23;
		double q31, q32, q33;
		double q41, q42, q43;
	};
} quaternion_update_matrix_w_t;

/**
 * @brief helper function to construct pData for jacobian matrix instance
 * @param jacobian_flat pointer of flattened jacobian matrix to write to
 * @param start_coor_x starting coordinate x in 13x13 jacobian (0-indexed)
 * @param start_coor_y starting coordinate y in 13x13 jacobian (0-indexed)
 * @param num_row number of rows of sub-structure
 * @param num_col number of columns of sub-structure
 * @param flat_data pointer to the data of sub-structure in row major order
 *
 * 2D coor (x, y) flattening to 1D coor (index): index = x * num_col + y
 *
 * CAUTIOUS only writes structures of pData with width of 13
 */
void write_pData(double *jacobian_flat, int start_coor_x, int start_coor_y, int num_row,
				 int num_col, const double *flat_data);

#endif
