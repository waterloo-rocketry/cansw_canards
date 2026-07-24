/**
 * Math types and utils used in the processor project
 */
#ifndef COMMON_MATH_H
#define COMMON_MATH_H

#include "arm_math.h"
#include <math.h>

#include "rocketlib/include/common.h"

#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265
#endif

static inline bool float_equal(float64_t a, float64_t b) {
	return fabs(a - b) < 0.00001;
}

#define SIZE_VECTOR_3D 3
#define SIZE_QUAT 4
#define SIDE_MATRIX_3D 3

#define DEG_PER_RAD (180.0 / M_PI)
#define RAD_PER_DEG (M_PI / 180.0)

#define MS_PER_SEC 1000

// use for checking float equality
#define EQ_EPSILON 1e-8

/**
 * 3D float64_t vector.
 */
// TODO: rename to vector3d_f64_t
typedef union {
	float64_t array[SIZE_VECTOR_3D];

	struct {
		float64_t x;
		float64_t y;
		float64_t z;
	};
} vector3d_t;

typedef union {
	float array[SIZE_VECTOR_3D];

	struct {
		float x;
		float y;
		float z;
	};
} vector3d_f32_t;

/**
 * float64_t quaternion.
 */
// TODO: rename to quaternion_f64_t
typedef union {
	float64_t array[SIZE_QUAT];

	struct {
		float64_t w;
		float64_t x;
		float64_t y;
		float64_t z;
	};
} quaternion_t;

/**
 * float64_t quaternion.
 */
typedef union {
	float array[SIZE_QUAT];

	struct {
		float w;
		float x;
		float y;
		float z;
	};
} quaternion_f32_t;

/**
 * 3D (rotation) matrix.
 */
typedef union {
	float64_t array[SIDE_MATRIX_3D][SIDE_MATRIX_3D];

	// elements sij, with the row i and the column j
	struct {
		float64_t s11, s12, s13;
		float64_t s21, s22, s23;
		float64_t s31, s32, s33;
	};

	float64_t flat[SIDE_MATRIX_3D * SIDE_MATRIX_3D];
} matrix3d_t;

// helper function for estimator models
static inline float64_t cot(float64_t x) {
	if (float_equal(tan(x), 0)) {
		return 0;
	}
	return 1 / tan(x);
}

// diy helpers for f64 because cmsis-dsp doesnt have them

/**
 * @brief Floating-point matrix addition (float64_t-precision).
 * @param[in]  pSrcA points to the first input matrix structure
 * @param[in]  pSrcB points to the second input matrix structure
 * @param[out] pDst  points to output matrix structure
 * @return     execution status
 */
static inline void arm_mat_add_f64(const arm_matrix_instance_f64 *pSrcA,
								   const arm_matrix_instance_f64 *pSrcB,
								   arm_matrix_instance_f64 *pDst) {
	uint32_t numSamples;
	float64_t *pInA, *pInB, *pOut;

	// Check for matrix size mismatch
	if ((pSrcA->numRows != pSrcB->numRows) || (pSrcA->numCols != pSrcB->numCols) ||
		(pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols)) {
		return;
	}

	numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

	pInA = pSrcA->pData;
	pInB = pSrcB->pData;
	pOut = pDst->pData;

	while (numSamples > 0U) {
		*pOut++ = *pInA++ + *pInB++;
		numSamples--;
	}
}

/**
 * @brief Floating-point matrix and vector multiplication (float64_t-precision).
 * @param[in]  pSrcMat points to the input matrix structure
 * @param[in]  pVec    points to input vector
 * @param[out] pDst    points to output vector
 */
static inline void arm_mat_vec_mult_f64(const arm_matrix_instance_f64 *pSrcMat,
										const float64_t *pVec, float64_t *pDst) {
	uint16_t row, col;
	const float64_t *pMat = pSrcMat->pData;
	float64_t sum;
	uint16_t numRows = pSrcMat->numRows;
	uint16_t numCols = pSrcMat->numCols;

	for (row = 0; row < numRows; row++) {
		sum = 0.0;
		for (col = 0; col < numCols; col++) {
			sum += pMat[row * numCols + col] * pVec[col];
		}
		pDst[row] = sum;
	}
}

/**
 * @brief Floating-point matrix scaling (float64_t-precision).
 * @param[in]  pSrc   points to input matrix
 * @param[in]  scale  scale factor
 * @param[out] pDst   points to output matrix
 * @return     execution status
 */
static inline void arm_mat_scale_f64(const arm_matrix_instance_f64 *pSrc, float64_t scale,
									 arm_matrix_instance_f64 *pDst) {
	uint32_t numSamples;
	float64_t *pIn = pSrc->pData;
	float64_t *pOut = pDst->pData;

	// Check for matrix size mismatch
	if ((pSrc->numRows != pDst->numRows) || (pSrc->numCols != pDst->numCols)) {}

	numSamples = (uint32_t)pSrc->numRows * pSrc->numCols;

	while (numSamples > 0U) {
		*pOut++ = (*pIn++) * scale;
		numSamples--;
	}
}

#endif // COMMON_MATH_H
