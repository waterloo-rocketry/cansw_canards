#ifndef COMMON_CORDIC_MATH_H
#define COMMON_CORDIC_MATH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Approximate trigonometry using the STM32 HAL CORDIC peripheral.
 *
 * Call after MX_CORDIC_Init(), from one owner with exclusive use of hcordic.
 * Angles are radians; Q1.31 calculations do not retain double precision.
 * Range reduction is intended for GNC-scale angles. Failures return NaN.
 */
double cordic_math_sin(double radians);
double cordic_math_cos(double radians);

/** Both output pointers must be non-null and distinct. On failure, non-null
 * outputs are set to NaN and false is returned. Computes both in one operation. */
bool cordic_math_sincos(double radians, double *sin_out, double *cos_out);

/** Finite inputs only. Handles axes and signed zeros as atan2 does, including
 * atan2(+0,+0) = +0 and atan2(+0,-0) = +pi. */
double cordic_math_atan2(double y, double x);

/** Domain [-1,1]. Uses CPU sqrtf followed by CORDIC atan2. */
double cordic_math_asin(double x);
double cordic_math_acos(double x);

typedef struct {
	uint32_t invalid_inputs;
	uint32_t hal_failures;
	uint32_t last_hal_error; /**< HAL CORDIC error bits, zero after reset. */
} cordic_math_stats_t;

/** Access/reset from the same owner as the mathematical operations. */
cordic_math_stats_t cordic_math_get_stats(void);
void cordic_math_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif // COMMON_CORDIC_MATH_H
