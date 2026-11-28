#ifndef COMMON_CORDIC_MATH_H
#define COMMON_CORDIC_MATH_H

#include <stdbool.h>

#include "rocketlib.h"

/**
 * calculate trigonometry using the stm32 hal cordic peripheral
 * call after peripheral initialization
 * angles are radians; q1.31 calculations do not retain double precision
 * failures return nan
 */
float64_t cordic_math_sin(float64_t radians);
float64_t cordic_math_cos(float64_t radians);

// both pointers must be non-null and distinct; failure sets outputs to nan
bool cordic_math_sincos(float64_t radians, float64_t *sin_out, float64_t *cos_out);
void cordic_math_sincos_ll(float64_t radians, float64_t *sin_out, float64_t *cos_out);
void cordic_math_sincos_reg(float64_t radians, float64_t *sin_out, float64_t *cos_out);

// finite inputs only; handles axes and signed zeros as atan2 does
float64_t cordic_math_atan2(float64_t y, float64_t x);
float64_t cordic_math_atan2_ll(float64_t y, float64_t x);

// domain [-1, 1]; cpu sqrtf followed by cordic atan2
float64_t cordic_math_asin(float64_t x);
float64_t cordic_math_acos(float64_t x);

#endif
