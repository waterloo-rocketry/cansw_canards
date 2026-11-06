#ifndef COMMON_CORDIC_MATH_H
#define COMMON_CORDIC_MATH_H

#include <stdbool.h>

/**
 * calculate trigonometry using the stm32 hal cordic peripheral
 * call after peripheral initialization
 * angles are radians; q1.31 calculations do not retain double precision
 * failures return nan
 */
double cordic_math_sin(double radians);
double cordic_math_cos(double radians);

// both pointers must be non-null and distinct; failure sets outputs to nan
bool cordic_math_sincos(double radians, double *sin_out, double *cos_out);

// finite inputs only; handles axes and signed zeros as atan2 does
double cordic_math_atan2(double y, double x);

// domain [-1, 1]; cpu sqrtf followed by cordic atan2
double cordic_math_asin(double x);
double cordic_math_acos(double x);

#endif
