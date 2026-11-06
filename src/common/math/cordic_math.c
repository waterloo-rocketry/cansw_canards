#include "common/math/cordic_math.h"
#include "common/math/math.h"

#include "cordic.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

static const double Q31_SCALE = 2147483648.0;
static cordic_math_stats_t stats;

// Saturate before casting: +1 is not representable in signed Q1.31.
static int32_t to_q31(double value) {
	double scaled = value * Q31_SCALE;
	if (scaled >= INT32_MAX) {
		return INT32_MAX;
	}
	if (scaled <= INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)scaled;
}

static bool calculate(uint32_t function, uint32_t reads, const int32_t input[2],
                      int32_t output[2]) {
	const CORDIC_ConfigTypeDef config = {
		.Function = function,
		.Precision = CORDIC_PRECISION_6CYCLES,
		.Scale = CORDIC_SCALE_0,
		.NbWrite = CORDIC_NBWRITE_2,
		.NbRead = reads,
		.InSize = CORDIC_INSIZE_32BITS,
		.OutSize = CORDIC_OUTSIZE_32BITS,
	};

	// Configure every operation so alternating sincos/phase calls are independent
	// In this HAL, the final read for NbCalc=1 blocks in hardware; Timeout is not
	// a hard bound on the read? Keep the HAL timebase running when using this API.
	if ((HAL_CORDIC_Configure(&hcordic, &config) != HAL_OK) ||
	    (HAL_CORDIC_Calculate(&hcordic, input, output, 1U, 1U) != HAL_OK)) {
		stats.hal_failures++;
		stats.last_hal_error = HAL_CORDIC_GetError(&hcordic);
		return false;
	}
	return true;
}

bool cordic_math_sincos(double radians, double *sin_out, double *cos_out) {
	if (sin_out != NULL) {
		*sin_out = NAN;
	}
	if (cos_out != NULL) {
		*cos_out = NAN;
	}
	if ((sin_out == NULL) || (cos_out == NULL) || (sin_out == cos_out) || !isfinite(radians)) {
		stats.invalid_inputs++;
		return false;
	}

	if ((radians < -M_PI) || (radians >= M_PI)) {
		radians = fmod(radians, 2.0 * M_PI);
		if (radians >= M_PI) {
			radians -= 2.0 * M_PI;
		} else if (radians < -M_PI) {
			radians += 2.0 * M_PI;
		}
	}

	// Always write modulus explicitly: phase mode also uses the second argument.
	const int32_t input[2] = {to_q31(radians / M_PI), INT32_MAX};
	int32_t output[2];
	if (!calculate(CORDIC_FUNCTION_COSINE, CORDIC_NBREAD_2, input, output)) {
		return false;
	}
	*cos_out = output[0] / Q31_SCALE;
	*sin_out = output[1] / Q31_SCALE;
	return true;
}

double cordic_math_sin(double radians) {
	double sine, cosine;
	cordic_math_sincos(radians, &sine, &cosine);
	return sine;
}

double cordic_math_cos(double radians) {
	double sine, cosine;
	cordic_math_sincos(radians, &sine, &cosine);
	return cosine;
}

double cordic_math_atan2(double y, double x) {
	if (!isfinite(x) || !isfinite(y)) {
		stats.invalid_inputs++;
		return NAN;
	}
	if (y == 0.0) {
		return copysign(signbit(x) ? M_PI : 0.0, y);
	}
	if (x == 0.0) {
		return copysign(M_PI / 2.0, y);
	}

	// Common scaling preserves angle and bounds modulus to sqrt(0.5) < 1.
	// Divide before multiplying to avoid overflow/underflow in the scale factor.
	double magnitude = fmax(fabs(x), fabs(y));
	const int32_t input[2] = {to_q31(0.5 * (x / magnitude)), to_q31(0.5 * (y / magnitude))};
	int32_t output[2];
	if (!calculate(CORDIC_FUNCTION_PHASE, CORDIC_NBREAD_1, input, output)) {
		return NAN;
	}
	return (output[0] / Q31_SCALE) * M_PI;
}

static bool inverse_input_valid(double x) {
	if (!isfinite(x) || (x < -1.0) || (x > 1.0)) {
		stats.invalid_inputs++;
		return false;
	}
	return true;
}

double cordic_math_asin(double x) {
	if (!inverse_input_valid(x)) {
		return NAN;
	}
	if (fabs(x) == 1.0) {
		return copysign(M_PI / 2.0, x);
	}
	// Compute the product before narrowing so doubles near +/-1 remain nonzero.
	double adjacent = sqrtf((float)((1.0 - x) * (1.0 + x)));
	return cordic_math_atan2(x, adjacent);
}

double cordic_math_acos(double x) {
	if (!inverse_input_valid(x)) {
		return NAN;
	}
	if (x == 1.0) {
		return 0.0;
	}
	if (x == -1.0) {
		return M_PI;
	}
	double opposite = sqrtf((float)((1.0 - x) * (1.0 + x)));
	return cordic_math_atan2(opposite, x);
}

cordic_math_stats_t cordic_math_get_stats(void) {
	return stats;
}

void cordic_math_reset_stats(void) {
	stats = (cordic_math_stats_t){0};
}
