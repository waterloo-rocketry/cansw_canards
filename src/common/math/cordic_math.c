#include "common/math/cordic_math.h"
#include "common/math/math.h"

#include "cordic.h"
#include <math.h>
#include <stddef.h>

static const double Q31_SCALE = 2147483648.0;

// clamp before casting; q1.31 cannot represent +1
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

static bool calculate(uint32_t function, uint32_t reads, const int32_t input[2],  int32_t output[2]) {
	const CORDIC_ConfigTypeDef config = {
		.Function = function,
		.Precision = CORDIC_PRECISION_6CYCLES,
		.Scale = CORDIC_SCALE_0,
		.NbWrite = CORDIC_NBWRITE_2,
		.NbRead = reads,
		.InSize = CORDIC_INSIZE_32BITS,
		.OutSize = CORDIC_OUTSIZE_32BITS,
	};

	// configure each call; sincos and atan2 use different modes
	return HAL_CORDIC_Configure(&hcordic, &config) == HAL_OK &&
	       HAL_CORDIC_Calculate(&hcordic, input, output, 1U, 1U) == HAL_OK;
}

bool cordic_math_sincos(double radians, double *sin_out, double *cos_out) {
	if (sin_out != NULL) {
		*sin_out = NAN;
	}
	if (cos_out != NULL) {
		*cos_out = NAN;
	}
	if ((sin_out == NULL) || (cos_out == NULL) || (sin_out == cos_out) || !isfinite(radians)) {
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

	// set the modulus to ~1; atan2 uses the second argument for y
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
		return NAN;
	}
	if (y == 0.0) {
		return copysign(signbit(x) ? M_PI : 0.0, y);
	}
	if (x == 0.0) {
		return copysign(M_PI / 2.0, y);
	}

	// scale both coordinates equally into [-0.5, 0.5] 
	double magnitude = fmax(fabs(x), fabs(y));
	const int32_t input[2] = {to_q31(0.5 * (x / magnitude)), to_q31(0.5 * (y / magnitude))};
	int32_t output[2];

	if (!calculate(CORDIC_FUNCTION_PHASE, CORDIC_NBREAD_1, input, output)) {
		return NAN;
	}
	return (output[0] / Q31_SCALE) * M_PI;
}

static bool inverse_sincos_input_valid(double x) {
	return fabs(x) <= 1.0;
}

double cordic_math_asin(double x) {
	if (!inverse_sincos_input_valid(x)) {
		return NAN;
	}
	// compute in double first; casting x to float could round it to +/-1
	double adjacent = sqrtf((float)((1.0 - x) * (1.0 + x)));
	return cordic_math_atan2(x, adjacent);
}

double cordic_math_acos(double x) {
	if (!inverse_sincos_input_valid(x)) {
		return NAN;
	}
	double opposite = sqrtf((float)((1.0 - x) * (1.0 + x)));
	return cordic_math_atan2(opposite, x);
}
