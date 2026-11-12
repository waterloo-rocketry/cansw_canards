#include <math.h>

#include "stm32h7xx_ll_cordic.h"
#include "cordic.h"

#include "common/math/cordic_math.h"
#include "common/math/math.h"

// 2^31
static const float64_t Q31_SCALE = 2147483648.0;

bool cordic_math_sincos(float64_t radians, float64_t *sin_out, float64_t *cos_out)
{
    const float64_t scaled = radians * (Q31_SCALE / M_PI);
    const int32_t angle = scaled >= Q31_SCALE
                        ? INT32_MAX
                        : (int32_t)scaled;

    const CORDIC_ConfigTypeDef config = {
        .Function = CORDIC_FUNCTION_COSINE,
        .Precision = CORDIC_PRECISION_6CYCLES,
        .Scale = CORDIC_SCALE_0,
        .NbWrite = CORDIC_NBWRITE_2,
        .NbRead = CORDIC_NBREAD_2,
        .InSize = CORDIC_INSIZE_32BITS,
        .OutSize = CORDIC_OUTSIZE_32BITS,
    };

    const int32_t input[2] = {angle, INT32_MAX};
    int32_t output[2];

    if (HAL_CORDIC_Configure(&hcordic, &config) != HAL_OK ||
        HAL_CORDIC_Calculate(&hcordic, input, output, 1U, 1U) != HAL_OK) {
        *sin_out = NAN;
        *cos_out = NAN;
        return false;
    }

    *cos_out = (float64_t)output[0] / Q31_SCALE;
    *sin_out = (float64_t)output[1] / Q31_SCALE;
    return true;
}

// assumes valid inputs (distinct valid pointers for outputs; radians in [-pi, pi) )
void cordic_math_sincos_ll(float64_t radians, float64_t *sin_out, float64_t *cos_out)
{
    const float64_t scaled = radians * (2147483648.0 / M_PI);

	// might be able to drop clamping? if i can guarantee [-M_PI, M_PI)
    const int32_t angle = scaled >= 2147483648.0 ? INT32_MAX : (int32_t)scaled;

    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_COSINE,
                     LL_CORDIC_PRECISION_6CYCLES,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_2,
                     LL_CORDIC_NBREAD_2,
                     LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);

    LL_CORDIC_WriteData(CORDIC, (uint32_t)angle);
	LL_CORDIC_WriteData(CORDIC, (uint32_t)INT32_MAX);

    const int32_t cos = (int32_t)LL_CORDIC_ReadData(CORDIC);
    const int32_t sin = (int32_t)LL_CORDIC_ReadData(CORDIC);

    *cos_out = (float64_t)cos / Q31_SCALE;
    *sin_out   = (float64_t)sin / Q31_SCALE;
}

void cordic_math_sincos_reg(float64_t radians, float64_t *sin_out, float64_t *cos_out)
{
    const float64_t scaled = radians * (Q31_SCALE / M_PI);
    const int32_t angle = scaled >= Q31_SCALE ? INT32_MAX : (int32_t)scaled;

    // overwrites the csr config
    CORDIC->CSR =
        LL_CORDIC_FUNCTION_COSINE |
        LL_CORDIC_PRECISION_6CYCLES |
        LL_CORDIC_SCALE_0 |
        LL_CORDIC_NBWRITE_2 |
        LL_CORDIC_NBREAD_2 |
        LL_CORDIC_INSIZE_32BITS |
        LL_CORDIC_OUTSIZE_32BITS;

    CORDIC->WDATA = (uint32_t)angle;
    CORDIC->WDATA = (uint32_t)INT32_MAX;

    const int32_t cosine = (int32_t)CORDIC->RDATA;
    const int32_t sine   = (int32_t)CORDIC->RDATA;

    *sin_out = (float64_t)sine / Q31_SCALE;
    *cos_out = (float64_t)cosine / Q31_SCALE;
}

float64_t cordic_math_sin(float64_t radians) {
	float64_t sine, cosine;
	cordic_math_sincos_ll(radians, &sine, &cosine);
	return sine;
}

float64_t cordic_math_cos(float64_t radians) {
	float64_t sine, cosine;
	cordic_math_sincos(radians, &sine, &cosine);
	return cosine;
}

// Assumes finite x and y, and exclusive access to CORDIC.
float64_t cordic_math_atan2(float64_t y, float64_t x)
{
    if (y == 0.0) {
        return copysign(signbit(x) ? M_PI : 0.0, y);
    }
    if (x == 0.0) {
        return copysign(M_PI / 2.0, y);
    }

    const float64_t ax = fabs(x);
    const float64_t ay = fabs(y);
    const float64_t magnitude = ax > ay ? ax : ay;

    // Normalize into [-0.5, 0.5], then convert directly to Q31.
    const int32_t input[2] = {
        (int32_t)((x / magnitude) * (Q31_SCALE / 2.0)),
        (int32_t)((y / magnitude) * (Q31_SCALE / 2.0)),
    };
    int32_t phase;

    const CORDIC_ConfigTypeDef config = {
        .Function = CORDIC_FUNCTION_PHASE,
        .Precision = CORDIC_PRECISION_6CYCLES,
        .Scale = CORDIC_SCALE_0,
        .NbWrite = CORDIC_NBWRITE_2,
        .NbRead = CORDIC_NBREAD_1,
        .InSize = CORDIC_INSIZE_32BITS,
        .OutSize = CORDIC_OUTSIZE_32BITS,
    };

    if (HAL_CORDIC_Configure(&hcordic, &config) != HAL_OK ||
        HAL_CORDIC_Calculate(&hcordic, input, &phase, 1U, 1U) != HAL_OK) {
        return NAN;
    }

    return (float64_t)phase * (M_PI / Q31_SCALE);
}

float64_t cordic_math_atan2_ll(float64_t y, float64_t x)
{
    if (y == 0.0) {
        return copysign(signbit(x) ? M_PI : 0.0, y);
    }
    if (x == 0.0) {
        return copysign(M_PI / 2.0, y);
    }

    const float64_t ax = fabs(x);
    const float64_t ay = fabs(y);
    const float64_t magnitude = ax > ay ? ax : ay;

    const int32_t x_q31 = (int32_t)((x / magnitude) * (Q31_SCALE / 2.0));
    const int32_t y_q31 = (int32_t)((y / magnitude) * (Q31_SCALE / 2.0));

    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_PHASE,
                     LL_CORDIC_PRECISION_6CYCLES,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_2,
                     LL_CORDIC_NBREAD_1,
                     LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);

    LL_CORDIC_WriteData(CORDIC, (uint32_t)x_q31);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)y_q31);

    const int32_t phase = (int32_t)LL_CORDIC_ReadData(CORDIC);

    return (float64_t)phase * (M_PI / Q31_SCALE);
}




// will have to rewrite all of the asin/acos;
static bool inverse_sincos_input_valid(float64_t x) {
	return fabs(x) <= 1.0;
}

float64_t cordic_math_asin(float64_t x) {
	if (!inverse_sincos_input_valid(x)) {
		return NAN;
	}
	// compute in double first; casting x to float could round it to +/-1
	float64_t adjacent = sqrtf((float32_t)((1.0 - x) * (1.0 + x)));
	return cordic_math_atan2(x, adjacent);
}

float64_t cordic_math_acos(float64_t x) {
	if (!inverse_sincos_input_valid(x)) {
		return NAN;
	}
	float64_t opposite = sqrtf((float32_t)((1.0 - x) * (1.0 + x)));
	return cordic_math_atan2(opposite, x);
}


// do smth for sqrts