#ifdef ENABLE_BENCHMARK

#include <math.h>

#include "stm32h7xx_ll_cordic.h"

#include "common/math/cordic_math.h"
#include "common/math/math.h"

#include "application/benchmark/cordic_benchmark.h"

enum {
	BENCH_DOUBLE,
	BENCH_FLOAT,
	BENCH_Q31,
	BENCH_CORDIC,
	BENCH_CORDIC_LL,
	BENCH_CORDIC_REG,
	BENCH_CORDIC_RAW,
	BENCH_COUNT
};

static const unsigned SAMPLE_COUNT = 1000000;
static const float64_t ANGLE_MIN = -M_PI;
static const float64_t ANGLE_MAX = M_PI;
static const float64_t Q31_SCALE = 2147483648.0;

BENCHMARK_FILE_INIT(BENCH_COUNT);
volatile cordic_benchmark_results_t cordic_benchmark_results;

// benchmark inputs are already within [-pi, pi)
static int32_t angle_to_q31(float64_t angle) {
	float64_t scaled = angle * (Q31_SCALE / M_PI);
	if (scaled >= INT32_MAX) {
		return INT32_MAX;
	}
	if (scaled <= INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)scaled;
}

static void q31_sincos(float64_t angle, float64_t *sine, float64_t *cosine) {
	q31_t phase = angle_to_q31(angle);
	q31_t sin_q31, cos_q31;
	arm_sin_cos_q31(phase, &sin_q31, &cos_q31);
	*sine = sin_q31 / Q31_SCALE;
	*cosine = cos_q31 / Q31_SCALE;
}

static volatile float64_t input_angle;
static volatile float64_t sine_result;
static volatile float64_t cosine_result;

void cordic_benchmark_sincos_run(void) {
	float64_t sine, cosine;

	// warmup
	for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
		input_angle = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * i / SAMPLE_COUNT;
		sine_result = sin(input_angle);
		cosine_result = cos(input_angle);
		sine_result = sinf((float32_t)input_angle);
		cosine_result = cosf((float32_t)input_angle);
		q31_sincos(input_angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		cordic_math_sincos(input_angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		cordic_math_sincos_ll(input_angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
	}

	// measurements
	for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
		input_angle = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * i / SAMPLE_COUNT;

		SINGLE_BENCHMARK_START(BENCH_DOUBLE);
		float64_t angle = input_angle;
		sine_result = sin(angle);
		cosine_result = cos(angle);
		SINGLE_BENCHMARK_END(BENCH_DOUBLE);

		SINGLE_BENCHMARK_START(BENCH_FLOAT);
		angle = input_angle;
		sine_result = sinf((float32_t)angle);
		cosine_result = cosf((float32_t)angle);
		SINGLE_BENCHMARK_END(BENCH_FLOAT);

		SINGLE_BENCHMARK_START(BENCH_Q31);
		angle = input_angle;
		q31_sincos(angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		SINGLE_BENCHMARK_END(BENCH_Q31);

		SINGLE_BENCHMARK_START(BENCH_CORDIC);
		angle = input_angle;
		cordic_math_sincos(angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		SINGLE_BENCHMARK_END(BENCH_CORDIC);

		SINGLE_BENCHMARK_START(BENCH_CORDIC_LL);
		angle = input_angle;
		cordic_math_sincos_ll(angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		SINGLE_BENCHMARK_END(BENCH_CORDIC_LL);

		SINGLE_BENCHMARK_START(BENCH_CORDIC_REG);
		angle = input_angle;
		cordic_math_sincos_ll(angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		SINGLE_BENCHMARK_END(BENCH_CORDIC_REG);

		// prepare the raw test outside its timed section
		const int32_t input = angle_to_q31(input_angle);
		LL_CORDIC_Config(CORDIC,
		                 LL_CORDIC_FUNCTION_COSINE,
		                 LL_CORDIC_PRECISION_6CYCLES,
		                 LL_CORDIC_SCALE_0,
		                 LL_CORDIC_NBWRITE_2,
		                 LL_CORDIC_NBREAD_2,
		                 LL_CORDIC_INSIZE_32BITS,
		                 LL_CORDIC_OUTSIZE_32BITS);

		// finish configuration writes before starting the timer
		__DSB();
		__ISB();

		SINGLE_BENCHMARK_START(BENCH_CORDIC_RAW);
		LL_CORDIC_WriteData(CORDIC, (uint32_t)input);
		LL_CORDIC_WriteData(CORDIC, (uint32_t)INT32_MAX);
		const int32_t cos_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
		const int32_t sin_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
		SINGLE_BENCHMARK_END(BENCH_CORDIC_RAW);

		// consume raw results outside the timed section
		cosine_result = cos_q31 / Q31_SCALE;
		sine_result = sin_q31 / Q31_SCALE;
	}

	// getting results
	cordic_benchmark_results_t results = {.cpu_hz = SystemCoreClock};
	GET_BENCHMARK_STAT(BENCH_DOUBLE, (&results.double_math));
	GET_BENCHMARK_STAT(BENCH_FLOAT, (&results.float_math));
	GET_BENCHMARK_STAT(BENCH_Q31, (&results.cmsis_q31));
	GET_BENCHMARK_STAT(BENCH_CORDIC, (&results.cordic));
	GET_BENCHMARK_STAT(BENCH_CORDIC_LL, (&results.cordic_ll));
	GET_BENCHMARK_STAT(BENCH_CORDIC_REG, (&results.cordic_reg));
	GET_BENCHMARK_STAT(BENCH_CORDIC_RAW, (&results.cordic_raw));
	cordic_benchmark_results = results;
}


static volatile float64_t input_x;
static volatile float64_t input_y;
static volatile float64_t phase_result;

// benchmark coordinates are in [-0.999, 0.999]; CMSIS returns radians in Q2.29.
static float64_t q31_atan2(float64_t y, float64_t x) {
    const q31_t x_q31 = (q31_t)(x * Q31_SCALE);
    const q31_t y_q31 = (q31_t)(y * Q31_SCALE);
    q31_t angle_q29;

    if (arm_atan2_q31(y_q31, x_q31, &angle_q29) != ARM_MATH_SUCCESS) {
        return NAN;
    }

    return (float64_t)angle_q29 / 536870912.0;
}

void cordic_benchmark_atan2_run(void) {

    // warmup
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        input_x = -1.0 + (2.0 * (i % 1000U) + 1.0) / 1000.0;
        input_y = -1.0 + (2.0 * (i / 1000U) + 1.0) / 1000.0;

        phase_result = atan2(input_y, input_x);
        phase_result = atan2f((float32_t)input_y, (float32_t)input_x);
        phase_result = q31_atan2(input_y, input_x);
        phase_result = cordic_math_atan2(input_y, input_x);
        phase_result = cordic_math_atan2_ll(input_y, input_x);
    }

    // measurements
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        input_x = -1.0 + (2.0 * (i % 1000U) + 1.0) / 1000.0;
        input_y = -1.0 + (2.0 * (i / 1000U) + 1.0) / 1000.0;

        SINGLE_BENCHMARK_START(BENCH_DOUBLE);
        float64_t x = input_x;
        float64_t y = input_y;
        phase_result = atan2(y, x);
        SINGLE_BENCHMARK_END(BENCH_DOUBLE);

        SINGLE_BENCHMARK_START(BENCH_FLOAT);
        x = input_x;
        y = input_y;
        phase_result = atan2f((float32_t)y, (float32_t)x);
        SINGLE_BENCHMARK_END(BENCH_FLOAT);

        SINGLE_BENCHMARK_START(BENCH_Q31);
        x = input_x;
        y = input_y;
        phase_result = q31_atan2(y, x);
        SINGLE_BENCHMARK_END(BENCH_Q31);

        SINGLE_BENCHMARK_START(BENCH_CORDIC);
        x = input_x;
        y = input_y;
        phase_result = cordic_math_atan2(y, x);
        SINGLE_BENCHMARK_END(BENCH_CORDIC);

        SINGLE_BENCHMARK_START(BENCH_CORDIC_LL);
        x = input_x;
        y = input_y;
        phase_result = cordic_math_atan2_ll(y, x);
        SINGLE_BENCHMARK_END(BENCH_CORDIC_LL);

		// prepare raw test
        x = input_x;
        y = input_y;
        const float64_t magnitude = fmax(fabs(x), fabs(y));
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

        // finish configuration writes before starting the timer
        __DSB();
        __ISB();

        SINGLE_BENCHMARK_START(BENCH_CORDIC_RAW);
        LL_CORDIC_WriteData(CORDIC, (uint32_t)x_q31);
        LL_CORDIC_WriteData(CORDIC, (uint32_t)y_q31);
        const int32_t phase_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
        SINGLE_BENCHMARK_END(BENCH_CORDIC_RAW);

        phase_result = (float64_t)phase_q31 * (M_PI / Q31_SCALE);
    }

    cordic_benchmark_results_t results = {.cpu_hz = SystemCoreClock};
    GET_BENCHMARK_STAT(BENCH_DOUBLE, (&results.double_math));
    GET_BENCHMARK_STAT(BENCH_FLOAT, (&results.float_math));
    GET_BENCHMARK_STAT(BENCH_Q31, (&results.cmsis_q31));
    GET_BENCHMARK_STAT(BENCH_CORDIC, (&results.cordic));
    GET_BENCHMARK_STAT(BENCH_CORDIC_LL, (&results.cordic_ll));
    GET_BENCHMARK_STAT(BENCH_CORDIC_RAW, (&results.cordic_raw));
    cordic_benchmark_results = results;
}



#endif
