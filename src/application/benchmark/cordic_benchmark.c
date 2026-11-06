#ifdef ENABLE_BENCHMARK

#include "application/benchmark/cordic_benchmark.h"
#include "common/math/cordic_math.h"
#include "common/math/math.h"

#include <math.h>

enum {
	BENCH_DOUBLE,
	BENCH_FLOAT,
	BENCH_Q31,
	BENCH_CORDIC,
	BENCH_COUNT
};

static const unsigned SAMPLE_COUNT = 256;
static const double ANGLE_MIN = -M_PI;
static const double ANGLE_MAX = M_PI;
static const double Q31_SCALE = 2147483648.0;

BENCHMARK_FILE_INIT(BENCH_COUNT);
volatile cordic_benchmark_results_t cordic_benchmark_results;

static volatile double input_angle;
static volatile double sine_result;
static volatile double cosine_result;

static void q31_sincos(double angle, double *sine, double *cosine) {
	q31_t phase = (q31_t)(angle / M_PI * Q31_SCALE);
	q31_t sin_q31, cos_q31;
	arm_sin_cos_q31(phase, &sin_q31, &cos_q31);
	*sine = sin_q31 / Q31_SCALE;
	*cosine = cos_q31 / Q31_SCALE;
}

void cordic_benchmark_run(void) {
	double sine, cosine;

	// warmup all impls?
	for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
		input_angle = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * i / SAMPLE_COUNT;
		sine_result = sin(input_angle);
		cosine_result = cos(input_angle);
		sine_result = sinf((float)input_angle);
		cosine_result = cosf((float)input_angle);
		q31_sincos(input_angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
		cordic_math_sincos(input_angle, &sine, &cosine);
		sine_result = sine;
		cosine_result = cosine;
	}

	// measurements
	for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
		input_angle = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * i / SAMPLE_COUNT;

		SINGLE_BENCHMARK_START(BENCH_DOUBLE);
		double angle = input_angle;
		sine_result = sin(angle);
		cosine_result = cos(angle);
		SINGLE_BENCHMARK_END(BENCH_DOUBLE);

		SINGLE_BENCHMARK_START(BENCH_FLOAT);
		angle = input_angle;
		sine_result = sinf((float)angle);
		cosine_result = cosf((float)angle);
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
	}

	// getting results
	cordic_benchmark_results_t results = {.cpu_hz = SystemCoreClock};
	GET_BENCHMARK_STAT(BENCH_DOUBLE, (&results.double_math));
	GET_BENCHMARK_STAT(BENCH_FLOAT, (&results.float_math));
	GET_BENCHMARK_STAT(BENCH_Q31, (&results.cmsis_q31));
	GET_BENCHMARK_STAT(BENCH_CORDIC, (&results.cordic));
	cordic_benchmark_results = results;
}

__attribute__((noinline)) void cordic_benchmark_done(void) {
	__DSB();
}

#endif
