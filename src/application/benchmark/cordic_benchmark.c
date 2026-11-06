#ifdef ENABLE_BENCHMARK

#include "application/benchmark/cordic_benchmark.h"
#include "common/math/math.h"
#include "stm32h7xx_hal.h"
#include <limits.h>
#include <math.h>

static const double Q31_SCALE = 2147483648.0;
volatile cordic_benchmark_results_t cordic_benchmark_results;
BENCHMARK_FILE_INIT(18);

typedef void (*operation_t)(double x, double y, double *a, double *b);

static struct {
	double x, y;
} inputs[CORDIC_BENCH_SAMPLES];

static volatile double outputs[CORDIC_BENCH_SAMPLES][2];

static int32_t to_q31(double x) {
	double scaled = x * Q31_SCALE;
	if (scaled >= INT32_MAX) {
		return INT32_MAX;
	}
	if (scaled <= INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)scaled;
}

static void double_sincos(double x, double y, double *a, double *b) {
	(void)y;
	*a = sin(x);
	*b = cos(x);
}

static void float_sincos(double x, double y, double *a, double *b) {
	(void)y;
	*a = sinf((float)x);
	*b = cosf((float)x);
}

static void q31_sincos(double x, double y, double *a, double *b) {
	(void)y;
	if (x < -M_PI || x >= M_PI) {
		x = fmod(x, 2.0 * M_PI);
		if (x >= M_PI) {
			x -= 2.0 * M_PI;
		}
		if (x < -M_PI) {
			x += 2.0 * M_PI;
		}
	}
	q31_t sine, cosine;
	arm_sin_cos_q31(to_q31(x / M_PI), &sine, &cosine);
	*a = sine / Q31_SCALE;
	*b = cosine / Q31_SCALE;
}

static void hal_sincos(double x, double y, double *a, double *b) {
	(void)y;
	cordic_math_sincos(x, a, b);
}

static void double_atan2(double x, double y, double *a, double *b) {
	*a = atan2(y, x);
	*b = 0.0;
}

static void float_atan2(double x, double y, double *a, double *b) {
	*a = atan2f((float)y, (float)x);
	*b = 0.0;
}

static void q31_atan2(double x, double y, double *a, double *b) {
	double scale = fmax(fabs(x), fabs(y));
	q31_t phase;
	arm_status status = arm_atan2_q31(to_q31(0.5 * (y / scale)), to_q31(0.5 * (x / scale)), &phase);
	// CMSIS atan2 returns radians in Q2.29, unlike CORDIC's angle/pi Q1.31.
	*a = status == ARM_MATH_SUCCESS ? phase / 536870912.0 : NAN;
	*b = 0.0;
}

static void hal_atan2(double x, double y, double *a, double *b) {
	*a = cordic_math_atan2(y, x);
	*b = 0.0;
}

static void double_asin(double x, double y, double *a, double *b) {
	(void)y;
	*a = asin(x);
	*b = 0.0;
}

static void float_asin(double x, double y, double *a, double *b) {
	(void)y;
	*a = asinf((float)x);
	*b = 0.0;
}

static void hal_asin(double x, double y, double *a, double *b) {
	(void)y;
	*a = cordic_math_asin(x);
	*b = 0.0;
}

static void double_acos(double x, double y, double *a, double *b) {
	(void)y;
	*a = acos(x);
	*b = 0.0;
}

static void float_acos(double x, double y, double *a, double *b) {
	(void)y;
	*a = acosf((float)x);
	*b = 0.0;
}

static void hal_acos(double x, double y, double *a, double *b) {
	(void)y;
	*a = cordic_math_acos(x);
	*b = 0.0;
}

static void measure(operation_t function, operation_t reference, bool phase,
                    volatile cordic_benchmark_result_t *result) {
	static uint8_t point;
	if (point >= BENCHMARK_COUNT) {
		return;
	}
	operation_t volatile operation = function;
	double a, b;
	for (unsigned i = 0; i < CORDIC_BENCH_SAMPLES; ++i) {
		operation(inputs[i].x, inputs[i].y, &a, &b);
		outputs[i][0] = a;
		outputs[i][1] = b;
	}
	for (unsigned batch = 0; batch < CORDIC_BENCH_BATCHES; ++batch) {
		for (unsigned i = 0; i < CORDIC_BENCH_SAMPLES; ++i) {
			SINGLE_BENCHMARK_START(point);
			__asm volatile("" ::: "memory");
			operation(inputs[i].x, inputs[i].y, &a, &b);
			__asm volatile("" ::: "memory");
			SINGLE_BENCHMARK_END(point);
			outputs[i][0] = a;
			outputs[i][1] = b;
		}
	}
	cordic_benchmark_result_t summary = {.supported = true};
	benchmark_data_t *cycles = &summary.cycles;
	GET_BENCHMARK_STAT(point, cycles);
	summary.average_us = summary.cycles.average * 1e6 / SystemCoreClock;
	for (unsigned i = 0; i < CORDIC_BENCH_SAMPLES; ++i) {
		reference(inputs[i].x, inputs[i].y, &a, &b);
		double actual_a = outputs[i][0], actual_b = outputs[i][1];
		if (!isfinite(actual_a) || !isfinite(actual_b)) {
			summary.nonfinite_results++;
			continue;
		}
		double error_a = actual_a - a;
		if (phase) {
			error_a = remainder(error_a, 2.0 * M_PI);
		}
		summary.max_abs_error =
			fmax(summary.max_abs_error, fmax(fabs(error_a), fabs(actual_b - b)));
	}
	*result = summary;
	point++;
}

void cordic_benchmark_run(void) {
	if (cordic_benchmark_results.complete) {
		return;
	}
	cordic_benchmark_results = (cordic_benchmark_results_t){0};
	cordic_benchmark_results.cpu_hz = SystemCoreClock;
	cordic_benchmark_results.samples_per_batch = CORDIC_BENCH_SAMPLES;
	cordic_benchmark_results.batches = CORDIC_BENCH_BATCHES;
	uint32_t start = DWT->CYCCNT;
	for (unsigned i = 0; i < 32; ++i) {
		__NOP();
	}
	cordic_benchmark_results.counter_running = DWT->CYCCNT != start;
	if (!cordic_benchmark_results.counter_running) {
		return;
	}
	cordic_math_reset_stats();

	for (unsigned wrapped = 0; wrapped < 2; ++wrapped) {
		for (unsigned i = 0; i < CORDIC_BENCH_SAMPLES; ++i) {
			double angle = ((i + 0.371) / CORDIC_BENCH_SAMPLES * 2.0 - 1.0) * M_PI;
			inputs[i].x = wrapped ? angle * 8.0 : angle;
			inputs[i].y = 0.0;
		}
		volatile cordic_benchmark_result_t *results =
			wrapped ? cordic_benchmark_results.sincos_wrapped : cordic_benchmark_results.sincos;
		measure(double_sincos, double_sincos, false, &results[CORDIC_BENCH_DOUBLE]);
		measure(float_sincos, double_sincos, false, &results[CORDIC_BENCH_FLOAT]);
		measure(q31_sincos, double_sincos, false, &results[CORDIC_BENCH_Q31]);
		measure(hal_sincos, double_sincos, false, &results[CORDIC_BENCH_HAL]);
	}
	for (unsigned i = 0; i < CORDIC_BENCH_SAMPLES; ++i) {
		double angle = ((i + 0.371) / CORDIC_BENCH_SAMPLES * 2.0 - 1.0) * M_PI;
		double scale = ldexp(1.0, (int)(i % 41) - 20);
		inputs[i].x = cos(angle) * scale;
		inputs[i].y = sin(angle) * scale;
	}
	measure(double_atan2, double_atan2, true, &cordic_benchmark_results.atan2[CORDIC_BENCH_DOUBLE]);
	measure(float_atan2, double_atan2, true, &cordic_benchmark_results.atan2[CORDIC_BENCH_FLOAT]);
	measure(q31_atan2, double_atan2, true, &cordic_benchmark_results.atan2[CORDIC_BENCH_Q31]);
	measure(hal_atan2, double_atan2, true, &cordic_benchmark_results.atan2[CORDIC_BENCH_HAL]);
	for (unsigned i = 0; i < CORDIC_BENCH_SAMPLES; ++i) {
		inputs[i].x = (i + 0.371) / CORDIC_BENCH_SAMPLES * 2.0 - 1.0;
		inputs[i].y = 0.0;
	}
	measure(double_asin, double_asin, false, &cordic_benchmark_results.asin[CORDIC_BENCH_DOUBLE]);
	measure(float_asin, double_asin, false, &cordic_benchmark_results.asin[CORDIC_BENCH_FLOAT]);
	measure(hal_asin, double_asin, false, &cordic_benchmark_results.asin[CORDIC_BENCH_HAL]);
	measure(double_acos, double_acos, false, &cordic_benchmark_results.acos[CORDIC_BENCH_DOUBLE]);
	measure(float_acos, double_acos, false, &cordic_benchmark_results.acos[CORDIC_BENCH_FLOAT]);
	measure(hal_acos, double_acos, false, &cordic_benchmark_results.acos[CORDIC_BENCH_HAL]);
	cordic_benchmark_results.cordic_errors = cordic_math_get_stats();
	cordic_benchmark_results.complete = true;
}

__attribute__((noinline)) void cordic_benchmark_done(void) {
	__DSB();
}

#endif // ENABLE_BENCHMARK
