#ifndef CORDIC_BENCHMARK_H
#define CORDIC_BENCHMARK_H

#include "common/benchmark/benchmark.h"
#include "common/math/cordic_math.h"

enum {
	CORDIC_BENCH_SAMPLES = 256,
	CORDIC_BENCH_BATCHES = 16
};

enum {
	CORDIC_BENCH_DOUBLE,
	CORDIC_BENCH_FLOAT,
	CORDIC_BENCH_Q31,
	CORDIC_BENCH_HAL,
	CORDIC_BENCH_METHODS
};

typedef struct {
	benchmark_data_t cycles;
	double average_us;
	double max_abs_error;
	uint32_t nonfinite_results;
	bool supported;
} cordic_benchmark_result_t;

typedef struct {
	cordic_benchmark_result_t sincos[CORDIC_BENCH_METHODS];
	cordic_benchmark_result_t sincos_wrapped[CORDIC_BENCH_METHODS];
	cordic_benchmark_result_t atan2[CORDIC_BENCH_METHODS];
	cordic_benchmark_result_t asin[CORDIC_BENCH_METHODS];
	cordic_benchmark_result_t acos[CORDIC_BENCH_METHODS];
	cordic_math_stats_t cordic_errors;
	uint32_t cpu_hz;
	uint32_t samples_per_batch;
	uint32_t batches;
	bool counter_running;
	bool complete;
} cordic_benchmark_results_t;

extern volatile cordic_benchmark_results_t cordic_benchmark_results;

/** One-shot benchmark after MX_CORDIC_Init(), before starting application tasks. */
void cordic_benchmark_run(void);
/** Set a debugger breakpoint here; called after results have been stored. */
void cordic_benchmark_done(void);

#endif // CORDIC_BENCHMARK_H
