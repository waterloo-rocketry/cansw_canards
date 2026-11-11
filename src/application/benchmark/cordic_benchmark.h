#ifndef CORDIC_BENCHMARK_H
#define CORDIC_BENCHMARK_H

#include "common/benchmark/benchmark.h"

typedef struct {
	benchmark_data_t double_math;
	benchmark_data_t float_math;
	benchmark_data_t cmsis_q31;
	benchmark_data_t cordic;
	benchmark_data_t cordic_ll;
	benchmark_data_t cordic_raw;
	uint32_t cpu_hz;
} cordic_benchmark_results_t;

extern volatile cordic_benchmark_results_t cordic_benchmark_results;

void cordic_benchmark_run(void);

#endif // CORDIC_BENCHMARK_H
