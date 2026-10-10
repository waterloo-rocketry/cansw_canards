/**
 * benchmark functiuons used the system
 */
#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <stdint.h>

// struct benchmark data
typedef struct {
	uint64_t sum;
	uint32_t count;
	float32_t average;
} benchmark_data_t;

#define BENCHMARK_FILE_INIT(number_of_benchmark_points)                                            \
	static uint8_t BENCHMARK_COUNT = number_of_benchmark_points;                                   \
	static uint32_t BENCHMARK_COUNTERS[number_of_benchmark_points] = {0};                          \
	static uint32_t BENCHMARK_TIME_START[number_of_benchmark_points] = {0};                        \
	static uint64_t BENCHMARK_TIME_SUMS[number_of_benchmark_points] = {0}

#define SINGLE_BENCHMARK_START(i) BENCHMARK_TIME_START[i] = DWT->CYCCNT

#define SINGLE_BENCHMARK_END(i)                                                                    \
	BENCHMARK_TIME_SUMS[i] = (DWT->CYCCNT - BENCHMARK_TIME_START[i]);                              \
	BENCHMARK_COUNTERS[i]++

#define GET_BENCHMARK_STAT(i, p_data)                                                              \
	p_data->sum = BENCHMARK_TIME_SUMS[i];                                                          \
	p_data->count = BENCHMARK_COUNTERS[i];                                                         \
	p_data->average = ((float32_t)BENCHMARK_TIME_SUMS[i]) / ((float32_t)BENCHMARK_COUNTERS[i])

#endif
