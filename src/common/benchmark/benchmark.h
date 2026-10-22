/**
 * benchmark functiuons used the system
 */
#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <stdint.h>

#include "rocketlib.h"
#include "stm32h7xx_hal.h"

// struct benchmark data
typedef struct {
	uint64_t sum;
	uint32_t count;
	float32_t average;
} benchmark_data_t;

/**
 * @brief needed to be called to turn on DWT counter for benchmarking purposes (Only to be called
 * ONCE)
 */
static inline void DWT_init(void) {
	// make sure the function is only called once
	static bool dwt_initialized = false;

	if (dwt_initialized) {
		return;
	}

	dwt_initialized = true;

	// Enable the trace/debug blocks.
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	// Unlock DWT registers on Cortex-M7.
	DWT->LAR = 0xc5acce55UL;

	// Reset and enable the CPU cycle counter.
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	__DSB();
	__ISB();
	return;
}

/**
 * @brief a macro function for initializing benchmarking capability for benchmarking
 * @param[in] number_of_benchmark_points (uint8_t) number of benchmark points that will be measured
 * within this file (ie, how many things to benchmark)
 * @details:
 * 	- This function must be called with file-wide scope
 * 	- make sure all benchmark points take less than 7 seconds to execute
 */
#define BENCHMARK_FILE_INIT(number_of_benchmark_points)                                            \
	static uint8_t BENCHMARK_COUNT = number_of_benchmark_points;                                   \
	static uint32_t BENCHMARK_COUNTERS[number_of_benchmark_points] = {0};                          \
	static uint32_t BENCHMARK_TIME_START[number_of_benchmark_points] = {0};                        \
	static uint64_t BENCHMARK_TIME_SUMS[number_of_benchmark_points] = {0}

/**
 * @brief the starting point of the i benchmark point
 * @param[in] i (uint8_t) the index of the benchmark point that will be measured
 */
#define SINGLE_BENCHMARK_START(i) BENCHMARK_TIME_START[i] = DWT->CYCCNT

/**
 * @brief the end point of the i benchmark point
 * @param[in] i (uint8_t) the index of the benchmark point that will be measured
 * @details an uninitialized start would not be counted or any that are exactly on DWT->CYCNT (the
 * probability is only 2^-32)
 */
#define SINGLE_BENCHMARK_END(i)                                                                    \
	if (BENCHMARK_TIME_START[i] != 0) {                                                            \
		BENCHMARK_TIME_SUMS[i] += (uint32_t)(DWT->CYCCNT - BENCHMARK_TIME_START[i]);               \
		BENCHMARK_COUNTERS[i]++;                                                                   \
	}

/**
 * @brief Get data on the i benchmark point
 * @param[in] i (uint8_t) the index of the benchmark point that will be measured
 * @param[out] p_data (benchmark_data_t *) the results of the i benchmark point
 */
#define GET_BENCHMARK_STAT(i, p_data)                                                              \
	p_data->sum = BENCHMARK_TIME_SUMS[i];                                                          \
	p_data->count = BENCHMARK_COUNTERS[i];                                                         \
	p_data->average = ((float32_t)BENCHMARK_TIME_SUMS[i]) / ((float32_t)BENCHMARK_COUNTERS[i])

#endif
