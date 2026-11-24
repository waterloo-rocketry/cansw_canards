/**
 * @file benchmark.h
 * @brief DWT cycle-count probes. Times are CPU cycles at BENCH_CPU_HZ.
 *
 * See README.md in this directory for what to collect and how to read it.
 */
#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <stdint.h>

#include "rocketlib.h"
#include "stm32h7xx_hal.h"

/* helpers */

#define BENCH_CPU_HZ (540U * 1000U * 1000U)
#define BENCH_CYCLES_PER_US (BENCH_CPU_HZ / (1000U * 1000U))

// convert between us and cycles
#define BENCH_US_TO_CYCLES(time_us) ((uint32_t)(time_us) * BENCH_CYCLES_PER_US)
#define BENCH_CYCLES_TO_US(num_cycles) ((uint32_t)(num_cycles) / BENCH_CYCLES_PER_US)

// TODO: add `BENCHMARK_FILE_INIT(n)` to the .c file before using the benchmark

#define BENCH_WARMUP_N 32U
#define BENCH_HIST_BIN_COUNT 6U

// histogram percentiles
#define BENCH_HIST_PCT_0 10U
#define BENCH_HIST_PCT_1 25U
#define BENCH_HIST_PCT_2 50U
#define BENCH_HIST_PCT_3 75U
#define BENCH_HIST_PCT_4 100U

STATIC_ASSERT(540U == BENCH_CYCLES_PER_US, "BENCH_CPU_HZ / 1e6");
STATIC_ASSERT(100U == BENCH_HIST_PCT_4, "bin 4 upper edge is the period");

extern const uint32_t BENCH_HIST_EDGE_PCT[BENCH_HIST_BIN_COUNT - 1];

/**
 * @brief benchmark data structure
 * @param last, min, max: most recent / smallest / largest sample after warmup
 * @param count: samples after warmup; mean = sum / count
 * @param sum: running sum of those samples
 * @param over_budget_count: samples with last >= BENCH_TARGET_US (hist bin 5)
 * @param hist[i]: samples in bucket i (percent of budget; see BENCH_HIST_EDGE_PCT).
 */
typedef struct {
	uint32_t last;
	uint32_t min;
	uint32_t max;
	uint32_t count; // after warmup; use with sum to calculate avg
	uint64_t sum;
	uint32_t over_budget_count;
	uint32_t hist[BENCH_HIST_BIN_COUNT];
} benchmark_data_t;

/**
 * @brief needed to be called to turn on DWT counter for benchmarking purposes
 * @note Only to be called ONCE
 */
void DWT_init(void);

/**
 * @brief sort the cycles into the histogram bins
 * @param[in] cycles (uint32_t) the number of cycles to calculate the bin index for
 * @param[in] budget (uint32_t) the budget in cycles
 * @return the histogram bin index
 */
uint32_t bench_hist_bin(uint32_t cycles, uint32_t budget);

#ifndef ENABLE_BENCHMARK

#define BENCHMARK_FILE_INIT(n)
#define SINGLE_BENCHMARK_START(i) ((void)0)
#define SINGLE_BENCHMARK_END(i) ((void)0)
#define BENCHMARK_RESET(i) ((void)0)
#define GET_BENCHMARK_STAT(i, p_data)                                                              \
	do {                                                                                           \
		if ((p_data) != NULL) {                                                                    \
			*(p_data) = (benchmark_data_t){0};                                                     \
		}                                                                                          \
	} while (0)

#else

/**
 * @brief a macro function for initializing benchmarking capability for benchmarking
 * @param[in] number_of_benchmark_points (uint8_t) number of benchmark points that will be measured
 * within this file (ie, how many things to benchmark)
 * @details:
 * 	- This function must be called with file-wide scope
 * 	- make sure all benchmark points take less than 7 seconds to execute
 */
#define BENCHMARK_FILE_INIT(n, target_period_us)                                                                     \
	STATIC_ASSERT(BENCH_TARGET_US > 0, "define BENCH_TARGET_US (period in microseconds)");         \
	static const uint8_t BENCHMARK_COUNT = (n);                                                    \
	static const uint32_t BENCHMARK_BUDGET = BENCH_US_TO_CYCLES(target_period_us);                  \
	static uint8_t BENCHMARK_STARTED[n];                                                           \
	static uint32_t BENCHMARK_WARMUP_SEEN[n];                                                      \
	static uint32_t BENCHMARK_TIME_START[n];                                                       \
	static uint32_t BENCHMARK_COUNTERS[n];                                                         \
	static uint64_t BENCHMARK_TIME_SUMS[n];                                                        \
	static uint32_t BENCHMARK_TIME_MAX[n];                                                         \
	static uint32_t BENCHMARK_TIME_MIN[n];                                                         \
	static uint32_t BENCHMARK_LAST[n];                                                             \
	static uint32_t BENCHMARK_OVER_BUDGET[n];                                                      \
	static uint32_t BENCHMARK_HIST[n][BENCH_HIST_BIN_COUNT]

/**
 * @brief the starting point of the i benchmark point
 * @param[in] i (uint8_t) the index of the benchmark point that will be measured
 */
#define SINGLE_BENCHMARK_START(i)                                                                  \
	do {                                                                                           \
		if ((i) < BENCHMARK_COUNT) {                                                               \
			BENCHMARK_STARTED[i] = 1;                                                              \
			BENCHMARK_TIME_START[i] = DWT->CYCCNT;                                                 \
			__asm volatile("" ::: "memory");                                                       \
		}                                                                                          \
	} while (0)

/**
 * @brief the end point of the i benchmark point
 * @param[in] i (uint8_t) the index of the benchmark point that will be measured
 * @details an uninitialized start would not be counted or any that are exactly on DWT->CYCNT (the
 * probability is only 2^-32)
 * The start time is automatically set back 0 after the call
 */
#define SINGLE_BENCHMARK_END(i)                                                                    \
	do {                                                                                           \
		if (((i) < BENCHMARK_COUNT) && (BENCHMARK_STARTED[i] != 0)) {                              \
			__asm volatile("" ::: "memory");                                                       \
			uint32_t _dt = DWT->CYCCNT - BENCHMARK_TIME_START[i];                                  \
			__asm volatile("" ::: "memory");                                                       \
			BENCHMARK_STARTED[i] = 0;                                                              \
			if (BENCHMARK_WARMUP_SEEN[i] < BENCH_WARMUP_N) {                                       \
				BENCHMARK_WARMUP_SEEN[i]++;                                                        \
			} else {                                                                               \
				BENCHMARK_LAST[i] = _dt;                                                           \
				BENCHMARK_TIME_SUMS[i] += _dt;                                                     \
				if ((BENCHMARK_COUNTERS[i] == 0) || (_dt < BENCHMARK_TIME_MIN[i])) {               \
					BENCHMARK_TIME_MIN[i] = _dt;                                                   \
				}                                                                                  \
				if (_dt > BENCHMARK_TIME_MAX[i]) {                                                 \
					BENCHMARK_TIME_MAX[i] = _dt;                                                   \
				}                                                                                  \
				BENCHMARK_COUNTERS[i]++;                                                           \
				if (_dt >= BENCHMARK_BUDGET) {                                                     \
					BENCHMARK_OVER_BUDGET[i]++;                                                    \
				}                                                                                  \
				BENCHMARK_HIST[i][bench_hist_bin(_dt, BENCHMARK_BUDGET)]++;                        \
			}                                                                                      \
		}                                                                                          \
	} while (0)

#define BENCHMARK_RESET(i)                                                                         \
	do {                                                                                           \
		uint32_t _b;                                                                               \
		if ((i) < BENCHMARK_COUNT) {                                                               \
			BENCHMARK_STARTED[i] = 0;                                                              \
			BENCHMARK_WARMUP_SEEN[i] = 0;                                                          \
			BENCHMARK_TIME_START[i] = 0;                                                           \
			BENCHMARK_COUNTERS[i] = 0;                                                             \
			BENCHMARK_TIME_SUMS[i] = 0;                                                            \
			BENCHMARK_TIME_MAX[i] = 0;                                                             \
			BENCHMARK_TIME_MIN[i] = 0;                                                             \
			BENCHMARK_LAST[i] = 0;                                                                 \
			BENCHMARK_OVER_BUDGET[i] = 0;                                                          \
			for (_b = 0; _b < BENCH_HIST_BIN_COUNT; _b++) {                                        \
				BENCHMARK_HIST[i][_b] = 0;                                                         \
			}                                                                                      \
		}                                                                                          \
	} while (0)

/**
 * @brief Get data on the i benchmark point
 * @param[in] i (uint8_t) the index of the benchmark point that will be measured
 * @param[out] p_data (benchmark_data_t *) the results of the i benchmark point
 */
#define GET_BENCHMARK_STAT(i, p_data)                                                              \
	do {                                                                                           \
		uint32_t _b;                                                                               \
		if (((p_data) != NULL) && ((i) < BENCHMARK_COUNT)) {                                       \
			(p_data)->last = BENCHMARK_LAST[i];                                                    \
			(p_data)->min = BENCHMARK_TIME_MIN[i];                                                 \
			(p_data)->max = BENCHMARK_TIME_MAX[i];                                                 \
			(p_data)->count = BENCHMARK_COUNTERS[i];                                               \
			(p_data)->sum = BENCHMARK_TIME_SUMS[i];                                                \
			(p_data)->over_budget_count = BENCHMARK_OVER_BUDGET[i];                                \
			for (_b = 0; _b < BENCH_HIST_BIN_COUNT; _b++) {                                        \
				(p_data)->hist[_b] = BENCHMARK_HIST[i][_b];                                        \
			}                                                                                      \
		}                                                                                          \
	} while (0)

#endif

#endif
