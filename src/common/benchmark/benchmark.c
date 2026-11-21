#include "common/benchmark/benchmark.h"

#include <stdbool.h>

#include "stm32h7xx_hal.h"

const uint32_t BENCH_HIST_EDGE_PCT[BENCH_HIST_BIN_COUNT - 1] = {
	BENCH_HIST_PCT_0, BENCH_HIST_PCT_1, BENCH_HIST_PCT_2, BENCH_HIST_PCT_3, BENCH_HIST_PCT_4,
};

void DWT_init(void) {
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
}

uint32_t bench_hist_bin(uint32_t cycles, uint32_t budget) {
	uint32_t b;
	if (budget == 0) {
		return BENCH_HIST_BIN_COUNT - 1;
	}
	for (b = 0; b < (BENCH_HIST_BIN_COUNT - 1); b++) {
		if (cycles < (uint32_t)(((uint64_t)budget * BENCH_HIST_EDGE_PCT[b]) / 100U)) {
			return b;
		}
	}
	return BENCH_HIST_BIN_COUNT - 1;
}
