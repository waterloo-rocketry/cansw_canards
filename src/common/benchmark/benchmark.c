#include "benchmark.h"

#include <stdbool.h>

#include "printf.h"
#include "stm32h7xx_hal.h"

const uint32_t BENCH_HIST_EDGE_PCT[BENCH_HIST_BIN_COUNT - 1] = {
	BENCH_HIST_PCT_0,
	BENCH_HIST_PCT_1,
	BENCH_HIST_PCT_2,
	BENCH_HIST_PCT_3,
	BENCH_HIST_PCT_4,
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
		if (cycles < (uint32_t)(((uint64_t)budget * BENCH_HIST_EDGE_PCT[b]) / 100u)) {
			return b;
		}
	}
	return BENCH_HIST_BIN_COUNT - 1;
}

uint32_t benchmark_hist_ascii(const uint32_t hist[BENCH_HIST_BIN_COUNT], char *buf,
                             uint32_t buf_len) {
	uint32_t peak = 1;
	uint32_t b;
	uint32_t used = 0;
	if ((buf == NULL) || (buf_len == 0)) {
		return 0;
	}
	for (b = 0; b < BENCH_HIST_BIN_COUNT; b++) {
		if (hist[b] > peak) {
			peak = hist[b];
		}
	}
	buf[0] = '\0';
	for (b = 0; b < BENCH_HIST_BIN_COUNT; b++) {
		uint32_t lo_pct = (b == 0) ? 0 : BENCH_HIST_EDGE_PCT[b - 1];
		uint32_t bars = (hist[b] * BENCH_HIST_BAR_WIDTH) / peak;
		uint32_t k;
		int n;
		if (used >= (buf_len - 1)) {
			break;
		}
		if ((b + 1) < BENCH_HIST_BIN_COUNT) {
			n = snprintf_(&buf[used], buf_len - used, "%u %3u-%3u%% |", (unsigned)b,
			              (unsigned)lo_pct, (unsigned)BENCH_HIST_EDGE_PCT[b]);
		} else {
			n = snprintf_(&buf[used], buf_len - used, "%u %3u%%+    |", (unsigned)b, (unsigned)lo_pct);
		}
		if (n < 0) {
			break;
		}
		used += (uint32_t)n;
		if (used >= (buf_len - 1)) {
			used = buf_len - 1;
			break;
		}
		for (k = 0; (k < bars) && (used < (buf_len - 1)); k++) {
			buf[used++] = '#';
		}
		n = snprintf_(&buf[used], buf_len - used, " %lu\n", (unsigned long)hist[b]);
		if (n < 0) {
			break;
		}
		used += (uint32_t)n;
		if (used >= buf_len) {
			used = buf_len - 1;
			break;
		}
	}
	if (used < buf_len) {
		buf[used] = '\0';
	} else {
		buf[buf_len - 1] = '\0';
	}
	return used;
}
