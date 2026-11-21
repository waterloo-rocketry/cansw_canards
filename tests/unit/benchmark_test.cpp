#include "fff.h"
#include <cstddef>
#include <cstring>
#include <gtest/gtest.h>

#define ENABLE_BENCHMARK

extern "C" {
#include <stddef.h>
#include "common/benchmark/benchmark.h"
int snprintf_(char *s, size_t n, const char *format, ...) {
	(void)s;
	(void)n;
	(void)format;
	return 0;
}
}

DEFINE_FFF_GLOBALS;

BENCHMARK_FILE_INIT(1);

class BenchmarkTest : public ::testing::Test {
protected:
	void SetUp() override {
		DWT->CYCCNT = 0;
		BENCHMARK_RESET(0);
	}
};

static void skip_warmup(void) {
	uint32_t n;
	for (n = 0; n < BENCH_WARMUP_N; n++) {
		DWT->CYCCNT = 0;
		SINGLE_BENCHMARK_START(0);
		DWT->CYCCNT = 1;
		SINGLE_BENCHMARK_END(0);
	}
}

TEST_F(BenchmarkTest, HistBinEdges) {
	const uint32_t budget = 1000;

	EXPECT_EQ(bench_hist_bin(0, budget), 0u);
	EXPECT_EQ(bench_hist_bin(99, budget), 0u);
	EXPECT_EQ(bench_hist_bin(100, budget), 1u);
	EXPECT_EQ(bench_hist_bin(249, budget), 1u);
	EXPECT_EQ(bench_hist_bin(250, budget), 2u);
	EXPECT_EQ(bench_hist_bin(500, budget), 3u);
	EXPECT_EQ(bench_hist_bin(750, budget), 4u);
	EXPECT_EQ(bench_hist_bin(999, budget), 4u);
	EXPECT_EQ(bench_hist_bin(1000, budget), 5u);
	EXPECT_EQ(bench_hist_bin(1, 0), 5u);
}

TEST_F(BenchmarkTest, GetBeforeSampleIsZero) {
	benchmark_data_t s;
	GET_BENCHMARK_STAT(0, &s);
	EXPECT_EQ(s.count, 0u);
	EXPECT_EQ(s.sum, 0u);
	EXPECT_EQ(s.max, 0u);
}

TEST_F(BenchmarkTest, WrapAndMax) {
	benchmark_data_t s;

	skip_warmup();

	DWT->CYCCNT = 0xFFFFFFF0u;
	SINGLE_BENCHMARK_START(0);
	DWT->CYCCNT = 0x10u; // unsigned wrap: 0x20 cycles 
	SINGLE_BENCHMARK_END(0);

	GET_BENCHMARK_STAT(0, &s);
	EXPECT_EQ(s.count, 1u);
	EXPECT_EQ(s.last, 0x20u);
	EXPECT_EQ(s.min, 0x20u);
	EXPECT_EQ(s.max, 0x20u);
	EXPECT_EQ(s.sum, 0x20u);

	DWT->CYCCNT = 0;
	SINGLE_BENCHMARK_START(0);
	DWT->CYCCNT = 100;
	SINGLE_BENCHMARK_END(0);

	GET_BENCHMARK_STAT(0, &s);
	EXPECT_EQ(s.count, 2u);
	EXPECT_EQ(s.last, 100u);
	EXPECT_EQ(s.min, 0x20u);
	EXPECT_EQ(s.max, 100u);
	EXPECT_EQ(s.sum, (uint64_t)0x20 + 100u);
}

static void record_cycles(uint32_t dt) {
    DWT->CYCCNT = 0;
    SINGLE_BENCHMARK_START(0);
    DWT->CYCCNT = dt;
    SINGLE_BENCHMARK_END(0);
}

TEST_F(BenchmarkTest, WarmupSamplesAreDiscarded) {
    benchmark_data_t s;

    // Act
    skip_warmup();
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.sum, 0u);
    EXPECT_EQ(s.last, 0u);
    EXPECT_EQ(s.over_budget_count, 0u);

    // Act: 33rd pair is the first kept sample
    record_cycles(50);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 1u);
    EXPECT_EQ(s.last, 50u);
    EXPECT_EQ(s.min, 50u);
    EXPECT_EQ(s.max, 50u);
    EXPECT_EQ(s.sum, 50u);
}

TEST_F(BenchmarkTest, UnderAndOverBudgetHist) {
    const uint32_t budget = BENCH_US_TO_CYCLES(BENCH_TARGET_US);
    benchmark_data_t s;
    uint32_t b;

    skip_warmup();

    // Act: well under 10% of budget -> bin 0
    record_cycles(100);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 1u);
    EXPECT_EQ(s.over_budget_count, 0u);
    EXPECT_EQ(s.hist[0], 1u);
    for (b = 1; b < BENCH_HIST_BIN_COUNT; b++) {
        EXPECT_EQ(s.hist[b], 0u);
    }

    // Act: exactly on budget -> bin 5 and over_budget
    record_cycles(budget);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 2u);
    EXPECT_EQ(s.over_budget_count, 1u);
    EXPECT_EQ(s.hist[0], 1u);
    EXPECT_EQ(s.hist[5], 1u);
}

TEST_F(BenchmarkTest, EndWithoutStartIsIgnored) {
    benchmark_data_t s;

    skip_warmup();

    // Act
    DWT->CYCCNT = 100;
    SINGLE_BENCHMARK_END(0);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.sum, 0u);
    EXPECT_EQ(s.last, 0u);
}

TEST_F(BenchmarkTest, GetNullAndOutOfRangeAreNoOps) {
    benchmark_data_t s;
    benchmark_data_t *null_data = NULL;
    volatile uint8_t bad_i = 1;
    uint32_t b;

    skip_warmup();
    record_cycles(40);
    GET_BENCHMARK_STAT(0, &s);
    EXPECT_EQ(s.count, 1u);

    // Act / Assert: NULL must not crash
    GET_BENCHMARK_STAT(0, null_data);

    // Arrange: poison, then GET invalid index
    memset(&s, 0xAA, sizeof(s));

    // Act
    GET_BENCHMARK_STAT(bad_i, &s);

    // Assert: struct unchanged
    EXPECT_EQ(s.count, 0xAAAAAAAAu);
    EXPECT_EQ(s.last, 0xAAAAAAAAu);

    // Act: START/END on invalid index must not touch slot 0
    DWT->CYCCNT = 0;
    SINGLE_BENCHMARK_START(bad_i);
    DWT->CYCCNT = 999;
    SINGLE_BENCHMARK_END(bad_i);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 1u);
    EXPECT_EQ(s.last, 40u);
    for (b = 0; b < BENCH_HIST_BIN_COUNT; b++) {
        if (b == 0) {
            EXPECT_EQ(s.hist[b], 1u);
        } else {
            EXPECT_EQ(s.hist[b], 0u);
        }
    }
}

TEST_F(BenchmarkTest, ResetClearsStatsAndWarmup) {
    benchmark_data_t s;
    uint32_t b;

    skip_warmup();
    record_cycles(80);
    GET_BENCHMARK_STAT(0, &s);
    EXPECT_EQ(s.count, 1u);

    // Act
    BENCHMARK_RESET(0);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.last, 0u);
    EXPECT_EQ(s.min, 0u);
    EXPECT_EQ(s.max, 0u);
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.sum, 0u);
    EXPECT_EQ(s.over_budget_count, 0u);
    for (b = 0; b < BENCH_HIST_BIN_COUNT; b++) {
        EXPECT_EQ(s.hist[b], 0u);
    }

    // Act: warmup must run again after reset
    record_cycles(80);
    GET_BENCHMARK_STAT(0, &s);

    // Assert
    EXPECT_EQ(s.count, 0u);
}

TEST_F(BenchmarkTest, MinUpdatesWhenLaterSampleIsSmaller) {
    benchmark_data_t s;

    skip_warmup();
    record_cycles(100);
    record_cycles(50);
    GET_BENCHMARK_STAT(0, &s);

    EXPECT_EQ(s.count, 2u);
    EXPECT_EQ(s.last, 50u);
    EXPECT_EQ(s.min, 50u);
    EXPECT_EQ(s.max, 100u);
    EXPECT_EQ(s.sum, 150u);
}

TEST_F(BenchmarkTest, DwtInitEnablesCounterOnce) {
    // Arrange
    CoreDebug->DEMCR = 0;
    DWT->CTRL = 0;
    DWT->LAR = 0;
    DWT->CYCCNT = 99;

    // Act
    DWT_init();

    // Assert
    EXPECT_EQ(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk, CoreDebug_DEMCR_TRCENA_Msk);
    EXPECT_EQ(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk, DWT_CTRL_CYCCNTENA_Msk);
    EXPECT_EQ(DWT->LAR, 0xc5acce55UL);
    EXPECT_EQ(DWT->CYCCNT, 0u);

    // Act: second call must not clear a live counter
    DWT->CYCCNT = 123;
    DWT_init();

    // Assert
    EXPECT_EQ(DWT->CYCCNT, 123u);
}
