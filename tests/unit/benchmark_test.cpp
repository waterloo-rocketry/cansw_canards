#include "fff.h"
#include <cstddef>
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
