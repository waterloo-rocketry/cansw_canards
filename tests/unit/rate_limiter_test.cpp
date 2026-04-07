#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "common/rate_limiter/rate_limiter.h"
#include <stdbool.h>
#include <stdint.h>
}

class RateLimiterTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(RateLimiterTest, RateLimiterSuccess1) {
    uint32_t last_time = 13012100;
    EXPECT_TRUE(rate_limiter(100, 13012120, &last_time));
    EXPECT_EQ(13012120, last_time);
}

TEST_F(RateLimiterTest, RateLimiterSuccessOnLimit) {
    uint32_t last_time = 13012100;
    EXPECT_TRUE(rate_limiter(4, 13012350, &last_time));
    EXPECT_EQ(13012350, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureWithSameTime) {
    uint32_t last_time = 13012100;
    EXPECT_FALSE(rate_limiter(4, 13012100, &last_time));
    EXPECT_EQ(13012100, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureRegular) {
    uint32_t last_time = 13012100;
    EXPECT_FALSE(rate_limiter(4, 13012300, &last_time));
    EXPECT_EQ(13012100, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureBelowBorder) {
    uint32_t last_time = 13012100;
    EXPECT_FALSE(rate_limiter(4, 13012349, &last_time));
    EXPECT_EQ(13012100, last_time);
}

TEST_F(RateLimiterTest, RateLimiterSuccessStartFrom0) {
    uint32_t last_time = 0;
    EXPECT_TRUE(rate_limiter(250, 5, &last_time));
    EXPECT_EQ(5, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureLastTimeLarger) {
    uint32_t last_time = 230;
    EXPECT_FALSE(rate_limiter(4, 100, &last_time));
    EXPECT_EQ(230, last_time);
}

// Same test as above just at maximum unsigned int
TEST_F(RateLimiterTest, RateLimiterFailurePassMax) {
    uint32_t last_time = UINT32_MAX;
    EXPECT_FALSE(rate_limiter(500, (uint32_t) (((uint64_t) UINT32_MAX) + 2), &last_time));
    EXPECT_EQ(UINT32_MAX, last_time);
}

TEST_F(RateLimiterTest, RateLimiterSuccessAtMax) {
    uint32_t last_time = UINT32_MAX - 2;
    EXPECT_TRUE(rate_limiter(500, UINT32_MAX, &last_time));
    EXPECT_EQ(UINT32_MAX, last_time);
}

TEST_F(RateLimiterTest, RateLimiterSuccessAtAfter32BitMultWouldOverflow) {
    uint32_t last_time = 0; 
    EXPECT_TRUE(rate_limiter(1000, (UINT32_MAX / 1000) + 1, &last_time));
    EXPECT_EQ((UINT32_MAX / 1000) + 1, last_time);
}