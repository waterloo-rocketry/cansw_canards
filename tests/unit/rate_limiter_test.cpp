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
    double last_time = 177457957100.2;
    EXPECT_EQ(true, rate_limiter(100, 177457957120.7, &last_time));
    EXPECT_FLOAT_EQ(177457957120.7, last_time);
}

TEST_F(RateLimiterTest, RateLimiterSuccessOnLimit) {
    double last_time = 177457957100;
    EXPECT_EQ(true, rate_limiter(4, 177457957350, &last_time));
    EXPECT_FLOAT_EQ(177457957350, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureWithSameTime) {
    double last_time = 177457957100.5;
    EXPECT_EQ(false, rate_limiter(4, 177457957100.5, &last_time));
    EXPECT_FLOAT_EQ(177457957100.5, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureRegular) {
    double last_time = 177457957100.2;
    EXPECT_EQ(false, rate_limiter(4, 177457957300, &last_time));
    EXPECT_FLOAT_EQ(177457957100.2, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureBelowBorder) {
    double last_time = 177457957100.2;
    EXPECT_EQ(false, rate_limiter(4, 177457957350.1, &last_time));
    EXPECT_FLOAT_EQ(177457957100.2, last_time);
}

TEST_F(RateLimiterTest, RateLimiterSuccessStartFrom0) {
    double last_time = 0;
    EXPECT_EQ(true, rate_limiter(250, 4.2, &last_time));
    EXPECT_FLOAT_EQ(4.2, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureLastTimeLarger) {
    double last_time = 230;
    EXPECT_EQ(false, rate_limiter(4, 100.2, &last_time));
    EXPECT_FLOAT_EQ(230, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureNeg1) {
    double last_time = 230;
    EXPECT_EQ(false, rate_limiter(4, -100.2, &last_time));
    EXPECT_FLOAT_EQ(230, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureNeg2) {
    double last_time = -230;
    EXPECT_EQ(false, rate_limiter(4, 100.2, &last_time));
    EXPECT_FLOAT_EQ(-230, last_time);
}

TEST_F(RateLimiterTest, RateLimiterFailureNeg3) {
    double last_time = -230;
    EXPECT_EQ(false, rate_limiter(1000, -228.5, &last_time));
    EXPECT_FLOAT_EQ(-230, last_time);
}