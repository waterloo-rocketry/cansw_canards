#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "hal_timer_mock.h"
#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"
#include "utils/mock_log.hpp"
}

// test fixture for timer tests
class TimerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(HAL_TIM_IC_GetState);
        RESET_FAKE(__HAL_TIM_GET_COUNTER);
        FFF_RESET_HISTORY();

        // initialize the timer handle
        memset(&timer_reg, 0, sizeof(TIM_TypeDef));
        htim2.Instance = &timer_reg;
        htim2.State = HAL_TIM_STATE_BUSY;

        // set default return values for the mocks
        HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
        __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // Will give 100ms by default
    }

    TIM_TypeDef timer_reg;
};

// test timer_get_ms with NULL pointer
TEST_F(TimerTest, GetMsNullPointerFails) {
    // Act
    w_status_t status = timer_get_ms(NULL);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_ms with invalid timer instance
TEST_F(TimerTest, GetMsInvalidTimerInstanceFails) {
    // Arrange
    uint32_t ms;
    htim2.Instance = NULL;

    // Act
    w_status_t status = timer_get_ms(&ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_ms with timer not running
TEST_F(TimerTest, GetMsTimerNotRunningFails) {
    // Arrange
    uint32_t ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_ERROR;

    // Act
    w_status_t status = timer_get_ms(&ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 1);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_ms successful operation
TEST_F(TimerTest, GetMsSuccessful) {
    // Arrange
    uint32_t ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // 1000 ticks

    // Act
    w_status_t status = timer_get_ms(&ms);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 1);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 1);
    EXPECT_EQ(ms, 100); // 1000 ticks * 0.1ms = 100ms
}

// test timer_get_ms with maximum counter value
TEST_F(TimerTest, GetMsMaxCounterValue) {
    // Arrange
    uint32_t ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = UINT32_MAX;

    // Act
    w_status_t status = timer_get_ms(&ms);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 1);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 1);
    EXPECT_EQ(ms, UINT32_MAX / 10);
}
