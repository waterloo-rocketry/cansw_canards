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

static void set_successful_init() {
     HAL_TIM_IC_Start_fake.return_val = HAL_OK;
    timer_init();
}

// test fixture for timer tests
class TimerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(HAL_TIM_IC_GetState);
        RESET_FAKE(__HAL_TIM_GET_COUNTER);
        RESET_FAKE(HAL_TIM_IC_Start);
        FFF_RESET_HISTORY();

        // initialize the timer handle
        memset(&timer_reg, 0, sizeof(TIM_TypeDef));
        htim2.Instance = &timer_reg;
        htim2.State = HAL_TIM_STATE_BUSY;

        // set default return values for the mocks
        HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
        __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // Will give 100ms by default
        HAL_TIM_IC_Start_fake.return_val = HAL_ERROR;
    }

    TIM_TypeDef timer_reg;
};

// TEST NO INIT FIRST to make the static global boolean is not being overridden

TEST_F(TimerTest, GetMsNoInitFails) {
    // Arrange
    uint32_t ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // 1000 ticks

    // Act
    w_status_t status = timer_get_ms(&ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

TEST_F(TimerTest, GetTenthMsNoInitFails) {
    // Arrange
    uint32_t tenth_ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // 1000 ticks

    // Act
    w_status_t status = timer_get_tenth_ms(&tenth_ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}


/* INIT */
TEST_F(TimerTest, InitSuccess) {
    // success in startup
    HAL_TIM_IC_Start_fake.return_val = HAL_OK;
    w_status_t status = timer_init();

    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(TimerTest, InitFailureWithHALFailure) {
    // error in startup
    HAL_TIM_IC_Start_fake.return_val = HAL_ERROR;
    w_status_t status = timer_init();

    EXPECT_EQ(status, W_FAILURE);
}


/* GET MS */
// test timer_get_ms with NULL pointer
TEST_F(TimerTest, GetMsNullPointerFails) {
    set_successful_init();
    
    // Act
    w_status_t status = timer_get_ms(NULL);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_ms with invalid timer instance
TEST_F(TimerTest, GetMsInvalidTimerInstanceFails) {
    set_successful_init();
    
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
    set_successful_init();
    
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
    set_successful_init();
    
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
    set_successful_init();
    
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

TEST_F(TimerTest, GetMsFailedInitFails) {
    HAL_TIM_IC_Start_fake.return_val = HAL_ERROR;
    w_status_t init_status = timer_init();
    EXPECT_EQ(init_status, W_FAILURE);

    // Arrange
    uint32_t ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // 1000 ticks

    // Act
    w_status_t status = timer_get_ms(&ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}


/* TENTH_MS */
// test timer_get_tenth_ms with NULL pointer
TEST_F(TimerTest, GetTenthMsNullPointerFails) {
    set_successful_init();
    
    // Act
    w_status_t status = timer_get_tenth_ms(NULL);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_tenth_ms with invalid timer instance
TEST_F(TimerTest, GetTenthMsInvalidTimerInstanceFails) {
    set_successful_init();
    
    // Arrange
    uint32_t tenth_ms;
    htim2.Instance = NULL;

    // Act
    w_status_t status = timer_get_tenth_ms(&tenth_ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_tenth_ms with timer not running
TEST_F(TimerTest, GetTenthMsTimerNotRunningFails) {
    set_successful_init();
    
    // Arrange
    uint32_t tenth_ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_ERROR;

    // Act
    w_status_t status = timer_get_tenth_ms(&tenth_ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 1);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}

// test timer_get_tenth_ms successful operation
TEST_F(TimerTest, GetTenthMsSuccessful) {
    set_successful_init();
    
    // Arrange
    uint32_t tenth_ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // 1000 ticks

    // Act
    w_status_t status = timer_get_tenth_ms(&tenth_ms);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 1);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 1);
    EXPECT_EQ(tenth_ms, 1000); // 1000 ticks = 1000 tenth of a ms
}

// test timer_get_tenth_ms with maximum counter value
TEST_F(TimerTest, GetTenthMsMaxCounterValue) {
    set_successful_init();
    
    // Arrange
    uint32_t tenth_ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = UINT32_MAX;

    // Act
    w_status_t status = timer_get_tenth_ms(&tenth_ms);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 1);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 1);
    EXPECT_EQ(tenth_ms, UINT32_MAX);
}

TEST_F(TimerTest, GetTenthMsFailedInitFails) {
    HAL_TIM_IC_Start_fake.return_val = HAL_ERROR;
    w_status_t init_status = timer_init();
    EXPECT_EQ(init_status, W_FAILURE);
    
    // Arrange
    uint32_t tenth_ms;
    HAL_TIM_IC_GetState_fake.return_val = HAL_TIM_STATE_BUSY;
    __HAL_TIM_GET_COUNTER_fake.return_val = 1000; // 1000 ticks

    // Act
    w_status_t status = timer_get_tenth_ms(&tenth_ms);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(HAL_TIM_IC_GetState_fake.call_count, 0);
    EXPECT_EQ(__HAL_TIM_GET_COUNTER_fake.call_count, 0);
}