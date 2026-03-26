#include "fff.h"
#include "utils/math_testing_helpers.hpp"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
    #include "drivers/ad_breakout_board/ad_breakout_board.h"
    #include "rocketlib/include/common.h"
    #include "drivers/ad_breakout_board/ADXL380.h"
    #include "drivers/ad_breakout_board/ADXRS649.h"
    #include "application/logger/log.h"
    #include "drivers/timer/timer.h"

    extern w_status_t ad_beakout_board_init();

    FAKE_VALUE_FUNC(w_status_t, adxrs649_init);
    FAKE_VALUE_FUNC(w_status_t, adxl380_init);
    FAKE_VALUE_FUNC(w_status_t, adxrs649_get_gyro_data, float*, int32_t*);
    FAKE_VALUE_FUNC(w_status_t, adxl380_get_accel_data, vector3d_t *, altimu_raw_imu_data_t *);
    FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float*);
    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);

}

class ADBreakoutBoardTets : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(adxrs649_init);
        RESET_FAKE(adxl380_init);
        RESET_FAKE(adxrs649_get_gyro_data);
        RESET_FAKE(adxl380_get_accel_data);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(log_text);
    };
};

TEST_F(ADBreakoutBoardTets, InitSuccess) {
    adxrs649_init_fake.return_val = W_SUCCESS;
    adxl380_init_fake.return_val = W_SUCCESS;

    w_status_t status = ad_beakout_board_init();

    EXPECT_EQ(adxrs649_init_fake.call_count, 1);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(ADBreakoutBoardTets, InitFailureADXRS) {
    adxrs649_init_fake.return_val = W_FAILURE;
    adxl380_init_fake.return_val = W_SUCCESS;

    w_status_t status = ad_beakout_board_init();

    EXPECT_EQ(adxrs649_init_fake.call_count, 1);
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(ADBreakoutBoardTets, InitFailureADXL) {
    adxrs649_init_fake.return_val = W_SUCCESS;
    adxl380_init_fake.return_val = W_FAILURE;

    w_status_t status = ad_beakout_board_init();

    EXPECT_EQ(adxrs649_init_fake.call_count, 1);
    EXPECT_EQ(status, W_FAILURE);
}