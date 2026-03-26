#include "fff.h"
#include <gtest/gtest.h>
#include "utils/math_testing_helpers.hpp"
#include<iostream>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include <stdint.h>
#include <stdbool.h>
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/adxl38x.h"
#include "drivers/i2c/i2c.h"
#include "drivers/gpio/gpio.h"

extern w_status_t adxl380_init();
extern w_status_t adxl380_get_raw_accel(altimu_raw_imu_data_t *raw_data);
extern w_status_t adxl380_get_accel_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data);

FAKE_VALUE_FUNC(w_status_t, adxl38x_init, adxl38x_dev_t *);
FAKE_VALUE_FUNC(w_status_t, adxl38x_soft_reset, adxl38x_dev_t *);
FAKE_VALUE_FUNC(w_status_t, adxl38x_set_op_mode, adxl38x_dev_t *, adxl38x_op_mode_t);
FAKE_VALUE_FUNC(w_status_t, adxl38x_set_range, adxl38x_dev_t *, adxl38x_range_t);
FAKE_VALUE_FUNC(w_status_t, adxl38x_register_update_bits, adxl38x_dev_t *, uint8_t, uint8_t, uint8_t);
FAKE_VALUE_FUNC(w_status_t, adxl38x_read_device_data, adxl38x_dev_t *, uint8_t, uint8_t, uint8_t);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
}

class ADXL380 : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(gpio_read);
        RESET_FAKE(gpio_write);

        RESET_FAKE(adxl38x_init);
        RESET_FAKE(adxl38x_soft_reset);
        RESET_FAKE(adxl38x_set_op_mode);
        RESET_FAKE(adxl38x_set_range);
        RESET_FAKE(adxl38x_register_update_bits);

        RESET_FAKE(adxl38x_read_device_data);

        RESET_FAKE(log_text);
        FFF_RESET_HISTORY(); 
    }

    void TearDown() override {}
};

TEST_F(ADXL380, initSuccess){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_SUCCESS, status);
};

TEST_F(ADXL380, initFailWithADXL38XInitFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_FAILURE;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XInitSettingFailure){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_FAILURE;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSettingFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_FAILURE;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XUpdateBitFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_FAILURE;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XUpdateBitFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_FAILURE;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XUpdateBitFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_FAILURE;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, getRawAccelFail){

    // set up function returns
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, getRawAccelSuccess){

    // set up function returns
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_FAILURE, status);
};