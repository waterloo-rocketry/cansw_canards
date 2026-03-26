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
FAKE_VALUE_FUNC(w_status_t, adxl38x_read_device_data, adxl38x_dev_t *, uint8_t, uint16_t, uint8_t *);
FAKE_VALUE_FUNC(uint8_t, adxl38x_field_prep_u8, uint8_t, uint8_t);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
}

altimu_raw_imu_data_t g_test_device_data_raw;
w_status_t adxl38x_set_raw_read_device_data(adxl38x_dev_t *dev, uint8_t base_address, uint16_t size, uint8_t *read_data){

    uint8_t test_num_array[6] = {(uint8_t) ((g_test_device_data_raw.x & 0xFF00) >> 8), (uint8_t) (g_test_device_data_raw.x & 0xFF), (uint8_t) ((g_test_device_data_raw.y & 0xFF00) >> 8), (uint8_t) (g_test_device_data_raw.y & 0xFF), (uint8_t) ((g_test_device_data_raw.z & 0xFF00) >> 8), (uint8_t) (g_test_device_data_raw.z & 0xFF)};
    read_data[0] = test_num_array[0];
    read_data[1] = test_num_array[1];
    read_data[2] = test_num_array[2];
    read_data[3] = test_num_array[3];
    read_data[4] = test_num_array[4];
    read_data[5] = test_num_array[5];
    return W_SUCCESS;
}

class ADXL380 : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(adxl38x_init);
        RESET_FAKE(adxl38x_soft_reset);
        RESET_FAKE(adxl38x_set_op_mode);
        RESET_FAKE(adxl38x_set_range);
        RESET_FAKE(adxl38x_register_update_bits);

        RESET_FAKE(adxl38x_read_device_data);
        RESET_FAKE(adxl38x_field_prep_u8);

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

TEST_F(ADXL380, getRawAccelFail){

    // set up function returns
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, getRawAccelSuccessRegular){

    // set up function returns
    g_test_device_data_raw.x = 0xF840;
    g_test_device_data_raw.y = 0x7D0;
    g_test_device_data_raw.z = 0x1;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(g_test_device_data_raw.x, raw_data.x);
    EXPECT_EQ(g_test_device_data_raw.y, raw_data.y);
    EXPECT_EQ(g_test_device_data_raw.z, raw_data.z);
};

TEST_F(ADXL380, getRawAccelSuccessEdgeCase){

    // set up function returns
    g_test_device_data_raw.x = 0x7FFF;
    g_test_device_data_raw.y = 0x8001;
    g_test_device_data_raw.z = 0x0;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(g_test_device_data_raw.x, raw_data.x);
    EXPECT_EQ(g_test_device_data_raw.y, raw_data.y);
    EXPECT_EQ(g_test_device_data_raw.z, raw_data.z);
};



TEST_F(ADXL380, getConvAccelFail){

    // set up function returns
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    vector3d_t data = {0};
    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_accel_data(&data, &raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, getConvAccelSuccessRegular){

    // set up function returns
    g_test_device_data_raw.x = 0xF840;
    g_test_device_data_raw.y = 0x7D0;
    g_test_device_data_raw.z = 0x1;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    vector3d_t data = {0};
    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_accel_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ(-1984 * 533.3 / 1000000, data.x);
    EXPECT_FLOAT_EQ(2000 * 533.3 / 1000000, data.y);
    EXPECT_FLOAT_EQ(1 * 533.3 / 1000000, data.z);
};

TEST_F(ADXL380, getConvAccelSuccessEdgeCase){

    // set up function returns
    g_test_device_data_raw.x = 0x7FFF;
    g_test_device_data_raw.y = 0x8001;
    g_test_device_data_raw.z = 0x0;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    vector3d_t data = {0};
    altimu_raw_imu_data_t raw_data = {0};

    w_status_t status= adxl380_get_accel_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ(32767 * 533.3 / 1000000, data.x);
    EXPECT_FLOAT_EQ(-32767 * 533.3 / 1000000, data.y);
    EXPECT_FLOAT_EQ(0 * 533.3 / 1000000, data.z);
};