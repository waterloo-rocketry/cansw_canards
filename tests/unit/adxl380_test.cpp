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
extern w_status_t adxl380_get_raw_accel(adxl380_raw_accel_data_t *raw_data);
extern w_status_t adxl380_get_accel_data(vector3d_t *data, adxl380_raw_accel_data_t *raw_data);

FAKE_VALUE_FUNC(w_status_t, adxl38x_init, adxl38x_dev_t *);
FAKE_VALUE_FUNC(w_status_t, adxl38x_soft_reset, adxl38x_dev_t *);
FAKE_VALUE_FUNC(w_status_t, adxl38x_selftest, adxl38x_dev_t *, adxl38x_op_mode_t, bool *, bool *, bool *);
FAKE_VALUE_FUNC(w_status_t, adxl38x_set_op_mode, adxl38x_dev_t *, adxl38x_op_mode_t);
FAKE_VALUE_FUNC(w_status_t, adxl38x_set_range, adxl38x_dev_t *, adxl38x_range_t);
FAKE_VALUE_FUNC(w_status_t, adxl38x_register_update_bits, adxl38x_dev_t *, uint8_t, uint8_t, uint8_t);
FAKE_VALUE_FUNC(w_status_t, adxl38x_read_device_data, adxl38x_dev_t *, uint8_t, uint16_t, uint8_t *);
FAKE_VALUE_FUNC(uint8_t, adxl38x_field_prep_u8, uint8_t, uint8_t);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *, ...);

FAKE_VALUE_FUNC(w_status_t, gpio_read, gpio_pin_t, gpio_level_t*, uint32_t);
}

typedef struct {
    w_status_t return_status;
    bool x_status;
    bool y_status;
    bool z_status;
} adxl38x_self_test_result_t;

adxl38x_self_test_result_t g_self_test_result;

w_status_t adxl38x_self_test_set_result(adxl38x_dev_t *p_dev, adxl38x_op_mode_t op_mode, bool *p_st_x, bool *p_st_y, bool *p_st_z) {
    *p_st_x = g_self_test_result.x_status;
    *p_st_y = g_self_test_result.y_status;
    *p_st_z = g_self_test_result.z_status;

    return g_self_test_result.return_status;
}

adxl380_raw_accel_data_t g_test_device_data_raw;
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


uint8_t raw_single_data = 100;
w_status_t adxl38x_set_raw_single_read_device_data(adxl38x_dev_t *dev, uint8_t base_address, uint16_t size, uint8_t *read_data){

    *read_data = raw_single_data;
    return W_SUCCESS;
}


gpio_level_t global_gpio_value = GPIO_LEVEL_HIGH;
w_status_t gpio_set_read(gpio_pin_t pin, gpio_level_t *level, uint32_t timeout) {
    *level = global_gpio_value;
    return W_SUCCESS;
}

class ADXL380 : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(adxl38x_init);
        RESET_FAKE(adxl38x_soft_reset);
        RESET_FAKE(adxl38x_selftest);
        RESET_FAKE(adxl38x_set_op_mode);
        RESET_FAKE(adxl38x_set_range);
        RESET_FAKE(adxl38x_register_update_bits);

        RESET_FAKE(adxl38x_read_device_data);
        RESET_FAKE(adxl38x_field_prep_u8);

        RESET_FAKE(log_text);
        RESET_FAKE(gpio_read);
        FFF_RESET_HISTORY(); 

        // preset the self-test to pass
        g_self_test_result.return_status = W_SUCCESS;
        g_self_test_result.x_status = true;
        g_self_test_result.y_status = true;
        g_self_test_result.z_status = true;

        adxl38x_selftest_fake.custom_fake = adxl38x_self_test_set_result;
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

TEST_F(ADXL380, initFailWithADXL38XSelfTestFailedComplete){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;
    
    //self-test
    g_self_test_result.return_status = W_FAILURE;
    g_self_test_result.x_status = true;
    g_self_test_result.y_status = true;
    g_self_test_result.z_status = true;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSelfTestXFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;
    
    //self-test
    g_self_test_result.return_status = W_SUCCESS;
    g_self_test_result.x_status = false;
    g_self_test_result.y_status = true;
    g_self_test_result.z_status = true;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSelfTestYFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;
    
    //self-test
    g_self_test_result.return_status = W_SUCCESS;
    g_self_test_result.x_status = true;
    g_self_test_result.y_status = false;
    g_self_test_result.z_status = true;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSelfTestZFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;
    
    //self-test
    g_self_test_result.return_status = W_SUCCESS;
    g_self_test_result.x_status = true;
    g_self_test_result.y_status = true;
    g_self_test_result.z_status = false;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSelfTestAllFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;
    
    //self-test
    g_self_test_result.return_status = W_SUCCESS;
    g_self_test_result.x_status = false;
    g_self_test_result.y_status = false;
    g_self_test_result.z_status = false;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSelfTestTwoFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;
    
    //self-test
    g_self_test_result.return_status = W_SUCCESS;
    g_self_test_result.x_status = false;
    g_self_test_result.y_status = false;
    g_self_test_result.z_status = true;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSoftResetFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_FAILURE;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_SUCCESS;
    adxl38x_register_update_bits_fake.return_val = W_SUCCESS;

    w_status_t status= adxl380_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, initFailWithADXL38XSettingOPModeFailure){

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

TEST_F(ADXL380, initFailWithADXL38XSettingRangeFail){

    // set up function returns
    adxl38x_init_fake.return_val = W_SUCCESS;

    // set up
    adxl38x_soft_reset_fake.return_val = W_SUCCESS;
    adxl38x_set_op_mode_fake.return_val = W_SUCCESS;
    adxl38x_set_range_fake.return_val = W_FAILURE;
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
    adxl380_init();

    // set up function returns
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    adxl380_raw_accel_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, getRawAccelSuccessRegular){
    adxl380_init();

    // set up function returns
    g_test_device_data_raw.x = 0xF840;
    g_test_device_data_raw.y = 0x7D0;
    g_test_device_data_raw.z = 0x1;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    adxl380_raw_accel_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(g_test_device_data_raw.x, raw_data.x);
    EXPECT_EQ(g_test_device_data_raw.y, raw_data.y);
    EXPECT_EQ(g_test_device_data_raw.z, raw_data.z);
};

TEST_F(ADXL380, getRawAccelSuccessEdgeCase){
    adxl380_init();

    // set up function returns
    g_test_device_data_raw.x = 0x7FFF;
    g_test_device_data_raw.y = 0x8001;
    g_test_device_data_raw.z = 0x0;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    adxl380_raw_accel_data_t raw_data = {0};

    w_status_t status= adxl380_get_raw_accel(&raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(g_test_device_data_raw.x, raw_data.x);
    EXPECT_EQ(g_test_device_data_raw.y, raw_data.y);
    EXPECT_EQ(g_test_device_data_raw.z, raw_data.z);
};



TEST_F(ADXL380, getConvAccelFail){
    adxl380_init();

    // set up function returns
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    vector3d_t data = {0};
    adxl380_raw_accel_data_t raw_data = {0};

    w_status_t status= adxl380_get_accel_data(&data, &raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXL380, getConvAccelSuccessRegular){
    adxl380_init();

    // set up function returns
    g_test_device_data_raw.x = 0xF840;
    g_test_device_data_raw.y = 0x7D0;
    g_test_device_data_raw.z = 0x1;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    vector3d_t data = {0};
    adxl380_raw_accel_data_t raw_data = {0};

    w_status_t status= adxl380_get_accel_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ(-1984 * 533.3 / 1000000, data.x);
    EXPECT_FLOAT_EQ(2000 * 533.3 / 1000000, data.y);
    EXPECT_FLOAT_EQ(1 * 533.3 / 1000000, data.z);
};

TEST_F(ADXL380, getConvAccelSuccessEdgeCase){
    adxl380_init();

    // set up function returns
    g_test_device_data_raw.x = 0x7FFF;
    g_test_device_data_raw.y = 0x8001;
    g_test_device_data_raw.z = 0x0;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_read_device_data;

    vector3d_t data = {0};
    adxl380_raw_accel_data_t raw_data = {0};

    w_status_t status= adxl380_get_accel_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ(32767 * 533.3 / 1000000, data.x);
    EXPECT_FLOAT_EQ(-32767 * 533.3 / 1000000, data.y);
    EXPECT_FLOAT_EQ(0 * 533.3 / 1000000, data.z);
};


TEST_F(ADXL380, getDrdyFailCauseGPIOAndReadFail){
    adxl380_init();

    // set up function returns
    gpio_read_fake.return_val = W_FAILURE;
    adxl38x_read_device_data_fake.return_val = W_FAILURE;

    bool drdy = false;

    w_status_t status= adxl380_is_data_ready(&drdy);
    EXPECT_EQ(W_IO_ERROR, status);
};


TEST_F(ADXL380, getDrdySuccessNotReadyGPIOFailAndReadSuccess){
    adxl380_init();

    // set up function returns
    gpio_read_fake.return_val = W_FAILURE;
    raw_single_data = 0;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_single_read_device_data;

    bool drdy = true;

    w_status_t status= adxl380_is_data_ready(&drdy);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(false, drdy);
};


TEST_F(ADXL380, getDrdySuccessReadyGPIO){
    adxl380_init();

    // set up function returns
    global_gpio_value = GPIO_LEVEL_HIGH;
    gpio_read_fake.custom_fake = gpio_set_read;
    raw_single_data = 0;
    adxl38x_read_device_data_fake.custom_fake = adxl38x_set_raw_single_read_device_data;

    bool drdy = false;

    w_status_t status= adxl380_is_data_ready(&drdy);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(true, drdy);
};