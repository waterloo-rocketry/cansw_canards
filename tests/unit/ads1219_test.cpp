#include "fff.h"
#include <gtest/gtest.h>
#include<iostream>

extern "C" {
#include <stdint.h>
#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADS1219.h"
#include "drivers/i2c/i2c.h"

extern w_status_t ads1219_read_value(ads1219_handle_t *handle, uint32_t *value);

FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, i2c_write_data, i2c_bus_t, uint8_t, const uint8_t *, uint8_t);

}

uint32_t i2c_test_input;
ads1219_handle_t test_handle = {.bus = I2C_BUS_4, .i2c_addr = 0x40};
w_status_t i2c_read_reg_custom_output(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len){

    uint8_t test_num_array[3] = {(uint8_t) ((((uint32_t) i2c_test_input) & 0xFF0000) >> 16), (uint8_t) ((((uint32_t) i2c_test_input) & 0xFF00) >> 8), static_cast<uint8_t>((uint32_t) i2c_test_input & 0xFF)};
    uint8_t* i2c_custom_output = (uint8_t*) test_num_array;
    data[0] = i2c_custom_output[0];
    data[1] = i2c_custom_output[1];
    data[2] = i2c_custom_output[2];
    return W_SUCCESS;
}

class ADS1219Test : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(i2c_write_reg);
        RESET_FAKE(i2c_read_reg);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(ADS1219Test, readValueTestBitConcate0){

    // set up the i2c value
    i2c_test_input = 0;
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_output;

    uint32_t result_num = 1;
    w_status_t status= ads1219_read_value(&test_handle, &result_num);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(i2c_test_input, (int32_t) result_num);
};

TEST_F(ADS1219Test, readValueTestBitConcateMax){

    // set up the i2c value
    i2c_test_input = 0x7FFFFF;
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_output;

    uint32_t result_num = 1;
    w_status_t status= ads1219_read_value(&test_handle, &result_num);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(i2c_test_input, (int32_t) result_num);
};

TEST_F(ADS1219Test, readValueTestBitConcateMin){

    // set up the i2c value
    i2c_test_input = (uint32_t) 0xFF800001;
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_output;

    uint32_t result_num = 1;
    w_status_t status= ads1219_read_value(&test_handle, &result_num);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(i2c_test_input, (int32_t) result_num);
};

TEST_F(ADS1219Test, readValueTestOverflowMax){

    // set up the i2c value
    i2c_test_input = 0x7FFFFF + 1;
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_output;

    uint32_t result_num = 1;
    w_status_t status= ads1219_read_value(&test_handle, &result_num);
    EXPECT_EQ(W_OVERFLOW, status);
};

TEST_F(ADS1219Test, readValueTestOverflowMin){

    // set up the i2c value
    i2c_test_input = (uint32_t) (0xFF800001);
    i2c_test_input--;
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_output;

    uint32_t result_num = 1;
    w_status_t status= ads1219_read_value(&test_handle, &result_num);
    EXPECT_EQ(W_OVERFLOW, status);
};