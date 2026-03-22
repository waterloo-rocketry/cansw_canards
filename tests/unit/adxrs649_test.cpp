#include "fff.h"
#include <gtest/gtest.h>
#include "utils/math_testing_helpers.hpp"
#include<iostream>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include <stdint.h>
#include <stdbool.h>
#include "drivers/ad_breakout_board/ADS1219.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/i2c/i2c.h"
#include "drivers/gpio/gpio.h"

extern w_status_t adxrs649_init();
extern w_status_t adxrs649_get_gyro_data(float *data, uint32_t *raw_data);

FAKE_VALUE_FUNC(w_status_t, gpio_read, gpio_pin_t, gpio_level_t*, uint32_t);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t);

FAKE_VALUE_FUNC(w_status_t, ads1219_get_millivolts, ads1219_handle_t *, float *);
FAKE_VALUE_FUNC(w_status_t, ads1219_init, ads1219_handle_t *, i2c_bus_t, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_channel, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_conversion_mode, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_gain, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_data_rate, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_vref, ads1219_handle_t *, uint8_t, float, float);
FAKE_VALUE_FUNC(w_status_t, ads1219_start, ads1219_handle_t *);
FAKE_VALUE_FUNC(w_status_t, ads1219_conversion_ready, ads1219_handle_t *, bool *);
FAKE_VALUE_FUNC(w_status_t, ads1219_read_value, ads1219_handle_t *, uint32_t *);
FAKE_VALUE_FUNC(w_status_t, ads1219_millivolts, ads1219_handle_t *, int32_t, float *);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
}

class ADS1219Test : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(gpio_read);
        RESET_FAKE(gpio_write);

        RESET_FAKE(ads1219_get_millivolts);
        RESET_FAKE(ads1219_init);
        RESET_FAKE(ads1219_set_channel);
        RESET_FAKE(ads1219_set_conversion_mode);
        RESET_FAKE(ads1219_set_gain);
        RESET_FAKE(ads1219_set_vref);
        RESET_FAKE(ads1219_start);
        RESET_FAKE(ads1219_conversion_ready);
        RESET_FAKE(ads1219_read_value);
        RESET_FAKE(ads1219_millivolts);

        RESET_FAKE(log_text);
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