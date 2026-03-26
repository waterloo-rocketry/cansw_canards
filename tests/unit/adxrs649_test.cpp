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

gpio_level_t global_gpio_value = GPIO_LEVEL_HIGH;
w_status_t gpio_return_value = W_SUCCESS;
w_status_t gpio_set_read(gpio_pin_t pin, gpio_level_t *level, uint32_t timeout) {
    *level = global_gpio_value;
    return gpio_return_value;
}


bool global_ads1219_conversion_ready = false;
w_status_t ads1219_conversion_ready_return = W_SUCCESS;
w_status_t ads1219_set_conversion_ready(ads1219_handle_t *handle, bool *ready) {
    *ready = global_ads1219_conversion_ready;
    return ads1219_conversion_ready_return;
}

float global_adc_output_mv = 0;
w_status_t ads1219_set_millivolts_return = W_SUCCESS;
w_status_t ads1219_set_millivolts(ads1219_handle_t *handle, const int32_t adc_count, float *mv) {
    *mv = global_adc_output_mv;
    return ads1219_set_millivolts_return;
}

w_status_t ads1219_read_value_set_data(ads1219_handle_t *handle, uint32_t *value) {
    *value = 1;
    return W_SUCCESS;
}

class ADXRS649 : public ::testing::Test {
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
        global_gpio_value = GPIO_LEVEL_HIGH;
        gpio_return_value = W_SUCCESS;  
        global_ads1219_conversion_ready = false;
        ads1219_conversion_ready_return = W_SUCCESS;  
        global_adc_output_mv = 0;
        ads1219_set_millivolts_return = W_SUCCESS;   
    }

    void TearDown() override {}
};

TEST_F(ADXRS649, initSuccess){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_SUCCESS, status);
};

TEST_F(ADXRS649, initFailAfterADS1219InitFail){

    // set up function returns
    ads1219_init_fake.return_val = W_FAILURE;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, initFailAfterSetUpFail){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_FAILURE;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_NE(W_SUCCESS, status);
};

// unable to fake self-test since it's static function

TEST_F(ADXRS649, initFailAfterStartFail){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;

    // start
    ads1219_start_fake.return_val = W_FAILURE;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, getGyroDataFailByBothDRDYFail){

    // read DRDY
    gpio_read_fake.return_val = W_FAILURE;
    ads1219_conversion_ready_fake.return_val = W_FAILURE;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;
    ads1219_millivolts_fake.return_val = W_SUCCESS;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_IO_ERROR, status);
};

TEST_F(ADXRS649, getGyroDataFailByGPIORead0){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_HIGH;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;

    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;
    ads1219_millivolts_fake.return_val = W_SUCCESS;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, getGyroDataFailWithGPIOFailADS1219SuceessNotReady){

    // read DRDY
    gpio_read_fake.return_val = W_FAILURE;

    global_ads1219_conversion_ready = false;
    ads1219_conversion_ready_return = W_SUCCESS;
    ads1219_conversion_ready_fake.custom_fake = ads1219_set_conversion_ready;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    ads1219_millivolts_fake.return_val = W_SUCCESS;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(1, ads1219_conversion_ready_fake.call_count);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, getGyroDataFailByReadValFail){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_LOW;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;
    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_FAILURE;
    ads1219_millivolts_fake.return_val = W_SUCCESS;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_IO_ERROR, status);
};

TEST_F(ADXRS649, getGyroDataFailByConvFail){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_LOW;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;
    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;
    ads1219_millivolts_fake.return_val = W_FAILURE;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorMax){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_LOW;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;
    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 4500;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(((global_adc_output_mv - 2500) / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorMin){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_LOW;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;
    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 500;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(((global_adc_output_mv - 2500) / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorZero){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_LOW;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;
    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 0;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(((global_adc_output_mv - 2500) / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorRegular){

    // read DRDY
    global_gpio_value = GPIO_LEVEL_LOW;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;
    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 1000;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(((global_adc_output_mv - 2500) / 0.1), data);
    EXPECT_EQ(1, raw_data);
};