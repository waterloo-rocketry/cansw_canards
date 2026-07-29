#include "fff.h"
#include <gtest/gtest.h>
#include "utils/math_testing_helpers.hpp"
#include<iostream>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include <stdint.h>
#include <stdbool.h>
#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADS1219.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/i2c/i2c.h"
#include "drivers/gpio/gpio.h"

extern w_status_t adxrs649_init();
extern w_status_t adxrs649_get_gyro_data(float64_t *data, uint32_t *raw_data);

FAKE_VALUE_FUNC(w_status_t, gpio_read, gpio_pin_t, gpio_level_t*, uint32_t);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t);

FAKE_VALUE_FUNC(w_status_t, ads1219_get_millivolts, ads1219_handle_t *, float64_t *);
FAKE_VALUE_FUNC(w_status_t, ads1219_init, ads1219_handle_t *, i2c_bus_t, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_channel, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_conversion_mode, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_gain, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_data_rate, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_set_vref, ads1219_handle_t *, uint8_t, float64_t, float64_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_sanity_check, ads1219_handle_t *, uint8_t);
FAKE_VALUE_FUNC(w_status_t, ads1219_start, ads1219_handle_t *);
FAKE_VALUE_FUNC(w_status_t, ads1219_conversion_ready, ads1219_handle_t *, bool *);
FAKE_VALUE_FUNC(w_status_t, ads1219_read_value, ads1219_handle_t *, uint32_t *);
FAKE_VALUE_FUNC(w_status_t, ads1219_millivolts, ads1219_handle_t *, int32_t, float64_t *);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *, ...);
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

float64_t global_adc_output_mv = 0;
w_status_t ads1219_set_millivolts_return = W_SUCCESS;
w_status_t ads1219_set_millivolts(ads1219_handle_t *handle, const int32_t adc_count, float64_t *mv) {
    *mv = global_adc_output_mv;
    return ads1219_set_millivolts_return;
}

w_status_t ads1219_read_value_set_data(ads1219_handle_t *handle, uint32_t *value) {
    *value = 1;
    return W_SUCCESS;
}

uint8_t count1_ST = 0;
w_status_t ads1219_ST_set_get_millivolts1(ads1219_handle_t *handle, float64_t *mv) {
    if (0 == count1_ST) {
        *mv = 150;
    } else if (2 == count1_ST) {
        *mv = -150;
    } else {
        *mv = 0;
    }

    count1_ST++;

    return W_SUCCESS;
}

uint8_t count2_ST = 0;
w_status_t ads1219_ST1_fail_set_get_millivolts(ads1219_handle_t *handle, float64_t *mv) {
    if (0 == count2_ST) {
        *mv = 90;
    } else if (2 == count2_ST) {
        *mv = -150;
    } else {
        *mv = 0;
    }

    count2_ST++;

    return W_SUCCESS;
}

uint8_t count3_ST = 0;
w_status_t ads1219_ST1_fail_set_get_millivolts2(ads1219_handle_t *handle, float64_t *mv) {
    if (0 == count3_ST) {
        *mv = 210;
    } else if (2 == count3_ST) {
        *mv = -150;
    } else {
        *mv = 0;
    }

    count3_ST++;

    return W_SUCCESS;
}


uint8_t count4_ST = 0;
w_status_t ads1219_ST2_fail_set_get_millivolts(ads1219_handle_t *handle, float64_t *mv) {
    if (0 == count4_ST) {
        *mv = 150;
    } else if (2 == count4_ST) {
        *mv = -90;
    } else {
        *mv = 0;
    }

    count4_ST++;

    return W_SUCCESS;
}

uint8_t count5_ST = 0;
w_status_t ads1219_ST2_fail_set_get_millivolts2(ads1219_handle_t *handle, float64_t *mv) {
    if (0 == count5_ST) {
        *mv = 150;
    } else if (2 == count5_ST) {
        *mv = -210;
    } else {
        *mv = 0;
    }

    count5_ST++;

    return W_SUCCESS;
}


void successful_adxrs649_init() {
     // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST_set_get_millivolts1;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    adxrs649_init();
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
        RESET_FAKE(ads1219_sanity_check);
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
        count1_ST = 0;
        count2_ST = 0;
        count3_ST = 0;
        count4_ST = 0;
        count5_ST = 0;
    }

    void TearDown() override {}
};

// NOTE: this must remain the first TEST_F in this file (declaration order == execution order
// for a gtest fixture) since `is_initialized` is a file-static in ADXRS649.c that starts false
// and is permanently flipped true by the first successful adxrs649_init() call below.
TEST_F(ADXRS649, getStatusNotInitialized){
    health_status_t status = adxrs649_get_status();

    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_ERROR, status.severity);
    EXPECT_EQ(CANARDS_MODULE_ID_ADXRS649, status.module_id);
    EXPECT_NE(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_NOT_INIT_OFFSET));
    EXPECT_EQ(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET));
    EXPECT_EQ(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_COMM_FAILURE_OFFSET));
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

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST_set_get_millivolts1;

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

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST_set_get_millivolts1;

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

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST_set_get_millivolts1;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_NE(W_SUCCESS, status);
};

// unable to fake self-test since it's static function

TEST_F(ADXRS649, initFailAfterADCSanityCheckFail){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_FAILURE;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST_set_get_millivolts1;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, initFailAfterStartFail){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST_set_get_millivolts1;

    // start
    ads1219_start_fake.return_val = W_FAILURE;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

// ST 1 Fail Low
TEST_F(ADXRS649, initFailAfterST1FailLow){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST1_fail_set_get_millivolts;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

// ST 1 Fail High
TEST_F(ADXRS649, initFailAfterST1FailHigh){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST1_fail_set_get_millivolts2;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

// ST 2 Fail High
TEST_F(ADXRS649, initFailAfterST2FailHigh){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST2_fail_set_get_millivolts;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

// ST 2 Fail Low
TEST_F(ADXRS649, initFailAfterST2FailLow){

    // set up function returns
    ads1219_init_fake.return_val = W_SUCCESS;

    // set up
    ads1219_set_channel_fake.return_val = W_SUCCESS;
    ads1219_set_conversion_mode_fake.return_val = W_SUCCESS;
    ads1219_set_gain_fake.return_val = W_SUCCESS;
    ads1219_set_data_rate_fake.return_val = W_SUCCESS;
    ads1219_set_vref_fake.return_val = W_SUCCESS;

    // ADC Sanity Check
    ads1219_sanity_check_fake.return_val = W_SUCCESS;

    // self-test
    gpio_write_fake.return_val = W_SUCCESS;
    ads1219_get_millivolts_fake.custom_fake = ads1219_ST2_fail_set_get_millivolts2;

    // start
    ads1219_start_fake.return_val = W_SUCCESS;

    w_status_t status= adxrs649_init();
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, getDataReadyFailByBothDRDYFail){
    successful_adxrs649_init();

    // read DRDY
    gpio_read_fake.return_val = W_FAILURE;
    ads1219_conversion_ready_fake.return_val = W_FAILURE;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;
    ads1219_millivolts_fake.return_val = W_SUCCESS;

    bool drdy = false;
    w_status_t status= adxrs649_is_data_ready(&drdy);
    EXPECT_EQ(W_IO_ERROR, status);
};

TEST_F(ADXRS649, getDataReadySuccessNotReadyGPIOReadHigh){
    successful_adxrs649_init();

    // read DRDY
    global_gpio_value = GPIO_LEVEL_HIGH;
    gpio_return_value = W_SUCCESS; 
    gpio_read_fake.custom_fake = gpio_set_read;

    ads1219_conversion_ready_fake.return_val = W_SUCCESS;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;
    ads1219_millivolts_fake.return_val = W_SUCCESS;

    bool drdy = true;
    w_status_t status= adxrs649_is_data_ready(&drdy);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(false, drdy);
};

TEST_F(ADXRS649, getGyroDataSuccessNotReadyWithGPIOFailADS1219SuceessNotReady){
    successful_adxrs649_init();

    // read DRDY
    gpio_read_fake.return_val = W_FAILURE;

    global_ads1219_conversion_ready = false;
    ads1219_conversion_ready_return = W_SUCCESS;
    ads1219_conversion_ready_fake.custom_fake = ads1219_set_conversion_ready;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    ads1219_millivolts_fake.return_val = W_SUCCESS;

    bool drdy = true;
    w_status_t status= adxrs649_is_data_ready(&drdy);
    EXPECT_EQ(1, ads1219_conversion_ready_fake.call_count);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(false, drdy);
};

TEST_F(ADXRS649, getGyroDataSuccessReadyWithGPIOFailADS1219SuceessReady){
    successful_adxrs649_init();

    // read DRDY
    gpio_read_fake.return_val = W_FAILURE;

    global_ads1219_conversion_ready = true;
    ads1219_conversion_ready_return = W_SUCCESS;
    ads1219_conversion_ready_fake.custom_fake = ads1219_set_conversion_ready;

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    ads1219_millivolts_fake.return_val = W_SUCCESS;

    bool drdy = false;
    w_status_t status= adxrs649_is_data_ready(&drdy);
    EXPECT_EQ(1, ads1219_conversion_ready_fake.call_count);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(true, drdy);
};

TEST_F(ADXRS649, getGyroDataFailByReadValFail){
    successful_adxrs649_init();

    // read value
    ads1219_read_value_fake.return_val = W_FAILURE;
    ads1219_millivolts_fake.return_val = W_SUCCESS;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_IO_ERROR, status);
};

TEST_F(ADXRS649, getGyroDataFailByConvFail){
    successful_adxrs649_init();

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;
    ads1219_millivolts_fake.return_val = W_FAILURE;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_FAILURE, status);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorMax){

    successful_adxrs649_init();

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 4500;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ((global_adc_output_mv / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorMin){
    successful_adxrs649_init();

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 500;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ((global_adc_output_mv / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorZero){
    successful_adxrs649_init();

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 0;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ((global_adc_output_mv / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

TEST_F(ADXRS649, getGyroDataSuccessWithoutErrorRegular){
    successful_adxrs649_init();

    // read value
    ads1219_read_value_fake.return_val = W_SUCCESS;

    global_adc_output_mv = 1000;
    ads1219_set_millivolts_return = W_SUCCESS;
    ads1219_millivolts_fake.custom_fake = ads1219_set_millivolts;

    ads1219_read_value_fake.custom_fake = ads1219_read_value_set_data;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t status= adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_FLOAT_EQ((global_adc_output_mv / 0.1), data);
    EXPECT_EQ(1, raw_data);
};

// From this point on in the file, `is_initialized` is guaranteed true (set by the very first
// TEST_F above), so `adxrs649_get_status` will never report CANARDS_MODULE_E_NOT_INIT_OFFSET.
// `adxrs649_health` counters are cumulative file-statics with no test-visible reset, so each
// test below flushes any residual state with an initial call to adxrs649_get_status() (which
// clears the "recent_*" counters as a side effect) before asserting on newly-triggered failures.

TEST_F(ADXRS649, getStatusOkWhenHealthy){
    successful_adxrs649_init();

    // flush any residual error counters accumulated by earlier tests in this binary
    adxrs649_get_status();

    health_status_t status = adxrs649_get_status();

    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_OK, status.severity);
    EXPECT_EQ(CANARDS_MODULE_ID_ADXRS649, status.module_id);
    EXPECT_EQ(0u, status.error_bitfield);
};

TEST_F(ADXRS649, getStatusSetsCommFailureBitOnDataReadyCheckFailure){
    successful_adxrs649_init();
    adxrs649_get_status(); // flush residual state

    // force both the GPIO and ADS1219 data-ready paths to fail
    gpio_read_fake.return_val = W_FAILURE;
    ads1219_conversion_ready_fake.return_val = W_FAILURE;

    bool drdy = false;
    w_status_t drdy_status = adxrs649_is_data_ready(&drdy);
    EXPECT_EQ(W_IO_ERROR, drdy_status);

    health_status_t status = adxrs649_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_ERROR, status.severity);
    EXPECT_NE(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_COMM_FAILURE_OFFSET));

    // counters are cleared once read, so a subsequent call without new failures is healthy
    health_status_t status_after = adxrs649_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_OK, status_after.severity);
    EXPECT_EQ(0u, status_after.error_bitfield);
};

TEST_F(ADXRS649, getStatusSetsCommFailureBitOnReadValueFailure){
    successful_adxrs649_init();
    adxrs649_get_status(); // flush residual state

    ads1219_read_value_fake.return_val = W_FAILURE;

    float64_t data = 0;
    uint32_t raw_data = 0;
    w_status_t gyro_status = adxrs649_get_gyro_data(&data, &raw_data);
    EXPECT_EQ(W_IO_ERROR, gyro_status);

    health_status_t status = adxrs649_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_ERROR, status.severity);
    EXPECT_NE(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_COMM_FAILURE_OFFSET));
};

TEST_F(ADXRS649, getStatusSetsInvalidParamBitOnNullDrdyPtr){
    successful_adxrs649_init();
    adxrs649_get_status(); // flush residual state

    w_status_t drdy_status = adxrs649_is_data_ready(NULL);
    EXPECT_EQ(W_INVALID_PARAM, drdy_status);

    health_status_t status = adxrs649_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_ERROR, status.severity);
    EXPECT_NE(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET));

    // counter clears after being read
    health_status_t status_after = adxrs649_get_status();
    EXPECT_EQ(0u, status_after.error_bitfield & (1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET));
};

TEST_F(ADXRS649, getStatusSetsInvalidParamBitOnNullGyroDataPtrs){
    successful_adxrs649_init();
    adxrs649_get_status(); // flush residual state

    w_status_t gyro_status = adxrs649_get_gyro_data(NULL, NULL);
    EXPECT_EQ(W_INVALID_PARAM, gyro_status);

    health_status_t status = adxrs649_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_ERROR, status.severity);
    EXPECT_NE(0u, status.error_bitfield & (1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET));
};