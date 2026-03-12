#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/ad_breakout_board/ads1219_stm32.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "FreeRTOS.h"

// define addresses
#define ADS1219_ADDR 0x00 // TODO: to be set once the address has been determind

// sensor range
static const float32_t ADXRS649_GYRO_RANGE = 20000.0;

// conversion factors
static const float32_t ADXRS649_CONV_mV_DEG_S = 0.1; // from datasheet

// self-test constants
static const float32_t MIN_SELF_TEST_mV = 130; // magnitude
static const float32_t MAX_SELF_TEST_mV = 170; // magnitude
static const int32_t MAX_NUM_TESTS = 2; // unitless

/**
 * @brief perform the self-test on the ADXRS649
 * @param adc_handle is the pointer to adc handle
 * @return the status of the self-test
 */
w_status_t adxrs649_self_test(ads1219_handle_t *adc_handle) {
    w_status_t status = W_SUCCESS;
    float32_t adc_voltage; // will be mV
    int32_t test_num = 0;

    // SELF-TEST 1
    status |= gpio_write(GPIO_PIN_ADXRS649_ST1, GPIO_LEVEL_HIGH, 10); // TODO: create new GPIO pin for test

    while ((MAX_NUM_TESTS >= test_num) && ((-1 * MIN_SELF_TEST_mV) > adc_voltage) && ((-1 * MAX_SELF_TEST_mV) < adc_voltage)) {
        // wait and retest
        vTaskDelay(pdMS_TO_TICKS(2));
        status |= ads1219_get_millivolts(adc_handle, &adc_voltage);
        test_num++;
    }

    // make sure self-test 1 is turned off
    status |= gpio_write(GPIO_PIN_ADXRS649_ST1, GPIO_LEVEL_LOW, 10); // TODO: create new GPIO pin for test

    if (MAX_NUM_TESTS < test_num) {
        return W_FAILURE;
    }

    // SELF-TEST 2
    status |= gpio_write(GPIO_PIN_ADXRS649_ST2, GPIO_LEVEL_HIGH, 10); // TODO: create new GPIO pin for test

    while ((MAX_NUM_TESTS >= test_num) && ((MIN_SELF_TEST_mV) < adc_voltage) && ((MAX_SELF_TEST_mV) > adc_voltage)) {
        // wait and retest
        vTaskDelay(pdMS_TO_TICKS(2));
        status |= ads1219_get_millivolts(adc_handle, &adc_voltage);
        test_num++;
    }

    // make sure self-test 1 is turned off
    status |= gpio_write(GPIO_PIN_ADXRS649_ST2, GPIO_LEVEL_LOW, 10); // TODO: create new GPIO pin for test

    if (MAX_NUM_TESTS < test_num) {
        return W_FAILURE;
    }

    return status;
}