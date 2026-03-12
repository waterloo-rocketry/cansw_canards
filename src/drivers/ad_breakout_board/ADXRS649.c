#include "drivers/ad_breakout_board/ADXRS649.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/ad_breakout_board/ads1219_stm32.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"

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

// ADC constants
static float32_t V_EXTERNAL_REF_P_mV = 5000.0f;
static float32_t V_EXTERNAL_REF_N_mV = 0.0f;

// global adc handle
static ads1219_handle_t adc_handle = {};

/**
 * @brief perform the self-test on the ADXRS649
 * @return the status of the self-test
 */
static w_status_t adxrs649_self_test() {
	w_status_t status = W_SUCCESS;
	float32_t adc_voltage; // will be mV
	int32_t test_num = 0;

	// SELF-TEST 1
	status |=
		gpio_write(GPIO_PIN_ADXRS649_ST1, GPIO_LEVEL_HIGH, 1); // TODO: create new GPIO pin for test

	while ((MAX_NUM_TESTS >= test_num) && ((-1 * MIN_SELF_TEST_mV) > adc_voltage) &&
		   ((-1 * MAX_SELF_TEST_mV) < adc_voltage)) {
		// wait and retest
		vTaskDelay(pdMS_TO_TICKS(2));
		status |= ads1219_get_millivolts(&adc_handle, &adc_voltage);
		test_num++;
	}

	// make sure self-test 1 is turned off. Make sure the command is sent regardless of previous
	// status
	if (W_SUCCESS != gpio_write(GPIO_PIN_ADXRS649_ST1,
								GPIO_LEVEL_LOW,
								1)) { // TODO: create new GPIO pin for test
		// TODO: logging message Critical error
		return W_FAILURE;
	}

	if (MAX_NUM_TESTS < test_num) {
		return W_FAILURE;
	}

	// SELF-TEST 2
	status |=
		gpio_write(GPIO_PIN_ADXRS649_ST2, GPIO_LEVEL_HIGH, 1); // TODO: create new GPIO pin for test

	while ((MAX_NUM_TESTS >= test_num) && ((MIN_SELF_TEST_mV) < adc_voltage) &&
		   ((MAX_SELF_TEST_mV) > adc_voltage)) {
		// wait and retest
		vTaskDelay(pdMS_TO_TICKS(2));
		status |= ads1219_get_millivolts(&adc_handle, &adc_voltage);
		test_num++;
	}

	// make sure self-test 2 is turned off. Make sure the command is sent regardless of previous
	// status
	if (W_SUCCESS != gpio_write(GPIO_PIN_ADXRS649_ST2,
								GPIO_LEVEL_LOW,
								1)) { // TODO: create new GPIO pin for test
		// TODO: logging message Critical error
		return W_FAILURE;
	}
	if (MAX_NUM_TESTS < test_num) {
		return W_FAILURE;
	}

	return status;
}

/**
 * @brief initialize and start up the ADXRS649 AD Gyro and ADS1219
 * @return the status at which the ADXRS649 initalization goes
 */
w_status_t adxrs649_init() {
	if (W_SUCCESS != ads1219_init(&adc_handle,
								  I2C_BUS_ADC,
								  ADS1219_ADDR)) { // TODO: to be set once an I2C bus is determined
		// TODO: logging message
		return W_FAILURE;
	}

	// set up ADC
	w_status_t adc_setup_status = ads1219_set_channel(&adc_handle, ADS1219_MUX_SINGLE_1);
	adc_setup_status |= ads1219_set_conversion_mode(&adc_handle, ADS1219_CM_CONTINUOUS);
	adc_setup_status |= ads1219_set_gain(&adc_handle, ADS1219_GAIN_ONE);
	adc_setup_status |= ads1219_set_data_rate(&adc_handle, ADS1219_DATARATE_1000SPS);
	adc_setup_status |= ads1219_set_vref(
		&adc_handle, ADS1219_VREF_EXTERNAL, V_EXTERNAL_REF_N_mV, V_EXTERNAL_REF_P_mV);

	if (W_SUCCESS != adc_setup_status) {
		// TODO: logging message
		return adc_setup_status;
	}

	if (W_SUCCESS != adxrs649_self_test()) {
		// TODO: logging message
		return W_FAILURE;
	}

	return W_SUCCESS;
}