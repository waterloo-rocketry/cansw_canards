
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADS1219.h"
#include "drivers/ad_breakout_board/ADS1219_reg.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"

// define addresses
static const uint16_t ADS1219_ADDR = 0x40;

// 1000 SPS (Samples per Second), Continuous Conversion, and External Ref
static const uint8_t ADS1219_CONFIG_SETTINGS =
	0x0F; // this is the configuration we are operating with

// sensor range
static const float64_t ADXRS649_GYRO_RANGE_DPS = 20000.0;

// conversion factors
static const float64_t MV_TO_DPS = 0.1; // from datasheet
static const float64_t NULL_BIAS_mV = 0.0 * 1000;

// self-test constants
static const float64_t MIN_SELF_TEST_mV = 100; // magnitude
static const float64_t MAX_SELF_TEST_mV = 200; // magnitude
static const uint32_t MAX_NUM_TESTS = 2; // unitless
static const uint8_t SELF_TEST_DELAY_MS = 10;
static const uint8_t INIT_DELAY_MS = 10;

// ADC constants
static float64_t V_EXTERNAL_REF_P_mV = 2048.0;
static float64_t V_EXTERNAL_REF_N_mV = 0.0;

typedef struct {
	uint32_t self_test_fails;
	uint32_t adc_sanity_check_fails;
	uint32_t recent_data_ready_check_fails;
	uint32_t recent_data_read_fails;
	uint32_t recent_null_params;
	uint32_t millivolt_conversion_fails;
} adxrs649_health_t;

// global adc handle
static ads1219_handle_t g_ads_handle = {};
static bool is_initialized = false;

static adxrs649_health_t adxrs649_health = {0};

/**
 * @brief perform the self-test on the ADXRS649
 * @return the status of the self-test
 */
static w_status_t adxrs649_self_test() {
	w_status_t status = W_SUCCESS;
	float64_t adc_voltage; // will be mV
	uint32_t test_num = 0;

	// SELF-TEST 1
	status |= gpio_write(GPIO_PIN_ADXRS649_ST1, GPIO_LEVEL_HIGH, 1);
	vTaskDelay(pdMS_TO_TICKS(SELF_TEST_DELAY_MS));

	status |= ads1219_get_millivolts(&g_ads_handle, &adc_voltage);

	// This is will exit either getting data within bounds or when we reach max retry
	while ((MAX_NUM_TESTS >= test_num) &&
		   ((MIN_SELF_TEST_mV > adc_voltage) || (MAX_SELF_TEST_mV < adc_voltage))) {
		// wait and retest
		vTaskDelay(pdMS_TO_TICKS(SELF_TEST_DELAY_MS));
		status |= ads1219_get_millivolts(&g_ads_handle, &adc_voltage);
		test_num++;
	}

	// make sure self-test 1 is turned off. Make sure the command is sent regardless of previous
	// status
	if (gpio_write(GPIO_PIN_ADXRS649_ST1, GPIO_LEVEL_LOW, 1) != W_SUCCESS) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed to set ST1 to LOW");
		return W_FAILURE;
	}

	if (MAX_NUM_TESTS < test_num) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed ST1.");
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(SELF_TEST_DELAY_MS));
	status |= ads1219_get_millivolts(&g_ads_handle, &adc_voltage);

	// SELF-TEST 2
	test_num = 0;

	vTaskDelay(pdMS_TO_TICKS(SELF_TEST_DELAY_MS));

	status |= gpio_write(GPIO_PIN_ADXRS649_ST2, GPIO_LEVEL_HIGH, 1);
	vTaskDelay(pdMS_TO_TICKS(SELF_TEST_DELAY_MS));

	status |= ads1219_get_millivolts(&g_ads_handle, &adc_voltage);

	// This is will exit either getting data within bounds or when we reach max retry
	while ((MAX_NUM_TESTS >= test_num) &&
		   (((-1 * MIN_SELF_TEST_mV) < adc_voltage) || ((-1 * MAX_SELF_TEST_mV) > adc_voltage))) {
		// wait and retest
		vTaskDelay(pdMS_TO_TICKS(SELF_TEST_DELAY_MS));
		status |= ads1219_get_millivolts(&g_ads_handle, &adc_voltage);
		test_num++;
	}

	// make sure self-test 2 is turned off. Make sure the command is sent regardless of previous
	// status
	if (gpio_write(GPIO_PIN_ADXRS649_ST2, GPIO_LEVEL_LOW, 1) != W_SUCCESS) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed to set ST2 to LOW");
		return W_FAILURE;
	}
	if (MAX_NUM_TESTS < test_num) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed ST2.");
		return W_FAILURE;
	}

	return status;
}

/**
 * @brief initialize and start up the ADXRS649 AD Gyro and ADS1219
 * @return the status at which the ADXRS649 initalization goes
 */
w_status_t adxrs649_init() {
	// don't reinit
	if (is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXRS649", "Reinitialization is not allowed.");
		return W_FAILURE;
	}

	is_initialized = true;

	// reset both gpio pins to low to start

	if (W_SUCCESS != ads1219_init(&g_ads_handle, I2C_BUS_2, ADS1219_ADDR)) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Unable to initialize the ADC.");
		return W_FAILURE;
	}

	// set up ADC
	w_status_t adc_setup_status = ads1219_set_channel(&g_ads_handle, ADS1219_MUX_DIFF_0_1);
	adc_setup_status |= ads1219_set_conversion_mode(&g_ads_handle, ADS1219_CM_CONTINUOUS);
	adc_setup_status |= ads1219_set_gain(&g_ads_handle, ADS1219_GAIN_ONE);
	adc_setup_status |= ads1219_set_data_rate(&g_ads_handle, ADS1219_DATARATE_1000SPS);
	adc_setup_status |= ads1219_set_vref(
		&g_ads_handle, ADS1219_VREF_EXTERNAL, V_EXTERNAL_REF_N_mV, V_EXTERNAL_REF_P_mV);

	if (adc_setup_status != W_SUCCESS) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed to write settings ADC for gyro.");
		return W_FAILURE;
	}

	if (ads1219_start(&g_ads_handle) != W_SUCCESS) {
		log_text(
			0, LOG_LVL_FATAL, "ADXRS649", "Failed to start continuous conversion for the ADC.");
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

	// currect ads setting 00001111 -> 0x0F
	// perform sanity check
	if (ads1219_sanity_check(&g_ads_handle, ADS1219_CONFIG_SETTINGS) != W_SUCCESS) {
		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed ADC sanity check.");
		adxrs649_health.adc_sanity_check_fails++;
		return W_FAILURE;
	}

	if (adxrs649_self_test() != W_SUCCESS) {
		// Make sure ST pins are low
		if (gpio_write(GPIO_PIN_ADXRS649_ST1, GPIO_LEVEL_LOW, 1) != W_SUCCESS) {
			log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed to set ST1 to LOW");
		}
		if (gpio_write(GPIO_PIN_ADXRS649_ST2, GPIO_LEVEL_LOW, 1) != W_SUCCESS) {
			log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed to set ST2 to LOW");
		}

		log_text(0, LOG_LVL_FATAL, "ADXRS649", "Failed gyro self-test.");
		adxrs649_health.self_test_fails++;
		return W_FAILURE;
	}

	if (ads1219_start(&g_ads_handle) != W_SUCCESS) {
		log_text(
			0, LOG_LVL_FATAL, "ADXRS649", "Failed to start continuous conversion for the ADC.");
		return W_FAILURE;
	}

	is_initialized = true;

	return W_SUCCESS;
}

/**
 * @brief gets the state of new data for the gyro
 * @param p_drdy a return pointer for if adxrs649 is data ready
 * @return the status of getting data from gyro
 */
w_status_t adxrs649_is_data_ready(bool *p_drdy) {
	if (!is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXRS649", "Failed initialized.");
		return W_FAILURE;
	}
	if ((NULL == p_drdy)) {
		log_text(0, LOG_LVL_WARN, "ADXRS649", "Invalid return ptr.");
		adxrs649_health.recent_null_params++;
		return W_INVALID_PARAM;
	}

	gpio_level_t ndrdy; // NOT-DRDY

	if (W_SUCCESS == gpio_read(GPIO_PIN_ADS1219_INT, &ndrdy, 0)) {
		// data ready is on the negative-edge
		*p_drdy = (GPIO_LEVEL_LOW == ndrdy) ? true : false;

	} else {
		// use I2C to get value
		if (ads1219_conversion_ready(&g_ads_handle, p_drdy) != W_SUCCESS) {
			adxrs649_health.recent_data_ready_check_fails++;
			return W_IO_ERROR;
		}
	}

	return W_SUCCESS;
}

/**
 * @brief get the spin rate data from the ADXRS649 Gyro
 * @param p_data is a pointer to where the data will be stored (deg/sec)
 * @param p_raw_data is a pointer to the raw data of the gyro
 * @return the status of the get data function
 * W_IO_ERROR - occours when fialed to read I2C or GPIO messages
 */
w_status_t adxrs649_get_gyro_data(float64_t *p_data, uint32_t *p_raw_data) {
	if (!is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXRS649", "Failed initialized.");
		return W_FAILURE;
	}

	if ((NULL == p_data) || (NULL == p_raw_data)) {
		log_text(0, LOG_LVL_WARN, "ADXRS649", "Invalid return ptrs.");
		adxrs649_health.recent_null_params++;
		return W_INVALID_PARAM;
	}

	if (ads1219_read_value(&g_ads_handle, p_raw_data) != W_SUCCESS) {
		adxrs649_health.recent_data_read_fails++;
		return W_IO_ERROR;
	}

	float64_t data_mv = 0;
	if (ads1219_millivolts(&g_ads_handle, (int32_t)*p_raw_data, &data_mv) != W_SUCCESS) {
		adxrs649_health.millivolt_conversion_fails++;
		return W_FAILURE;
	}

	*p_data = (data_mv - NULL_BIAS_mV) / MV_TO_DPS;

	return W_SUCCESS;
}

/**
 * @brief gets the health status of the ADXRS649 gyro and logs info
 * @return the health status of the ADXRS649
 */
health_status_t adxrs649_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_ADXRS649, .error_bitfield = 0};

	log_text(10,
			 LOG_LVL_INFO,
			 "ADXRS649",
			 "init=%d, st_fail=%u, adc_sanity_fails=%u, millivolt_conversion_fails=%u",
			 is_initialized,
			 adxrs649_health.self_test_fails,
			 adxrs649_health.adc_sanity_check_fails,
			 adxrs649_health.millivolt_conversion_fails);

	log_text(10,
			 LOG_LVL_INFO,
			 "ADXRS649",
			 "data_read_check_fails=%u, data_read_fails=%u, null_param=%u",
			 adxrs649_health.recent_data_ready_check_fails,
			 adxrs649_health.recent_data_read_fails,
			 adxrs649_health.recent_null_params);

	if (!is_initialized) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_NOT_INIT;
	}

	if (adxrs649_health.recent_null_params) {
		adxrs649_health.recent_null_params = 0;
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_INVALID_PARAM;
	}

	if (adxrs649_health.recent_data_read_fails || adxrs649_health.recent_data_ready_check_fails) {
		adxrs649_health.recent_data_ready_check_fails = 0;
		adxrs649_health.recent_data_read_fails = 0;
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_COMM_FAILURE;
	}

	return status;
}
