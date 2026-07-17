
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/adxl38x.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"

static const uint8_t ADXL_ADDRS = 0x1D;
static const uint8_t ADXL_FILTER_MASK = 0x18; // 0001100
static const uint8_t ADXL_INT0_MASK = 0x01; // 00000001

static const uint8_t ADXL_SOFT_RESET_DELAY = 1;
static const float32_t ADXL_16G_SCALE_FACTOR_MICRO_G_LSB = 533.3;
static const int32_t ADXL_MICRO_G_G = 1000000;

typedef struct {
	uint32_t self_test_fails;
	uint32_t self_test_incomplete;
	uint32_t recent_data_ready_check_fails;
	uint32_t recent_data_read_fails;
	uint32_t get_raw_accel_fails;
	uint32_t recent_null_params;
} adxl380_health_t;

static adxl38x_dev_t g_adx380_handle = {0};

static bool is_initialized = false;

static adxl380_health_t adxl380_health = {0};

/**
 * @brief this is initializes the ADXL380
 * @note Must be called after scheduler start
 * @return the status of the function call
 */
w_status_t adxl380_init() {
	// so that we don't reinitialize
	if (is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Reinitialization is not allowed.");
		return W_FAILURE;
	}

	is_initialized = true;
	// init the handle
	g_adx380_handle.i2c_addr = ADXL_ADDRS;
	g_adx380_handle.i2c_bus = I2C_BUS_2; // TODO: To be corrected

	if (W_SUCCESS != adxl38x_init(&g_adx380_handle)) {
		log_text(0, LOG_LVL_FATAL, "ADXL380", "NO-OS based driver failed to init.");
		return W_FAILURE;
	}

	w_status_t init_setting_status = W_SUCCESS;

	// re-zero all register to make sure no old setting will be carried over and prepare for
	// self-test
	if (W_SUCCESS != adxl38x_soft_reset(&g_adx380_handle)) {
		log_text(0, LOG_LVL_FATAL, "ADXL380", "Soft reset of registers failed.");
		return W_FAILURE;
	}
	vTaskDelay(pdMS_TO_TICKS(ADXL_SOFT_RESET_DELAY));

	// perform self-test
	bool st_failed = false;
	bool x_axis_status = false;
	bool y_axis_status = false;
	bool z_axis_status = false;

	if (W_SUCCESS !=
		adxl38x_selftest(
			&g_adx380_handle, ADXL38X_MODE_HP, &x_axis_status, &y_axis_status, &z_axis_status)) {
		log_text(0, LOG_LVL_FATAL, "ADXL380", "Self-test unable to be completed.");
		adxl380_health.self_test_incomplete++;
		return W_FAILURE;

	} else {
		if (!x_axis_status) {
			log_text(0, LOG_LVL_FATAL, "ADXL380", "x-axis self-test failed.");
			st_failed = true;
		}

		if (!y_axis_status) {
			log_text(0, LOG_LVL_FATAL, "ADXL380", "y-axis self-test failed.");
			st_failed = true;
		}

		if (!z_axis_status) {
			log_text(0, LOG_LVL_FATAL, "ADXL380", "z-axis self-test failed.");
			st_failed = true;
		}
	}

	if (st_failed) {
		adxl380_health.self_test_fails++;
		return W_FAILURE;
	}

	// re-zero all register to make sure self-test settings are not accidentally carried over
	init_setting_status |= adxl38x_soft_reset(&g_adx380_handle);
	vTaskDelay(pdMS_TO_TICKS(ADXL_SOFT_RESET_DELAY));

	init_setting_status |= adxl38x_set_op_mode(&g_adx380_handle, ADXL38X_MODE_HP);
	init_setting_status |= adxl38x_set_range(&g_adx380_handle, ADXL380_RANGE_16G);

	// FIFO is auto disabled

	/* FILTER: 00011000 -> 0x18
	DCF_BYPASS: 0
	EQ: 0
	LPF: 1/4 -> 01
	HPF_PATH: 1
	HPF: 000
	*/
	init_setting_status |=
		adxl38x_register_update_bits(&g_adx380_handle, ADXL38X_FILTER, ADXL_FILTER_MASK, 0x18);

	/* Set up INT0 for drdy */
	init_setting_status |=
		adxl38x_register_update_bits(&g_adx380_handle, ADXL38X_INT0_MAP0, ADXL_INT0_MASK, 0x01);

	/* DIGITAL ENABLE REGISTER
	MODE_CHANNEL_EN : x, y, z on
	*/
	init_setting_status |= adxl38x_register_update_bits(
		&g_adx380_handle,
		ADXL38X_DIG_EN,
		ADXL38X_MASK_CHEN_DIG_EN,
		adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_XYZ));

	if (W_SUCCESS != init_setting_status) {
		log_text(0, LOG_LVL_FATAL, "ADXL380", "Failed to set up the correct initial register bit.");

	} else {
		is_initialized = true;
	}

	return init_setting_status;
}

/**
 * @brief this gets the raw acceleration data from the ADXL380
 * @param p_raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_raw_accel(adxl380_raw_accel_data_t *p_raw_data) {
	// make sure have initialized
	if (!is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Not initialized.");
		return W_FAILURE;
	}

	uint8_t raw_data_array[6] = {0};

	if (W_SUCCESS !=
		adxl38x_read_device_data(&g_adx380_handle, ADXL38X_XDATA_H, 6, (uint8_t *)raw_data_array)) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Failed to read data from I2C.");
		adxl380_health.recent_data_read_fails++;
		return W_FAILURE;
	}

	p_raw_data->x = (uint16_t)((((uint16_t)raw_data_array[0]) << 8) | raw_data_array[1]);
	p_raw_data->y = (uint16_t)((((uint16_t)raw_data_array[2]) << 8) | raw_data_array[3]);
	p_raw_data->z = (uint16_t)((((uint16_t)raw_data_array[4]) << 8) | raw_data_array[5]);

	return W_SUCCESS;
}

/**
 * @brief gets the state of new data for the gyro
 * @param p_drdy a return pointer for if adxl380 is data ready
 * @return the status of getting data from gyro
 */
w_status_t adxl380_is_data_ready(bool *p_drdy) {
	if (!is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Failed initialized.");
		return W_FAILURE;
	}
	if (NULL == p_drdy) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Invalid return ptr.");
		adxl380_health.recent_null_params++;
		return W_INVALID_PARAM;
	}

	gpio_level_t drdy; // NOT-DRDY

	if (W_SUCCESS == gpio_read(GPIO_PIN_ADXL380_INT0, &drdy, 0)) {
		// data ready is on the positive-edge
		*p_drdy = (GPIO_LEVEL_HIGH == drdy) ? true : false;

	} else {
		uint8_t reg_drdy = 0;
		// use I2C to get value
		if (adxl38x_read_device_data(&g_adx380_handle, ADXL38X_STATUS3, 1, &reg_drdy) !=
			W_SUCCESS) {
			adxl380_health.recent_data_ready_check_fails++;
			return W_IO_ERROR;
		}

		*p_drdy = (1 == reg_drdy) ? true : false; // to make this clear
	}

	return W_SUCCESS;
}

/**
 * @brief this gets the acceleration data (raw and processed) from the ADXL380
 * @param p_data pointer to the vector form of the acceleration
 * @param p_raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_accel_data(vector3d_t *p_data, adxl380_raw_accel_data_t *p_raw_data) {
	// make sure have initialized
	if (!is_initialized) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Not initialized.");
		return W_FAILURE;
	}
	if ((NULL == p_data) || (NULL == p_raw_data)) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Invalid return ptr.");
		adxl380_health.recent_null_params++;
		return W_INVALID_PARAM;
	}

	if (W_SUCCESS != adxl380_get_raw_accel(p_raw_data)) {
		log_text(0, LOG_LVL_WARN, "ADXL380", "Failed to get raw acceleration.");
		adxl380_health.get_raw_accel_fails++;
		return W_FAILURE;
	}

	p_data->x =
		((float64_t)((int16_t)p_raw_data->x)) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;
	p_data->y =
		((float64_t)((int16_t)p_raw_data->y)) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;
	p_data->z =
		((float64_t)((int16_t)p_raw_data->z)) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;

	return W_SUCCESS;
}

/**
 * @brief gets the health status of the ADXL380 accelerometer and logs info
 * @return the health status of the ADXL380
 */
health_status_t adxl380_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_ADXL380, .error_bitfield = 0};

	log_text(10,
			 LOG_LVL_INFO,
			 "ADXL380",
			 "init=%u, st_fail=%u, st_incomplete=%u, data_ready_check_fails=%u",
			 is_initialized,
			 adxl380_health.self_test_fails,
			 adxl380_health.self_test_incomplete,
			 adxl380_health.recent_data_ready_check_fails);

	log_text(10,
			 LOG_LVL_INFO,
			 "ADXL380",
			 "read_fails=%u, get_accel_fails=%u, null_param=%u",
			 adxl380_health.recent_data_read_fails,
			 adxl380_health.get_raw_accel_fails,
			 adxl380_health.recent_null_params);

	if (!is_initialized) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_NOT_INIT;
	}

	if (adxl380_health.recent_null_params) {
		adxl380_health.recent_null_params = 0;
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_INVALID_PARAM;
	}

	if (adxl380_health.recent_data_read_fails || adxl380_health.recent_data_ready_check_fails) {
		adxl380_health.recent_data_ready_check_fails = 0;
		adxl380_health.recent_data_read_fails = 0;
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_COMM_FAILURE;
	}

	return status;
}
