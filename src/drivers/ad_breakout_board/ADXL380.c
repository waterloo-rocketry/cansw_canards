
#include <stdbool.h>
#include <stdint.h>

#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/adxl38x.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"
#include "task.h"

static const uint8_t ADXL_ADDRS = 0x1D;
static const uint8_t ADXL_PDM_AUDIO_MASK = 0x20;

static const uint8_t ADXL_SOFT_RESET_DELAY = 1;
static const float32_t ADXL_16G_SCALE_FACTOR_MICRO_G_LSB = 533.3;
static const int32_t ADXL_MICRO_G_G = 1000000;

adxl38x_dev_t g_adx380_handle = {};

/**
 * @brief this is initializes the ADXL380
 * @return the status of the function call
 */
w_status_t adxl380_init() {
	// init the handle
	g_adx380_handle.i2c_addr = ADXL_ADDRS;
	g_adx380_handle.i2c_bus = I2C_BUS_4; // TODO: To be corrected

	if (W_SUCCESS != adxl38x_init(&g_adx380_handle)) {
		log_text(0, "ADXL380", "ERROR: NO-OS based driver failed to init.");
		return W_FAILURE;
	}

	w_status_t init_setting_status = W_SUCCESS;

	// re-zero all register to make sure no old setting will be carried over
	init_setting_status |= adxl38x_soft_reset(&g_adx380_handle);
	vTaskDelay(pdMS_TO_TICKS(ADXL_SOFT_RESET_DELAY));

	init_setting_status |= adxl38x_set_op_mode(&g_adx380_handle, ADXL38X_MODE_HP);
	init_setting_status |= adxl38x_set_range(&g_adx380_handle, ADXL380_RANGE_16G);

	// FIFO is auto disabled

	/* OP_MODE
	PDM_MODE: on
	*/
	init_setting_status |=
		adxl38x_register_update_bits(&g_adx380_handle, ADXL38X_OP_MODE, ADXL_PDM_AUDIO_MASK, 0x20);

	/* DIGITAL ENABLE REGISTER
	MODE_CHANNEL_EN : x, y, z on
	*/
	init_setting_status |= adxl38x_register_update_bits(
		&g_adx380_handle,
		ADXL38X_DIG_EN,
		ADXL38X_MASK_CHEN_DIG_EN,
		adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_XYZ));

	if (W_SUCCESS != init_setting_status) {
		log_text(0, "ADXL380", "ERROR: Failed to set up the correct inital register bit.");
	}

	return init_setting_status;
}

/**
 * @brief this gets the raw acceleration data from the ADXL380
 * @param p_raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_raw_accel(adxl380_raw_accel_data_t *p_raw_data) {
	uint8_t raw_data_array[6] = {0};

	if (W_SUCCESS !=
		adxl38x_read_device_data(&g_adx380_handle, ADXL38X_XDATA_H, 6, (uint8_t *)raw_data_array)) {
		log_text(0, "ADXL380", "ERROR: Failed to read data from I2C.");
		return W_FAILURE;
	}

	p_raw_data->x = (uint16_t)((((uint16_t)raw_data_array[0]) << 8) | raw_data_array[1]);
	p_raw_data->y = (uint16_t)((((uint16_t)raw_data_array[2]) << 8) | raw_data_array[3]);
	p_raw_data->z = (uint16_t)((((uint16_t)raw_data_array[4]) << 8) | raw_data_array[5]);

	return W_SUCCESS;
}

/**
 * @brief this gets the acceleration data (raw and processed) from the ADXL380
 * @param data pointer to the vector form of the acceleration
 * @param p_raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_accel_data(vector3d_t *data, adxl380_raw_accel_data_t *p_raw_data) {
	if (W_SUCCESS != adxl380_get_raw_accel(p_raw_data)) {
		log_text(0, "ADXL380", "ERROR: Failed to get raw acceleration.");
		return W_FAILURE;
	}

	data->x =
		((float64_t)((int16_t)p_raw_data->x)) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;
	data->y =
		((float64_t)((int16_t)p_raw_data->y)) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;
	data->z =
		((float64_t)((int16_t)p_raw_data->z)) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;

	return W_SUCCESS;
}
