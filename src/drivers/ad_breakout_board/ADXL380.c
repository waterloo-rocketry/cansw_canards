#include "rocketlib/include/common.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "drivers/ad_breakout_board/adxl38x.h"
#include "drivers/i2c/i2c.h"
#include <stdbool.h>
#include <stdint.h>

static uint8_t ADXL_ADDRS = 0x00;
static uint8_t ADXL_PDM_AUDIO_MASK = 0x20;

static uint8_t ADXL_SOFT_RESET_DELAY = 1;
static float ADXL_16G_SCALE_FACTOR_MICRO_G_LSB = 533.3;
static int32_t ADXL_MICRO_G_G = 1000000;


adxl38x_dev_t adx380_handle = {};

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	return i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
}

/**
 * @brief this is initializes the ADXL380
 * @return the status of the function call
 */
w_status_t adxl380_init(){

	// init the handle
	adx380_handle.i2c_addr = ADXL_ADDRS;
	adx380_handle.i2c_bus = I2C_BUS_4;


	if (W_SUCCESS != adxl38x_init(&adx380_handle)) {
		// TODO: log error
		return W_FAILURE;
	}

	w_status_t init_setting_status = W_SUCCESS;

	/*
	Confirmed
	Filters (All HP filters) (to be clarified)
	*/
	init_setting_status |= adxl38x_soft_reset(&adx380_handle);
	vTaskDelay(pdMS_TO_TICKS(ADXL_SOFT_RESET_DELAY));

	init_setting_status |= adxl38x_set_op_mode(&adx380_handle, ADXL38X_MODE_HP);
	init_setting_status |= adxl38x_set_range(&adx380_handle, ADXL380_RANGE_16G);

	// FIFO is auto disabled

	/* OP_MODE
	PDM_MODE: on	
	*/
	init_setting_status |= adxl38x_register_update_bits(&adx380_handle, ADXL38X_OP_MODE, ADXL_PDM_AUDIO_MASK, 0x20);

	/* DIGITAL ENABLE REGISTER
	MODE_CHANNEL_EN : x, y, z on
	*/
	init_setting_status |= adxl38x_register_update_bits(&adx380_handle, ADXL38X_DIG_EN, ADXL38X_MASK_CHEN_DIG_EN, adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_XYZ));

	if (W_SUCCESS != init_setting_status) {
		// TODO: Error logging
	}

	return init_setting_status;
}


/**
 * @brief this gets the raw acceleration data from the ADXL380
 * @param raw_x pointer to the raw x value
 * @param raw_y pointer to the raw y value
 * @param raw_z pointer to the raw z value
 * @return the status of the function call
 */
w_status_t adxl380_get_raw_accel(altimu_raw_imu_data_t *raw_data){

	uint8_t* raw_data_array;
    
	if (W_SUCCESS != adxl38x_read_device_data(&adx380_handle, ADXL38X_XDATA_H, 6, raw_data_array)) {
		//TODO: Error logging
		return W_FAILURE;
	}

	raw_data->x = (uint16_t) ((((uint16_t) raw_data_array[0]) << 8) | raw_data_array[1]);
	raw_data->y = (uint16_t) ((((uint16_t) raw_data_array[2]) << 8) | raw_data_array[3]);
	raw_data->z = (uint16_t) ((((uint16_t) raw_data_array[4]) << 8) | raw_data_array[5]);

	return W_SUCCESS;
}

/**
 * @brief Converts raw acceleration code to scaled g numerator.
 */
static int64_t adxl380_accel_int64_conv(uint16_t raw_accel) {
	int64_t accel_data;

	accel_data = (int64_t) ((((uint64_t) raw_accel) ^ 0x8000) - 0x8000);

	return accel_data;
}

/**
 * @brief this gets the acceleration data (raw and processed) from the ADXL380
 * @param data pointer to the vector form of the acceleration
 * @param raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_accel_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data){

	if (W_SUCCESS != adxl380_get_raw_accel(raw_data)) {
		// TODO: Error logging
		return W_FAILURE;
	}

	data->x = adxl380_accel_int64_conv(raw_data->x) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;
	data->y = adxl380_accel_int64_conv(raw_data->y) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;
	data->z = adxl380_accel_int64_conv(raw_data->z) * ADXL_16G_SCALE_FACTOR_MICRO_G_LSB / ADXL_MICRO_G_G;

	return W_SUCCESS;
}
