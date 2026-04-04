/**
 * @file ADXL380.c
 * @brief ADXL380-focused translation of the Analog Devices no-OS adxl38x driver
 *        to Waterloo Rocketry codebase primitives.
 *
 * Notes:
 * - This file is standalone and does not modify existing ADXL380 entry points.
 * - Communication is translated to the project I2C API (drivers/i2c/i2c.h).
 */

/********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "common/math/math.h"
#include "drivers/ad_breakout_board/adxl38x.h"
#include "stm32h7xx_hal.h"

#define ADXL38X_MAX_XFER_LEN 64
#define ADXL38X_NEG_ACC_MSK 0xFFFF0000

#define ADXL38X_RESET_DELAY_MS 1
#define ADXL38X_OP_MODE_SETTLE_DELAY_MS 2
#define ADXL38X_SELFTEST_SAMPLE_DELAY_MS 10
#define ADXL38X_SELFTEST_SAMPLE_COUNT 25

#define ADXL38X_ONE_BYTE 1
#define ADXL38X_TWO_BYTES 2
#define ADXL38X_FOUR_BYTES 4
#define ADXL38X_SIX_BYTES 6

#define ADXL38X_MAX_FIFO_SAMPLES 320
#define ADXL38X_MAX_FIFO_SAMPLES_XYZ_OR_YZT 318

#define ADXL38X_DEV_ID_AD_EXPECTED ADXL38X_RESET_DEVID_AD
#define ADXL38X_DEV_ID_MST_EXPECTED ADXL38X_RESET_DEVID_MST
#define ADXL38X_PART_ID_EXPECTED ADXL38X_RESET_PART_ID

#ifndef BIT
#define BIT(x) ((uint32_t)1u << (x))
#endif

static const uint8_t ADX38X_SCALE_MUL[3] = {1, 2, 4};

static int64_t adxl38x_accel_conv(adxl38x_dev_t *p_dev, uint16_t raw_accel);
static w_status_t adxl38x_set_to_standby(adxl38x_dev_t *p_dev);

static uint8_t adxl38x_mask_shift_u8(uint8_t mask) {
	uint8_t shift = 0;
	while (((mask >> shift) & 0x1) == 0 && shift < 8) {
		shift++;
	}
	return shift;
}

uint8_t adxl38x_field_prep_u8(uint8_t mask, uint8_t value) {
	return (uint8_t)((value << adxl38x_mask_shift_u8(mask)) & mask);
}

static uint8_t adxl38x_field_get_u8(uint8_t mask, uint8_t value) {
	return (uint8_t)((value & mask) >> adxl38x_mask_shift_u8(mask));
}

static uint16_t adxl38x_get_unaligned_be16(const uint8_t *p_data) {
	return (uint16_t)(((uint16_t)p_data[0] << 8) | p_data[1]);
}

static uint32_t adxl38x_get_unaligned_be32(const uint8_t *p_data) {
	return ((uint32_t)p_data[0] << 24) | ((uint32_t)p_data[1] << 16) | ((uint32_t)p_data[2] << 8) |
		   (uint32_t)p_data[3];
}

static int32_t adxl38x_sign_extend16(uint16_t value) {
	return (int32_t)(int16_t)value;
}

/**
 * @brief Reads from the device over project I2C API.
 */
w_status_t adxl38x_read_device_data(adxl38x_dev_t *p_dev, uint8_t base_address, uint16_t size,
									uint8_t *p_read_data) {
	if ((p_dev == NULL) || (p_read_data == NULL) || (size == 0) || (size > ADXL38X_MAX_XFER_LEN)) {
		return W_INVALID_PARAM;
	}

	return i2c_read_reg(p_dev->i2c_bus, p_dev->i2c_addr, base_address, p_read_data, (uint8_t)size);
}

/**
 * @brief Writes to the device over project I2C API.
 */
w_status_t adxl38x_write_device_data(adxl38x_dev_t *p_dev, uint8_t base_address, uint16_t size,
									 uint8_t *p_write_data) {
	if ((p_dev == NULL) || (p_write_data == NULL) || (size == 0) || (size > ADXL38X_MAX_XFER_LEN)) {
		return W_INVALID_PARAM;
	}

	return i2c_write_reg(
		p_dev->i2c_bus, p_dev->i2c_addr, base_address, p_write_data, (uint8_t)size);
}

/**
 * @brief Updates masked bits in a register.
 */
w_status_t adxl38x_register_update_bits(adxl38x_dev_t *p_dev, uint8_t reg_addr, uint8_t mask,
										uint8_t update_val) {
	w_status_t ret;
	uint8_t data;

	ret = adxl38x_read_device_data(p_dev, reg_addr, ADXL38X_ONE_BYTE, &data);
	if (ret != W_SUCCESS) {
		return ret;
	}

	if (reg_addr == ADXL38X_OP_MODE) {
		ret = adxl38x_set_to_standby(p_dev);
		if (ret != W_SUCCESS) {
			return ret;
		}
	}

	data &= (uint8_t)(~mask);
	data |= (uint8_t)(update_val & mask);

	return adxl38x_write_device_data(p_dev, reg_addr, ADXL38X_ONE_BYTE, &data);
}

/**
 * @brief Initializes ADXL380 device context and verifies IDs.
 */
w_status_t adxl38x_init(adxl38x_dev_t *p_dev) {
	w_status_t ret;
	uint8_t reg_value;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_DEVID_AD, ADXL38X_ONE_BYTE, &reg_value);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (reg_value != ADXL38X_DEV_ID_AD_EXPECTED) {
		return W_FAILURE;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_DEVID_MST, ADXL38X_ONE_BYTE, &reg_value);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (reg_value != ADXL38X_DEV_ID_MST_EXPECTED) {
		return W_FAILURE;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_PART_ID, ADXL38X_ONE_BYTE, &reg_value);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (reg_value != ADXL38X_PART_ID_EXPECTED) {
		return W_FAILURE;
	}

	p_dev->range = ADXL380_RANGE_4G;
	p_dev->op_mode = ADXL38X_MODE_STDBY;
	return W_SUCCESS;
}

/**
 * @brief Performs software reset.
 */
w_status_t adxl38x_soft_reset(adxl38x_dev_t *p_dev) {
	w_status_t ret;
	uint8_t reg_value;
	uint8_t data = ADXL38X_RESET_CODE;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_write_device_data(p_dev, ADXL38X_REG_RESET, ADXL38X_ONE_BYTE, &data);
	if (ret != W_SUCCESS) {
		return ret;
	}

	/* no_os_udelay(500) translation */
	HAL_Delay(ADXL38X_RESET_DELAY_MS);

	ret = adxl38x_read_device_data(p_dev, ADXL38X_DEVID_AD, ADXL38X_ONE_BYTE, &reg_value);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (reg_value != ADXL38X_DEV_ID_AD_EXPECTED) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Sets operation mode.
 */
w_status_t adxl38x_set_op_mode(adxl38x_dev_t *p_dev, adxl38x_op_mode_t op_mode) {
	w_status_t ret;

	ret =
		adxl38x_register_update_bits(p_dev,
									 ADXL38X_OP_MODE,
									 ADXL38X_MASK_OP_MODE,
									 adxl38x_field_prep_u8(ADXL38X_MASK_OP_MODE, (uint8_t)op_mode));
	if (ret == W_SUCCESS) {
		p_dev->op_mode = op_mode;
	}

	/* no_os_mdelay(2) translation */
	HAL_Delay(ADXL38X_OP_MODE_SETTLE_DELAY_MS);
	return ret;
}

/**
 * @brief Gets current operation mode.
 */
w_status_t adxl38x_get_op_mode(adxl38x_dev_t *p_dev, adxl38x_op_mode_t *p_op_mode) {
	w_status_t ret;
	uint8_t op_mode_reg_val;

	if ((p_dev == NULL) || (p_op_mode == NULL)) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_OP_MODE, ADXL38X_ONE_BYTE, &op_mode_reg_val);
	if (ret != W_SUCCESS) {
		return ret;
	}

	*p_op_mode = (adxl38x_op_mode_t)adxl38x_field_get_u8(ADXL38X_MASK_OP_MODE, op_mode_reg_val);
	p_dev->op_mode = *p_op_mode;
	return W_SUCCESS;
}

/**
 * @brief Sets measurement range.
 */
w_status_t adxl38x_set_range(adxl38x_dev_t *p_dev, adxl38x_range_t range_val) {
	w_status_t ret;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	ret =
		adxl38x_register_update_bits(p_dev,
									 ADXL38X_OP_MODE,
									 ADXL38X_MASK_RANGE,
									 adxl38x_field_prep_u8(ADXL38X_MASK_RANGE, (uint8_t)range_val));
	if (ret == W_SUCCESS) {
		p_dev->range = range_val;
	}

	return ret;
}

/**
 * @brief Gets current measurement range.
 */
w_status_t adxl38x_get_range(adxl38x_dev_t *p_dev, adxl38x_range_t *p_range_val) {
	w_status_t ret;
	uint8_t range_reg_val;

	if ((p_dev == NULL) || (p_range_val == NULL)) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_OP_MODE, ADXL38X_ONE_BYTE, &range_reg_val);
	if (ret != W_SUCCESS) {
		return ret;
	}

	*p_range_val = (adxl38x_range_t)adxl38x_field_get_u8(ADXL38X_MASK_RANGE, range_reg_val);
	p_dev->range = *p_range_val;
	return W_SUCCESS;
}

/**
 * @brief Sets self-test control bits.
 */
w_status_t adxl38x_set_self_test_registers(adxl38x_dev_t *p_dev, bool st_mode, bool st_force,
										   bool st_dir) {
	w_status_t ret;
	uint8_t st_fields_value = 0;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	if (st_mode) {
		st_fields_value |= BIT(7);
	}
	if (st_force) {
		st_fields_value |= BIT(6);
	}
	if (st_dir) {
		st_fields_value |= BIT(5);
	}

	ret = adxl38x_register_update_bits(
		p_dev, ADXL38X_SNSR_AXIS_EN, ADXL38X_SLF_TST_CTRL_MSK, st_fields_value);

	return ret;
}

/**
 * @brief Clears self-test control bits.
 */
w_status_t adxl38x_clear_self_test_registers(adxl38x_dev_t *p_dev) {
	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	return adxl38x_register_update_bits(
		p_dev, ADXL38X_SNSR_AXIS_EN, ADXL38X_SLF_TST_CTRL_MSK, ADXL38X_RESET_ZERO);
}

/**
 * @brief Reads status0..status3 into a 32-bit packed value.
 */
w_status_t adxl38x_get_sts_reg(adxl38x_dev_t *p_dev, adxl38x_sts_reg_flags_t *p_status_flags) {
	w_status_t ret;
	uint8_t status_value[4];

	if ((p_dev == NULL) || (p_status_flags == NULL)) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_STATUS0, ADXL38X_FOUR_BYTES, status_value);
	if (ret != W_SUCCESS) {
		return ret;
	}

	p_status_flags->value = adxl38x_get_unaligned_be32(status_value);
	return W_SUCCESS;
}

/**
 * @brief Reads raw X/Y/Z outputs.
 */
w_status_t adxl38x_get_raw_xyz(adxl38x_dev_t *p_dev, uint16_t *p_raw_x, uint16_t *p_raw_y,
							   uint16_t *p_raw_z) {
	w_status_t ret;
	uint8_t array_raw_data[6] = {0};
	adxl38x_op_mode_t op_mode;

	if ((p_dev == NULL) || (p_raw_x == NULL) || (p_raw_y == NULL) || (p_raw_z == NULL)) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_register_update_bits(
		p_dev,
		ADXL38X_DIG_EN,
		ADXL38X_MASK_CHEN_DIG_EN,
		adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, (uint8_t)ADXL38X_CH_EN_XYZ));
	if (ret != W_SUCCESS) {
		return ret;
	}

	ret = adxl38x_get_op_mode(p_dev, &op_mode);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (op_mode == ADXL38X_MODE_STDBY) {
		ret = adxl38x_set_op_mode(p_dev, ADXL38X_MODE_HP);
		if (ret != W_SUCCESS) {
			return ret;
		}
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_XDATA_H, ADXL38X_SIX_BYTES, array_raw_data);
	if (ret != W_SUCCESS) {
		return ret;
	}

	*p_raw_x = adxl38x_get_unaligned_be16(array_raw_data);
	*p_raw_y = adxl38x_get_unaligned_be16(array_raw_data + 2);
	*p_raw_z = adxl38x_get_unaligned_be16(array_raw_data + 4);
	return W_SUCCESS;
}

/**
 * @brief Reads and converts temperature output.
 */
w_status_t adxl38x_get_temp(adxl38x_dev_t *p_dev, float32_t *p_temp_degC) {
	w_status_t ret;
	adxl38x_op_mode_t op_mode;
	uint8_t array_raw_data[2] = {0};
	int32_t temp_raw_lsb;

	if ((p_dev == NULL) || (p_temp_degC == NULL)) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_register_update_bits(
		p_dev,
		ADXL38X_DIG_EN,
		ADXL38X_MASK_CHEN_DIG_EN,
		adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, (uint8_t)ADXL38X_CH_EN_T));
	if (ret != W_SUCCESS) {
		return ret;
	}

	ret = adxl38x_get_op_mode(p_dev, &op_mode);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (op_mode == ADXL38X_MODE_STDBY) {
		ret = adxl38x_set_op_mode(p_dev, ADXL38X_MODE_HP);
		if (ret != W_SUCCESS) {
			return ret;
		}
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_TDATA_H, ADXL38X_TWO_BYTES, array_raw_data);
	if (ret != W_SUCCESS) {
		return ret;
	}

	temp_raw_lsb = (int32_t)(adxl38x_get_unaligned_be16(array_raw_data) >> 4);
	temp_raw_lsb -= ADXL38X_TEMP_OFFSET;
	*p_temp_degC = ((float32_t)temp_raw_lsb * (float32_t)ADXL38X_TEMP_SCALE_DEN) /
				   (float32_t)ADXL38X_TEMP_SCALE_NUM;
	return W_SUCCESS;
}

/**
 * @brief Reads selected raw channels (TZYX mapping).
 */
w_status_t adxl38x_get_raw_data(adxl38x_dev_t *p_dev, adxl38x_ch_select_t channels,
								uint16_t *p_raw_x, uint16_t *p_raw_y, uint16_t *p_raw_z,
								uint16_t *p_raw_temp) {
	uint8_t array_raw_data[8] = {0};
	uint8_t array_rearranged_data[8] = {0};
	w_status_t ret;
	int j = 0;
	adxl38x_op_mode_t op_mode;
	uint8_t tzyx;
	uint8_t start_addr;
	uint16_t num_bytes = 0;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_register_update_bits(
		p_dev,
		ADXL38X_DIG_EN,
		ADXL38X_MASK_CHEN_DIG_EN,
		adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, (uint8_t)channels));
	if (ret != W_SUCCESS) {
		return ret;
	}

	ret = adxl38x_get_op_mode(p_dev, &op_mode);
	if (ret != W_SUCCESS) {
		return ret;
	}
	if (op_mode == ADXL38X_MODE_STDBY) {
		ret = adxl38x_set_op_mode(p_dev, ADXL38X_MODE_HP);
		if (ret != W_SUCCESS) {
			return ret;
		}
	}

	tzyx = (uint8_t)channels;
	if ((tzyx & 0x01) != 0) {
		start_addr = ADXL38X_XDATA_H;
	} else if ((tzyx & 0x02) != 0) {
		start_addr = ADXL38X_YDATA_H;
	} else if ((tzyx & 0x04) != 0) {
		start_addr = ADXL38X_ZDATA_H;
	} else {
		start_addr = ADXL38X_TDATA_H;
	}

	while (tzyx != 0) {
		num_bytes += (uint16_t)(tzyx & 0x01);
		tzyx >>= 1;
	}
	num_bytes = (uint16_t)(num_bytes * 2);

	ret = adxl38x_read_device_data(p_dev, start_addr, num_bytes, array_raw_data);
	if (ret != W_SUCCESS) {
		return ret;
	}

	tzyx = (uint8_t)channels;
	for (uint8_t i = 0; i < 8; i += 2) {
		if ((tzyx & 0x01) != 0) {
			array_rearranged_data[i] = array_raw_data[j];
			array_rearranged_data[i + 1] = array_raw_data[j + 1];
			j += 2;
		} else {
			array_rearranged_data[i] = 0;
			array_rearranged_data[i + 1] = 0;
		}
		tzyx >>= 1;
	}

	if (p_raw_x != NULL) {
		*p_raw_x = adxl38x_get_unaligned_be16(array_rearranged_data);
	}
	if (p_raw_y != NULL) {
		*p_raw_y = adxl38x_get_unaligned_be16(array_rearranged_data + 2);
	}
	if (p_raw_z != NULL) {
		*p_raw_z = adxl38x_get_unaligned_be16(array_rearranged_data + 4);
	}
	if (p_raw_temp != NULL) {
		*p_raw_temp = (uint16_t)(adxl38x_get_unaligned_be16(array_rearranged_data + 6) >> 4);
	}

	return W_SUCCESS;
}

/**
 * @brief Reads selected acceleration channels and converts to g.
 */
w_status_t adxl38x_get_xyz_gees(adxl38x_dev_t *p_dev, adxl38x_ch_select_t channels, float32_t *p_x,
								float32_t *p_y, float32_t *p_z) {
	w_status_t ret;
	uint16_t raw_accel_x;
	uint16_t raw_accel_y;
	uint16_t raw_accel_z;

	if ((p_dev == NULL) || (p_x == NULL) || (p_y == NULL) || (p_z == NULL)) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_get_raw_data(p_dev, channels, &raw_accel_x, &raw_accel_y, &raw_accel_z, NULL);
	if (ret != W_SUCCESS) {
		return ret;
	}

	*p_x = (float32_t)adxl38x_accel_conv(p_dev, raw_accel_x) /
		   (float32_t)ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;
	*p_y = (float32_t)adxl38x_accel_conv(p_dev, raw_accel_y) /
		   (float32_t)ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;
	*p_z = (float32_t)adxl38x_accel_conv(p_dev, raw_accel_z) /
		   (float32_t)ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;

	return W_SUCCESS;
}

/**
 * @brief Converts raw acceleration code to scaled g numerator.
 */
static int64_t adxl38x_accel_conv(adxl38x_dev_t *p_dev, uint16_t raw_accel) {
	int32_t accel_data;

	if (p_dev == NULL) {
		return 0;
	}

	if ((raw_accel & BIT(15)) != 0) {
		accel_data = (int32_t)(raw_accel | ADXL38X_NEG_ACC_MSK);
	} else {
		accel_data = (int32_t)raw_accel;
	}

	return (int64_t)accel_data * (int64_t)ADXL380_ACC_SCALE_FACTOR_GEE_MUL *
		   (int64_t)ADX38X_SCALE_MUL[p_dev->range];
}

/**
 * @brief Forces standby before OP_MODE field edits.
 */
static w_status_t adxl38x_set_to_standby(adxl38x_dev_t *p_dev) {
	w_status_t ret;
	uint8_t op_mode_reg_val;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_OP_MODE, ADXL38X_ONE_BYTE, &op_mode_reg_val);
	if (ret != W_SUCCESS) {
		return ret;
	}

	op_mode_reg_val = (uint8_t)(op_mode_reg_val & (uint8_t)(~ADXL38X_MASK_OP_MODE));
	ret = adxl38x_write_device_data(p_dev, ADXL38X_OP_MODE, ADXL38X_ONE_BYTE, &op_mode_reg_val);
	if (ret == W_SUCCESS) {
		p_dev->op_mode =
			(adxl38x_op_mode_t)adxl38x_field_get_u8(ADXL38X_MASK_OP_MODE, op_mode_reg_val);
	}

	return ret;
}

/**
 * @brief Executes self-test and returns per-axis pass/fail.
 */
w_status_t adxl38x_selftest(adxl38x_dev_t *p_dev, adxl38x_op_mode_t op_mode, bool *p_st_x,
							bool *p_st_y, bool *p_st_z) {
	w_status_t ret;
	uint8_t array_raw_data[6] = {0};
	int32_t low_limit_xy;
	int32_t low_limit_z;
	int32_t up_limit_xy;
	int32_t up_limit_z;
	int32_t st_delta_data[3] = {0};
	int32_t st_positive_data[3] = {0};
	int32_t st_negative_data[3] = {0};
	int64_t x_sum = 0;
	int64_t y_sum = 0;
	int64_t z_sum = 0;

	if ((p_dev == NULL) || (p_st_x == NULL) || (p_st_y == NULL) || (p_st_z == NULL)) {
		return W_INVALID_PARAM;
	}

	*p_st_x = false;
	*p_st_y = false;
	*p_st_z = false;

	ret = adxl38x_set_op_mode(p_dev, ADXL38X_MODE_STDBY);
	if (ret != W_SUCCESS) {
		return ret;
	}

	if (op_mode == ADXL38X_MODE_HRT_SND) {
		ret = adxl38x_register_update_bits(
			p_dev,
			ADXL38X_DIG_EN,
			ADXL38X_MASK_CHEN_DIG_EN,
			adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, (uint8_t)ADXL38X_CH_EN_X));
	} else {
		ret = adxl38x_register_update_bits(
			p_dev,
			ADXL38X_DIG_EN,
			ADXL38X_MASK_CHEN_DIG_EN,
			adxl38x_field_prep_u8(ADXL38X_MASK_CHEN_DIG_EN, (uint8_t)ADXL38X_CH_EN_XYZ));
	}
	if (ret != W_SUCCESS) {
		return ret;
	}

	ret = adxl38x_set_op_mode(p_dev, op_mode);
	if (ret != W_SUCCESS) {
		return ret;
	}

	ret = adxl38x_set_self_test_registers(p_dev, true, true, false);
	if (ret != W_SUCCESS) {
		return ret;
	}

	for (uint8_t k = 0; k < ADXL38X_SELFTEST_SAMPLE_COUNT; k++) {
		HAL_Delay(ADXL38X_SELFTEST_SAMPLE_DELAY_MS);
		ret = adxl38x_read_device_data(p_dev, ADXL38X_XDATA_H, ADXL38X_SIX_BYTES, array_raw_data);
		if (ret != W_SUCCESS) {
			return ret;
		}

		x_sum += (int64_t)adxl38x_get_unaligned_be16(array_raw_data);
		y_sum += (int64_t)adxl38x_get_unaligned_be16(array_raw_data + 2);
		z_sum += (int64_t)adxl38x_get_unaligned_be16(array_raw_data + 4);
	}
	st_positive_data[0] = adxl38x_sign_extend16((uint16_t)(x_sum / ADXL38X_SELFTEST_SAMPLE_COUNT));
	st_positive_data[1] = adxl38x_sign_extend16((uint16_t)(y_sum / ADXL38X_SELFTEST_SAMPLE_COUNT));
	st_positive_data[2] = adxl38x_sign_extend16((uint16_t)(z_sum / ADXL38X_SELFTEST_SAMPLE_COUNT));

	ret = adxl38x_set_self_test_registers(p_dev, true, true, true);
	if (ret != W_SUCCESS) {
		return ret;
	}

	x_sum = 0;
	y_sum = 0;
	z_sum = 0;
	for (uint8_t k = 0; k < ADXL38X_SELFTEST_SAMPLE_COUNT; k++) {
		HAL_Delay(ADXL38X_SELFTEST_SAMPLE_DELAY_MS);
		ret = adxl38x_read_device_data(p_dev, ADXL38X_XDATA_H, ADXL38X_SIX_BYTES, array_raw_data);
		if (ret != W_SUCCESS) {
			return ret;
		}

		x_sum += (int64_t)adxl38x_get_unaligned_be16(array_raw_data);
		y_sum += (int64_t)adxl38x_get_unaligned_be16(array_raw_data + 2);
		z_sum += (int64_t)adxl38x_get_unaligned_be16(array_raw_data + 4);
	}
	st_negative_data[0] = adxl38x_sign_extend16((uint16_t)(x_sum / ADXL38X_SELFTEST_SAMPLE_COUNT));
	st_negative_data[1] = adxl38x_sign_extend16((uint16_t)(y_sum / ADXL38X_SELFTEST_SAMPLE_COUNT));
	st_negative_data[2] = adxl38x_sign_extend16((uint16_t)(z_sum / ADXL38X_SELFTEST_SAMPLE_COUNT));

	for (uint8_t k = 0; k < 3; k++) {
		st_delta_data[k] = abs(st_positive_data[k] - st_negative_data[k]);
		st_delta_data[k] = st_delta_data[k] * ADXL38X_ST_LIMIT_DENOMINATOR;
	}

	low_limit_xy = (ADXL380_XY_ST_LIMIT_MIN * ADXL380_ACC_SENSITIVITY) >> (uint8_t)(p_dev->range);
	up_limit_xy = (ADXL380_XY_ST_LIMIT_MAX * ADXL380_ACC_SENSITIVITY) >> (uint8_t)(p_dev->range);
	low_limit_z = (ADXL380_Z_ST_LIMIT_MIN * ADXL380_ACC_SENSITIVITY) >> (uint8_t)(p_dev->range);
	up_limit_z = (ADXL380_Z_ST_LIMIT_MAX * ADXL380_ACC_SENSITIVITY) >> (uint8_t)(p_dev->range);

	if ((st_delta_data[0] >= low_limit_xy) && (st_delta_data[0] <= up_limit_xy)) {
		*p_st_x = true;
	}
	if ((st_delta_data[1] >= low_limit_xy) && (st_delta_data[1] <= up_limit_xy)) {
		*p_st_y = true;
	}
	if ((st_delta_data[2] >= low_limit_z) && (st_delta_data[2] <= up_limit_z)) {
		*p_st_z = true;
	}

	ret = adxl38x_set_self_test_registers(p_dev, false, false, false);
	if (ret != W_SUCCESS) {
		return ret;
	}

	return W_SUCCESS;
}

/**
 * @brief Configures FIFO.
 */
w_status_t adxl38x_accel_set_FIFO(adxl38x_dev_t *p_dev, uint16_t num_samples, bool external_trigger,
								  adxl38x_fifo_mode_t fifo_mode, bool ch_ID_enable,
								  bool read_reset) {
	w_status_t ret;
	uint8_t write_data = 0;
	uint8_t fifo_samples_low;
	uint8_t fifo_samples_high;
	uint8_t set_channels;

	if (p_dev == NULL) {
		return W_INVALID_PARAM;
	}

	ret = adxl38x_read_device_data(p_dev, ADXL38X_DIG_EN, ADXL38X_ONE_BYTE, &set_channels);
	if (ret != W_SUCCESS) {
		return ret;
	}
	set_channels = adxl38x_field_get_u8(ADXL38X_MASK_CHEN_DIG_EN, set_channels);

	if (num_samples > ADXL38X_MAX_FIFO_SAMPLES) {
		return W_INVALID_PARAM;
	}
	if ((num_samples > ADXL38X_MAX_FIFO_SAMPLES_XYZ_OR_YZT) &&
		((set_channels == 0) || (set_channels == (uint8_t)ADXL38X_CH_EN_XYZ) ||
		 (set_channels == (uint8_t)ADXL38X_CH_EN_YZT))) {
		return W_INVALID_PARAM;
	}

	fifo_samples_low = (uint8_t)(num_samples & 0xFF);
	ret = adxl38x_write_device_data(p_dev, ADXL38X_FIFO_CFG1, ADXL38X_ONE_BYTE, &fifo_samples_low);
	if (ret != W_SUCCESS) {
		return ret;
	}

	fifo_samples_high = (uint8_t)(num_samples >> 8);
	fifo_samples_high &= BIT(0);
	write_data = fifo_samples_high;

	write_data |= adxl38x_field_prep_u8(ADXL38X_FIFOCFG_FIFOMODE_MSK, (uint8_t)fifo_mode);

	if (read_reset) {
		write_data |= BIT(7);
	}
	if (ch_ID_enable) {
		write_data |= BIT(6);
	}
	if (external_trigger && (fifo_mode == ADXL38X_FIFO_TRIGGER)) {
		write_data |= BIT(3);
	}

	return adxl38x_write_device_data(p_dev, ADXL38X_FIFO_CFG0, ADXL38X_ONE_BYTE, &write_data);
}

/**
 * @brief Converts raw 2-byte accel sample to fractional g.
 */
w_status_t adxl38x_data_raw_to_gees(adxl38x_dev_t *p_dev, uint8_t *p_raw_accel_data,
									float32_t *p_data_frac) {
	uint16_t data;

	if ((p_dev == NULL) || (p_raw_accel_data == NULL) || (p_data_frac == NULL)) {
		return W_INVALID_PARAM;
	}

	data = adxl38x_get_unaligned_be16(p_raw_accel_data);
	*p_data_frac =
		(float32_t)adxl38x_accel_conv(p_dev, data) / (float32_t)ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;

	return W_SUCCESS;
}
