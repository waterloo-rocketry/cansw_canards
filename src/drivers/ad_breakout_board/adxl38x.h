#ifndef ADXL38X_H
#define ADXL38X_H

#include <stdbool.h>
#include <stdint.h>

#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADXL380_regmap.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"

/*******************************************************************************
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

typedef union {
	_adxl38x_sts_reg_flags_t flags;
	uint32_t value;
} adxl38x_sts_reg_flags_t;

typedef struct {
	i2c_bus_t i2c_bus;
	uint8_t i2c_addr;
	adxl38x_range_t range;
	adxl38x_op_mode_t op_mode;
} adxl38x_dev_t;

// HELPER
uint8_t adxl38x_field_prep_u8(uint8_t mask, uint8_t value);

// MAIN DRIVER
w_status_t adxl38x_read_device_data(adxl38x_dev_t *p_dev, uint8_t base_address, uint16_t size,
									uint8_t *p_read_data);

w_status_t adxl38x_write_device_data(adxl38x_dev_t *p_dev, uint8_t base_address, uint16_t size,
									 uint8_t *p_write_data);

w_status_t adxl38x_register_update_bits(adxl38x_dev_t *p_dev, uint8_t reg_addr, uint8_t mask,
										uint8_t update_val);

w_status_t adxl38x_init(adxl38x_dev_t *p_dev);

w_status_t adxl38x_soft_reset(adxl38x_dev_t *p_dev);

w_status_t adxl38x_set_op_mode(adxl38x_dev_t *p_dev, adxl38x_op_mode_t op_mode);

w_status_t adxl38x_get_op_mode(adxl38x_dev_t *p_dev, adxl38x_op_mode_t *p_op_mode);

w_status_t adxl38x_set_range(adxl38x_dev_t *p_dev, adxl38x_range_t range_val);

w_status_t adxl38x_get_range(adxl38x_dev_t *p_dev, adxl38x_range_t *p_range_val);

w_status_t adxl38x_set_self_test_registers(adxl38x_dev_t *p_dev, bool st_mode, bool st_force,
										   bool st_dir);

w_status_t adxl38x_clear_self_test_registers(adxl38x_dev_t *p_dev);

w_status_t adxl38x_get_sts_reg(adxl38x_dev_t *p_dev, adxl38x_sts_reg_flags_t *p_status_flags);

w_status_t adxl38x_get_raw_xyz(adxl38x_dev_t *p_dev, uint16_t *p_raw_x, uint16_t *p_raw_y,
							   uint16_t *p_raw_z);

w_status_t adxl38x_get_temp(adxl38x_dev_t *p_dev, float32_t *p_temp_degC);

w_status_t adxl38x_get_raw_data(adxl38x_dev_t *p_dev, adxl38x_ch_select_t channels,
								uint16_t *p_raw_x, uint16_t *p_raw_y, uint16_t *p_raw_z,
								uint16_t *p_raw_temp);

w_status_t adxl38x_get_xyz_gees(adxl38x_dev_t *p_dev, adxl38x_ch_select_t channels, float32_t *p_x,
								float32_t *p_y, float32_t *p_z);

w_status_t adxl38x_selftest(adxl38x_dev_t *p_dev, adxl38x_op_mode_t op_mode, bool *p_st_x,
							bool *p_st_y, bool *p_st_z);

w_status_t adxl38x_accel_set_FIFO(adxl38x_dev_t *p_dev, uint16_t num_samples, bool external_trigger,
								  adxl38x_fifo_mode_t fifo_mode, bool ch_ID_enable,
								  bool read_reset);

w_status_t adxl38x_data_raw_to_gees(adxl38x_dev_t *p_dev, uint8_t *p_raw_accel_data,
									float32_t *p_data_frac);

#endif // ADXL38X_H
