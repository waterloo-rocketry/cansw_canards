#ifndef ADXL38X_H
#define ADXL38X_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/ad_breakout_board/ADXL380_regmap.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"

typedef union adxl38x_sts_reg_flags {
	struct _adxl38x_sts_reg_flags flags;
	uint32_t value;
} adxl38x_sts_reg_flags_t;

typedef struct adxl38x_dev {
	i2c_bus_t i2c_bus;
	uint8_t i2c_addr;
	enum adxl38x_range range;
	enum adxl38x_op_mode op_mode;
} adxl38x_dev_t;

// HELPER
uint8_t adxl38x_field_prep_u8(uint8_t mask, uint8_t value);

// MAIN DRIVER
w_status_t adxl38x_read_device_data(adxl38x_dev_t *dev, uint8_t base_address, uint16_t size,
									uint8_t *read_data);

w_status_t adxl38x_write_device_data(adxl38x_dev_t *dev, uint8_t base_address, uint16_t size,
									 uint8_t *write_data);

w_status_t adxl38x_register_update_bits(adxl38x_dev_t *dev, uint8_t reg_addr, uint8_t mask,
										uint8_t update_val);

w_status_t adxl38x_init(adxl38x_dev_t *dev);

w_status_t adxl38x_soft_reset(adxl38x_dev_t *dev);

w_status_t adxl38x_set_op_mode(adxl38x_dev_t *dev, enum adxl38x_op_mode op_mode);

w_status_t adxl38x_get_op_mode(adxl38x_dev_t *dev, enum adxl38x_op_mode *op_mode);

w_status_t adxl38x_set_range(adxl38x_dev_t *dev, enum adxl38x_range range_val);

w_status_t adxl38x_get_range(adxl38x_dev_t *dev, enum adxl38x_range *range_val);

w_status_t adxl38x_set_self_test_registers(adxl38x_dev_t *dev, bool st_mode, bool st_force,
										   bool st_dir);

w_status_t adxl38x_clear_self_test_registers(adxl38x_dev_t *dev);

w_status_t adxl38x_get_sts_reg(adxl38x_dev_t *dev, adxl38x_sts_reg_flags_t *status_flags);

w_status_t adxl38x_get_raw_xyz(adxl38x_dev_t *dev, uint16_t *raw_x, uint16_t *raw_y,
							   uint16_t *raw_z);

w_status_t adxl38x_get_temp(adxl38x_dev_t *dev, float *temp_degC);

w_status_t adxl38x_get_raw_data(adxl38x_dev_t *dev, enum adxl38x_ch_select channels,
								uint16_t *raw_x, uint16_t *raw_y, uint16_t *raw_z,
								uint16_t *raw_temp);

w_status_t adxl38x_get_xyz_gees(adxl38x_dev_t *dev, enum adxl38x_ch_select channels, float *x,
								float *y, float *z);

w_status_t adxl38x_selftest(adxl38x_dev_t *dev, enum adxl38x_op_mode op_mode, bool *st_x,
							bool *st_y, bool *st_z);

w_status_t adxl38x_accel_set_FIFO(adxl38x_dev_t *dev, uint16_t num_samples, bool external_trigger,
								  enum adxl38x_fifo_mode fifo_mode, bool ch_ID_enable,
								  bool read_reset);

w_status_t adxl38x_data_raw_to_gees(adxl38x_dev_t *dev, uint8_t *raw_accel_data, float *data_frac);

#endif // ADXL38X_H
