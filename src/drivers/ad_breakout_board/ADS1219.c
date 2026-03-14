/**
 * @file ADS1219.c
 * @brief ADS1219 24-bit ADC driver -- STM32 C implementation
 *
 * C port of ADS1219.cpp (Arduino / Wire) adapted for the project's I2C layer.
 *
 * ## How the I2C mapping works
 *
 * The project helper i2c_write_reg(bus, addr, reg, data, len) issues:
 *     START | addr+W | reg | data[0..len-1] | STOP
 *
 * and i2c_read_reg(bus, addr, reg, data, len) issues:
 *     START | addr+W | reg | REPEATED-START | addr+R | data[0..len-1] | STOP
 *
 * The ADS1219 protocol is command-oriented -- every operation starts with a
 * single command byte, optionally followed by data.  We map the command byte
 * to the `reg` parameter of the I2C helpers.  For "send command only" we use
 * i2c_write_reg with len=0 (no data bytes after the command), and for
 * "read after command" we use i2c_read_reg with the command as `reg`.
 *
 * ## Delay / tick
 *
 * HAL_Delay is used for the conversion wait.  In a FreeRTOS environment the
 * SysTick is overridden so HAL_Delay yields properly.  HAL_GetTick provides
 * the monotonic ms counter for the timeout loop (replaces Arduino millis()).
 */

#include "drivers/ad_breakout_board/ADS1219.h"
#include "drivers/i2c/i2c.h"
#include "stm32h7xx_hal.h"

/* ── internal helpers (static, replace the C++ private methods) ────────── */

/**
 * send_cmd -- send a bare command byte (RESET, START, POWERDOWN, RDATA, RREG)
 *
 * Original: ADS1219::send_cmd  ->  _write(&cmd, 1)
 *           which did beginTransmission / write(cmd) / endTransmission.
 *
 * We use i2c_write_reg with the command as the "register address" and zero
 * data bytes, which produces: START | addr+W | cmd | STOP -- identical on
 * the wire.
 */
static w_status_t ads1219_send_cmd(ads1219_handle_t *handle, uint8_t cmd) {
	uint8_t dummy_data = 0;
	return i2c_write_reg(handle->bus, handle->i2c_addr, cmd, &dummy_data, 1);
}

/**
 * read_register -- send a RREG command then read one byte back.
 *
 * Original: ADS1219::_read_register  ->  send_cmd(reg) ; _read(data, 1)
 *
 * i2c_read_reg(bus, addr, reg, buf, 1) produces:
 *     START | addr+W | reg | RSTART | addr+R | buf[0] | STOP
 * which is exactly the two-phase sequence the ADS1219 expects for RREG.
 */
static w_status_t ads1219_read_register(ads1219_handle_t *handle, uint8_t reg, uint8_t *data) {
	return i2c_read_reg(handle->bus, handle->i2c_addr, reg, data, 1);
}

/**
 * write_register -- write the config register (WREG + 1 data byte).
 *
 * Original: ADS1219::_write_register  ->  _write(&data, 1, true, &WREG, 1)
 *           which did beginTransmission / write(WREG) / write(data) / end.
 *
 * i2c_write_reg(bus, addr, WREG, &data, 1) produces:
 *     START | addr+W | WREG | data | STOP
 */
static w_status_t ads1219_write_register(ads1219_handle_t *handle, uint8_t data) {
	return i2c_write_reg(handle->bus, handle->i2c_addr, ADS1219_CMD_WREG, &data, 1);
}

/**
 * modify_register -- read-modify-write a field in the config register.
 *
 * Original: ADS1219::_modify_register
 *
 * @param value  New field value, already shifted to its bit position.
 * @param mask   Mask with 1s everywhere *except* the field bits.
 */
static w_status_t ads1219_modify_register(ads1219_handle_t *handle, uint8_t value, uint8_t mask) {
	uint8_t data;
	w_status_t status = ads1219_read_register(handle, ADS1219_CMD_RREG_CONFIG, &data);
	if (W_SUCCESS != status) {
		return status;
	}

	data = (data & mask) | (value & ~mask);
	return ads1219_write_register(handle, data);
}

/**
 * read_value -- issue RDATA then read 3 bytes and decode the 24-bit two's-
 * complement value.
 *
 * Original: ADS1219::_read_value
 *
 * The sign extension trick is the same: shift the MSB into the sign-bit
 * position of a 32-bit int and shift back.  In C this is implementation-
 * defined for negative values but works on every ARM GCC we target.
 */
static w_status_t ads1219_read_value(ads1219_handle_t *handle, int32_t *value) {
	uint8_t buf[3];

	/*
	 * RDATA is a two-phase command:  write the RDATA byte, then clock out
	 * 3 data bytes.  i2c_read_reg maps perfectly:
	 *     START | addr+W | RDATA | RSTART | addr+R | buf[0..2] | STOP
	 */
	w_status_t status = i2c_read_reg(handle->bus, handle->i2c_addr, ADS1219_CMD_RDATA, buf, 3);
	if (W_SUCCESS != status) {
		return status;
	}

	/* 24-bit two's-complement -> 32-bit int with sign extension */
	int32_t raw = (((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) | ((int32_t)buf[2] << 8)) >> 8;

	*value = raw;

	/* Over / underflow detection (full-scale codes) */
	if ((int32_t)0x7FFFFF <= raw) {
		return W_OVERFLOW;
	}
	if ((int32_t)0xFF800000 >= raw) { /* sign-extended 0x800000 */
		return W_OVERFLOW;
	}
	return W_SUCCESS;
}

/* ── public API ────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the handle and reset the device.
 *
 * @param[out] handle  Handle to populate
 * @param[in]  bus     Project I2C bus the device is on
 * @param[in]  addr    7-bit I2C address (use ADS1219_I2C_ADDRESS for default)
 * @return W_SUCCESS or an error code
 */
w_status_t ads1219_init(ads1219_handle_t *handle, i2c_bus_t bus, uint8_t addr) {
	if (!handle) {
		return W_INVALID_PARAM;
	}
	// assumes to use internal vref
	handle->bus = bus;
	handle->i2c_addr = addr;
	handle->aref_n = 0.0f;
	handle->aref_p = 2048.0f;
	handle->initialized = false;

	/* Reset the device to known defaults */
	w_status_t status = ads1219_send_cmd(handle, ADS1219_CMD_RESET);
	if (status != W_SUCCESS) {
		return status;
	}

	handle->initialized = true;

	return ads1219_get_gain(handle, &(handle->gain));
}

/**
 * @brief Issue a RESET command.
 * @param handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_reset(ads1219_handle_t *handle) {
	return ads1219_send_cmd(handle, ADS1219_CMD_RESET);
}

/**
 * @brief Issue a START / SYNC command (begin conversion).
 * @param handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_start(ads1219_handle_t *handle) {
	return ads1219_send_cmd(handle, ADS1219_CMD_START_SYNC);
}

/**
 * @brief Issue a POWERDOWN command.
 * @param handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_powerdown(ads1219_handle_t *handle) {
	return ads1219_send_cmd(handle, ADS1219_CMD_POWERDOWN);
}

/**
 * @brief Read the current gain setting from the config register.
 * @param handle handle to the ADC data
 * @param[out] gain  ADS1219_GAIN_ONE or ADS1219_GAIN_FOUR
 * @return status of function
 */
w_status_t ads1219_get_gain(ads1219_handle_t *handle, uint8_t *gain) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*gain = (reg & ~ADS1219_CONFIG_MASK_GAIN) ? ADS1219_GAIN_FOUR : ADS1219_GAIN_ONE;
	return W_SUCCESS;
}

/**
 * @brief Set the gain.
 * @param handle handle to the ADC data
 * @param[in] gain  ADS1219_GAIN_ONE or ADS1219_GAIN_FOUR
 * @return status of function
 */
w_status_t ads1219_set_gain(ads1219_handle_t *handle, uint8_t gain) {
	uint8_t value;
	if (gain == ADS1219_GAIN_ONE) {
		value = 0;
	} else if (gain == ADS1219_GAIN_FOUR) {
		value = (1 << 4);
	} else {
		return W_INVALID_PARAM;
	}

	handle->gain = gain;

	return ads1219_modify_register(handle, value, ADS1219_CONFIG_MASK_GAIN);
}

/**
 * @brief Read the current voltage-reference setting.
 * @param handle handle to the ADC data
 * @param[out] vref  ADS1219_VREF_INTERNAL or ADS1219_VREF_EXTERNAL
 * @return status of function
 */
w_status_t ads1219_get_vref(ads1219_handle_t *handle, uint8_t *vref) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*vref = (reg & ~ADS1219_CONFIG_MASK_VREF) ? ADS1219_VREF_EXTERNAL : ADS1219_VREF_INTERNAL;
	return W_SUCCESS;
}

/**
 * @brief Set the voltage reference and ref voltages.
 * @param handle handle to the ADC data
 * @param[in] vref    ADS1219_VREF_INTERNAL or ADS1219_VREF_EXTERNAL
 * @param[in] aref_n  Negative reference in mV (ignored for internal, 0 used)
 * @param[in] aref_p  Positive reference in mV (ignored for internal, 2048 used)
 * @return status of function
 */
w_status_t ads1219_set_vref(ads1219_handle_t *handle, uint8_t vref, float aref_n, float aref_p) {
	uint8_t value;
	if (vref == ADS1219_VREF_INTERNAL) {
		value = 0;
	} else if (vref == ADS1219_VREF_EXTERNAL) {
		value = 1;
	} else {
		return W_INVALID_PARAM;
	}

	w_status_t status = ads1219_modify_register(handle, value, ADS1219_CONFIG_MASK_VREF);
	if (status != W_SUCCESS) {
		return status;
	}

	if (vref == ADS1219_VREF_EXTERNAL) {
		handle->aref_n = aref_n;
		handle->aref_p = aref_p;
	} else {
		handle->aref_n = 0.0f;
		handle->aref_p = 2048.0f;
	}
	return W_SUCCESS;
}

/**
 * @brief Read the current data-rate setting.
 * @param handle handle to the ADC data
 * @param[out] rate  0-3 (ADS1219_DATARATE_*)
 * @return status of function
 */
w_status_t ads1219_get_data_rate(ads1219_handle_t *handle, uint8_t *rate) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*rate = (reg & ~ADS1219_CONFIG_MASK_DR) >> 2;
	return W_SUCCESS;
}

/**
 * @brief Set the data rate.
 * @param handle handle to the ADC data
 * @param[in] rate  0-3 (ADS1219_DATARATE_*)
 * @return status of function
 */
w_status_t ads1219_set_data_rate(ads1219_handle_t *handle, uint8_t rate) {
	if (rate > 3) {
		return W_INVALID_PARAM;
	}
	return ads1219_modify_register(handle, rate << 2, ADS1219_CONFIG_MASK_DR);
}

/**
 * @brief Read the current conversion-mode setting.
 * @param handle handle to the ADC data
 * @param[out] mode  ADS1219_CM_SINGLE_SHOT or ADS1219_CM_CONTINUOUS
 * @return status of function
 */
w_status_t ads1219_get_conversion_mode(ads1219_handle_t *handle, uint8_t *mode) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*mode = (reg & ~ADS1219_CONFIG_MASK_CM) >> 1;
	return W_SUCCESS;
}

/**
 * @brief Set the conversion mode.
 * @param handle handle to the ADC data
 * @param[in] mode  ADS1219_CM_SINGLE_SHOT or ADS1219_CM_CONTINUOUS
 * @return status of function
 */
w_status_t ads1219_set_conversion_mode(ads1219_handle_t *handle, uint8_t mode) {
	if (mode > 1) {
		return W_INVALID_PARAM;
	}
	return ads1219_modify_register(handle, mode << 1, ADS1219_CONFIG_MASK_CM);
}

/**
 * @brief Check whether a conversion result is ready.
 * @param handle handle to the ADC data
 * @param[out] ready  true when a new result is available
 * @return status of function
 */
w_status_t ads1219_conversion_ready(ads1219_handle_t *handle, bool *ready) {
	uint8_t stat;
	w_status_t status = ads1219_read_register(handle, ADS1219_CMD_RREG_STATUS, &stat);
	if (status != W_SUCCESS) {
		*ready = false;
		return status;
	}

	*ready = ((stat & 0x80) == 0x80);
	return W_SUCCESS;
}

/**
 * @brief Set the MUX Channel that is used
 * @param handle handle to the ADC data
 * @param mux the channel that is used (Macros defined above)
 * @return status of function
 */
w_status_t ads1219_set_channel(ads1219_handle_t *handle, uint8_t mux) {
	return ads1219_modify_register(handle, mux, ADS1219_CONFIG_MASK_MUX);
}

/**
 * @brief Convert a raw ADC count to millivolts.
 * @param[in]  handle    Device handle (carries reference voltages)
 * @param[in]  adc_count Raw signed count from a read function
 * @param[out] mv        Result in millivolts
 * @return status of function
 */
w_status_t ads1219_millivolts(ads1219_handle_t *handle, const int32_t adc_count, float *mv) {
	float span = handle->aref_p - handle->aref_n;

	if (handle->gain == ADS1219_GAIN_ONE) {
		*mv = (float)adc_count * span / 8388608.0f;
	} else if (handle->gain == ADS1219_GAIN_FOUR) {
		*mv = (float)adc_count * span / 4.0f / 8388608.0f;
	} else {
		return W_INVALID_PARAM;
	}

	return W_SUCCESS;
}

/**
 * @brief Return the converted ADC voltage (millivolts)
 * @param handle handle to the ADC data
 * @param data this is pointer to the data return which would be in terms of mV
 * @return status of function
 */
w_status_t ads1219_get_millivolts(ads1219_handle_t *handle, float *data) {
	int32_t value;

	if (W_SUCCESS != ads1219_read_value(handle, &value)) {
		return W_FAILURE;
	}

	return ads1219_millivolts(handle, value, data);
}
