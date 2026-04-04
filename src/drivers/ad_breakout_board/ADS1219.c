/*
ACK: https://github.com/binomaiheu/ADS1219
Based on above repository
*/

/**
 * @file ADS1219.c
 * @brief ADS1219 24-bit ADC driver -- STM32 C implementation
 *
 * C port of ADS1219.cpp (Arduino / Wire) adapted for the project's I2C layer.
 */

#include "drivers/ad_breakout_board/ADS1219.h"
#include "common/math/math.h"
#include "drivers/i2c/i2c.h"

/* ── internal helpers (static, replace the C++ private methods) ────────── */

static w_status_t ads1219_send_cmd(ads1219_handle_t *p_handle, uint8_t cmd) {
	return i2c_write_data(p_handle->bus, p_handle->i2c_addr, &cmd, 1);
}

static w_status_t ads1219_read_register(ads1219_handle_t *p_handle, uint8_t reg, uint8_t *data) {
	return i2c_read_reg(p_handle->bus, p_handle->i2c_addr, reg, data, 1);
}

static w_status_t ads1219_write_register(ads1219_handle_t *p_handle, uint8_t data) {
	return i2c_write_reg(p_handle->bus, p_handle->i2c_addr, ADS1219_CMD_WREG, &data, 1);
}

static w_status_t ads1219_modify_register(ads1219_handle_t *p_handle, uint8_t value, uint8_t mask) {
	uint8_t data;
	w_status_t status = ads1219_read_register(p_handle, ADS1219_CMD_RREG_CONFIG, &data);
	if (W_SUCCESS != status) {
		return status;
	}

	data = (data & mask) | (value & ~mask);
	return ads1219_write_register(p_handle, data);
}

/* ── public API ────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the handle and reset the device.
 *
 * @param[out] p_handle  Handle to populate
 * @param[in]  bus     Project I2C bus the device is on
 * @param[in]  addr    7-bit I2C address (use ADS1219_I2C_ADDRESS for default)
 * @return W_SUCCESS or an error code
 */
w_status_t ads1219_init(ads1219_handle_t *p_handle, i2c_bus_t bus, uint8_t addr) {
	if (!p_handle) {
		return W_INVALID_PARAM;
	}
	// assumes to use internal vref
	p_handle->bus = bus;
	p_handle->i2c_addr = addr;
	p_handle->aref_n = 0.0f;
	p_handle->aref_p = 2048.0f;
	p_handle->initialized = false;

	/* Reset the device to known defaults */
	w_status_t status = ads1219_send_cmd(p_handle, ADS1219_CMD_RESET);
	if (status != W_SUCCESS) {
		return status;
	}

	p_handle->initialized = true;

	return ads1219_get_gain(p_handle, &(p_handle->gain));
}

/**
 * @brief Issue a RESET command.
 * @param p_handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_reset(ads1219_handle_t *p_handle) {
	return ads1219_send_cmd(p_handle, ADS1219_CMD_RESET);
}

/**
 * @brief Issue a START / SYNC command (begin conversion).
 * @param handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_start(ads1219_handle_t *p_handle) {
	return ads1219_send_cmd(p_handle, ADS1219_CMD_START_SYNC);
}

/**
 * @brief Issue a POWERDOWN command.
 * @param handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_powerdown(ads1219_handle_t *p_handle) {
	return ads1219_send_cmd(p_handle, ADS1219_CMD_POWERDOWN);
}

/**
 * @brief Read the current gain setting from the config register.
 * @param handle handle to the ADC data
 * @param[out] gain  ADS1219_GAIN_ONE or ADS1219_GAIN_FOUR
 * @return status of function
 */
w_status_t ads1219_get_gain(ads1219_handle_t *p_handle, uint8_t *gain) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(p_handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*gain = (reg & ~ADS1219_CONFIG_MASK_GAIN) ? ADS1219_GAIN_FOUR : ADS1219_GAIN_ONE;
	return W_SUCCESS;
}

/**
 * @brief Set the gain.
 * @param p_handle handle to the ADC data
 * @param[in] gain  ADS1219_GAIN_ONE or ADS1219_GAIN_FOUR
 * @return status of function
 */
w_status_t ads1219_set_gain(ads1219_handle_t *p_handle, uint8_t gain) {
	uint8_t value;
	if (gain == ADS1219_GAIN_ONE) {
		value = 0;
	} else if (gain == ADS1219_GAIN_FOUR) {
		value = (1 << 4);
	} else {
		return W_INVALID_PARAM;
	}

	p_handle->gain = gain;

	return ads1219_modify_register(p_handle, value, ADS1219_CONFIG_MASK_GAIN);
}

/**
 * @brief Read the current voltage-reference setting.
 * @param p_handle handle to the ADC data
 * @param[out] vref  ADS1219_VREF_INTERNAL or ADS1219_VREF_EXTERNAL
 * @return status of function
 */
w_status_t ads1219_get_vref(ads1219_handle_t *p_handle, uint8_t *vref) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(p_handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*vref = (reg & ~ADS1219_CONFIG_MASK_VREF) ? ADS1219_VREF_EXTERNAL : ADS1219_VREF_INTERNAL;
	return W_SUCCESS;
}

/**
 * @brief Set the voltage reference and ref voltages.
 * @param p_handle handle to the ADC data
 * @param[in] vref    ADS1219_VREF_INTERNAL or ADS1219_VREF_EXTERNAL
 * @param[in] aref_n  Negative reference in mV (ignored for internal, 0 used)
 * @param[in] aref_p  Positive reference in mV (ignored for internal, 2048 used)
 * @return status of function
 */
w_status_t ads1219_set_vref(ads1219_handle_t *p_handle, uint8_t vref, float64_t aref_n,
							float64_t aref_p) {
	uint8_t value;
	if (vref == ADS1219_VREF_INTERNAL) {
		value = 0;
	} else if (vref == ADS1219_VREF_EXTERNAL) {
		value = 1;
	} else {
		return W_INVALID_PARAM;
	}

	w_status_t status = ads1219_modify_register(p_handle, value, ADS1219_CONFIG_MASK_VREF);
	if (status != W_SUCCESS) {
		return status;
	}

	if (vref == ADS1219_VREF_EXTERNAL) {
		p_handle->aref_n = aref_n;
		p_handle->aref_p = aref_p;
	} else {
		p_handle->aref_n = 0.0f;
		p_handle->aref_p = 2048.0f;
	}
	return W_SUCCESS;
}

/**
 * @brief Read the current data-rate setting.
 * @param p_handle handle to the ADC data
 * @param[out] rate  0-3 (ADS1219_DATARATE_*)
 * @return status of function
 */
w_status_t ads1219_get_data_rate(ads1219_handle_t *p_handle, uint8_t *rate) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(p_handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*rate = (reg & ~ADS1219_CONFIG_MASK_DR) >> 2;
	return W_SUCCESS;
}

/**
 * @brief Set the data rate.
 * @param p_handle handle to the ADC data
 * @param[in] rate  0-3 (ADS1219_DATARATE_*)
 * @return status of function
 */
w_status_t ads1219_set_data_rate(ads1219_handle_t *p_handle, uint8_t rate) {
	if (rate > 3) {
		return W_INVALID_PARAM;
	}
	return ads1219_modify_register(p_handle, rate << 2, ADS1219_CONFIG_MASK_DR);
}

/**
 * @brief Read the current conversion-mode setting.
 * @param p_handle handle to the ADC data
 * @param[out] mode  ADS1219_CM_SINGLE_SHOT or ADS1219_CM_CONTINUOUS
 * @return status of function
 */
w_status_t ads1219_get_conversion_mode(ads1219_handle_t *p_handle, uint8_t *mode) {
	uint8_t reg;
	w_status_t status = ads1219_read_register(p_handle, ADS1219_CMD_RREG_CONFIG, &reg);
	if (status != W_SUCCESS) {
		return status;
	}

	*mode = (reg & ~ADS1219_CONFIG_MASK_CM) >> 1;
	return W_SUCCESS;
}

/**
 * @brief Set the conversion mode.
 * @param p_handle handle to the ADC data
 * @param[in] mode  ADS1219_CM_SINGLE_SHOT or ADS1219_CM_CONTINUOUS
 * @return status of function
 */
w_status_t ads1219_set_conversion_mode(ads1219_handle_t *p_handle, uint8_t mode) {
	if (mode > 1) {
		return W_INVALID_PARAM;
	}
	return ads1219_modify_register(p_handle, mode << 1, ADS1219_CONFIG_MASK_CM);
}

/**
 * @brief Check whether a conversion result is ready.
 * @param p_handle handle to the ADC data
 * @param[out] ready  true when a new result is available
 * @return status of function
 */
w_status_t ads1219_conversion_ready(ads1219_handle_t *p_handle, bool *ready) {
	uint8_t stat;
	w_status_t status = ads1219_read_register(p_handle, ADS1219_CMD_RREG_STATUS, &stat);
	if (status != W_SUCCESS) {
		*ready = false;
		return status;
	}

	*ready = ((stat & 0x80) == 0x80);
	return W_SUCCESS;
}

/**
 * @brief Set the MUX Channel that is used
 * @param p_handle handle to the ADC data
 * @param mux the channel that is used (Macros defined above)
 * @return status of function
 */
w_status_t ads1219_set_channel(ads1219_handle_t *p_handle, uint8_t mux) {
	return ads1219_modify_register(p_handle, mux, ADS1219_CONFIG_MASK_MUX);
}

/**
 * @brief reads the value of the ads1219 and return the raw value
 * @param p_handle handle to the ADC data
 * @param value is a pointer to where the raw value will be stored
 * @return status of function
 */
w_status_t ads1219_read_value(ads1219_handle_t *p_handle, uint32_t *value) {
	uint8_t buf[3];

	/*
	 * RDATA is a two-phase command:  write the RDATA byte, then clock out
	 * 3 data bytes.  i2c_read_reg maps perfectly:
	 *     START | addr+W | RDATA | RSTART | addr+R | buf[0..2] | STOP
	 */
	w_status_t status = i2c_read_reg(p_handle->bus, p_handle->i2c_addr, ADS1219_CMD_RDATA, buf, 3);
	if (W_SUCCESS != status) {
		return status;
	}

	/* concat the raw data */
	uint32_t raw = ((((uint32_t)buf[0] << 8) | (uint32_t)buf[1]) << 8) | (uint32_t)buf[2];

	// preserve two's-complement in 32 bit
	raw = (raw ^ 0x00800000) - 0x00800000;

	*value = raw;

	/* TODO: double check */
	if ((0x7FFFFF < raw) && (0xFF800001 > raw)) {
		return W_OVERFLOW;
	}
	return W_SUCCESS;
}

/**
 * @brief Convert a raw ADC count to millivolts.
 * @param[in]  p_handle    Device handle (carries reference voltages)
 * @param[in]  adc_count Raw signed count from a read function
 * @param[out] mv        Result in millivolts
 * @return status of function
 */
w_status_t ads1219_millivolts(ads1219_handle_t *p_handle, const int32_t adc_count, float64_t *mv) {
	float64_t span = p_handle->aref_p - p_handle->aref_n;

	if (p_handle->gain == ADS1219_GAIN_ONE) {
		*mv = (float64_t)adc_count * span / 8388608.0f;
	} else if (p_handle->gain == ADS1219_GAIN_FOUR) {
		*mv = (float64_t)adc_count * span / 4.0f / 8388608.0f;
	} else {
		return W_INVALID_PARAM;
	}

	return W_SUCCESS;
}

/**
 * @brief Return the converted ADC voltage (millivolts)
 * @param p_handle handle to the ADC data
 * @param data this is pointer to the data return which would be in terms of mV
 * @return status of function
 */
w_status_t ads1219_get_millivolts(ads1219_handle_t *p_handle, float64_t *data) {
	uint32_t value;

	if (W_SUCCESS != ads1219_read_value(p_handle, &value)) {
		return W_FAILURE;
	}

	return ads1219_millivolts(p_handle, (int32_t)value, data);
}
