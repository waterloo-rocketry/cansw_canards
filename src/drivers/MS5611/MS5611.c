#include "drivers/MS5611/MS5611.h"

/* Conversion time in microseconds - max values from datasheet for safety */
static const uint32_t CONV_TIME_US[] = {
	600, /* OSR 256  — datasheet max 0.60ms */
	1170, /* OSR 512  — datasheet max 1.17ms */
	2280,
	/* OSR 1024 — datasheet max 2.28ms */ // this one is picked
	4540, /* OSR 2048 — datasheet max 4.54ms */
	9040, /* OSR 4096 — datasheet max 9.04ms */
};

/* D1 (pressure) convert commands indexed by ms5611_osr_t */
static const uint8_t D1_CMD[] = {
	MS5611_CMD_CONVERT_D1_OSR256,
	MS5611_CMD_CONVERT_D1_OSR512,
	MS5611_CMD_CONVERT_D1_OSR1024,
	MS5611_CMD_CONVERT_D1_OSR2048,
	MS5611_CMD_CONVERT_D1_OSR4096,
};

/* D2 (temperature) convert commands indexed by ms5611_osr_t */
static const uint8_t D2_CMD[] = {
	MS5611_CMD_CONVERT_D2_OSR256,
	MS5611_CMD_CONVERT_D2_OSR512,
	MS5611_CMD_CONVERT_D2_OSR1024,
	MS5611_CMD_CONVERT_D2_OSR2048,
	MS5611_CMD_CONVERT_D2_OSR4096,
};

static const int16_t second_comp_temp_threshold =
	2000; /* temperature (in centidegrees) below which second-order compensation is applied */
static const int16_t second_comp_extreme_temp_threshold =
	-1500; /* temperature (in centidegrees) below which additional extreme cold compensation is
			  applied */

// modify this struct to toggle barometer settings
static ms5611_handle_t handle = {.prom_coef = {0}, // will be populated by prom read
								 .bus = I2C_BUS_4,
								 .addr =
									 MS5611_ADDRESS_CSB_LOW, // according to canard board schematic,
															 // CSB is tied to GND, so addr is 0x77
								 .osr = MS5611_OSR_1024,
								 .initialized = false};

/**
 * @brief Delays for a specified number of microseconds.
 * @param us The number of microseconds to delay.
 */
static void delay_us(uint32_t us) {
	/* Round up to nearest ms tick — FreeRTOS resolution is 1ms */
	vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
}

/**
 * @brief Reads one byte of data from the MS5611 sensor over I2C.
 * @param cmd The command to read from the sensor.
 */
static w_status_t a_read(uint8_t reg, uint8_t *data, uint8_t len) {
	return i2c_read_reg(handle.bus, (uint8_t)handle.addr, reg, data, len);
}

/**
 * @brief Writes a command to the MS5611 sensor.
 * @param cmd The command to write.
 */
static w_status_t a_write_cmd(uint8_t cmd) {
	return i2c_write_data(handle.bus, (uint8_t)handle.addr, (uint8_t *)&cmd, (uint8_t)sizeof(cmd));
}

/**
 * @brief Reads the ADC value from the MS5611 sensor.
 * @param out Pointer to store the read ADC value.
 */
static w_status_t a_read_adc(uint32_t *out) {
	uint8_t prom_buf[3];
	if (a_read(MS5611_CMD_ADC_READ, prom_buf, 3) != W_SUCCESS) {
		return W_FAILURE;
	}
	*out = ((uint32_t)prom_buf[0] << 16) | ((uint32_t)prom_buf[1] << 8) | prom_buf[2];
	return W_SUCCESS;
}

/**
 * @brief Perfroms the CRC check on the PROM coefficients readout.
 * @param n_prom Array of 8 uint16_t values read from the PROM (prom_coef[0]..prom_coef[7])
 * @param crc The CRC value read from the PROM (lower 4 bits of prom_coef[7])
 * @return W_SUCCESS if CRC check passes, W_FAILURE if CRC check fails
 * @note This function is from github repository:
 * https://github.com/libdriver/ms5611/blob/main/src/driver_ms5611.h#L37
 */
static w_status_t a_ms5611_crc_check(uint16_t *n_prom, uint8_t crc) {
	uint8_t cnt;
	uint8_t n_bit;
	uint16_t n_rem;
	uint16_t crc_read;

	n_rem = 0x00; /* init 0 */
	crc_read = n_prom[7]; /* get crc */
	n_prom[7] = (0xFF00U & (n_prom[7])); /* init prom */

	for (cnt = 0; cnt < 16; cnt++) /* loop all */
	{
		if (cnt & 0x1) /* check LSB */
		{
			n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF); /* run part1 */
		} else {
			n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8); /* run part1 */
		}
		for (n_bit = 8; n_bit > 0; n_bit--) /* loop all */
		{
			if ((n_rem & 0x8000U) != 0) /* check bit */
			{
				n_rem = (n_rem << 1) ^ 0x3000; /* run part2 */
			} else {
				n_rem = (n_rem << 1); /* run part2 */
			}
		}
	}
	n_rem = (0x000F & (n_rem >> 12)); /* get rem */
	n_prom[7] = crc_read; /* set crc read */
	n_rem ^= 0x00; /* xor */

	if (n_rem != crc) {
		log_text(1, "ms5611", "CRC check failed: expected %u, got %u", crc, n_rem);
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Reads calibration coefficients from the PROM, performs CRC check, and stores them in the
 * handle struct.
 */
static w_status_t ms5611_prom_read(void) {
	uint8_t i, prom_buf[2];
	uint16_t prom_coef[8];
	w_status_t status = W_SUCCESS;

	// copy into a local var first to avoid modifying the handle with corrupt data on read failure
	for (i = 0; i < 8; i++) {
		status |= a_read(MS5611_CMD_PROM_READ_BASE + i * 2, prom_buf, 2);
		prom_coef[i] = ((uint16_t)prom_buf[0] << 8) | prom_buf[1];
	}

	status |= a_ms5611_crc_check(prom_coef, (uint8_t)(prom_coef[7] & 0x0F));

	if (W_SUCCESS == status) {
		for (i = 0; i < 8; i++) {
			handle.prom_coef[i] = prom_coef[i];
		}

		log_text(
			1,
			"ms5611",
			"INFO: PROM read successful, coefficients: C1=%u, C2=%u, C3=%u, C4=%u, C5=%u, C6=%u",
			handle.prom_coef[1],
			handle.prom_coef[2],
			handle.prom_coef[3],
			handle.prom_coef[4],
			handle.prom_coef[5],
			handle.prom_coef[6]);
	} else {
		log_text(1, "ms5611", "ERROR: PROM read failed.");
	}

	return status;
}

/**
 * @brief Initialized the sensor by resetting it, reading calibration coeff from PROM, and perform
 * crc check on the coefficients.
 */
w_status_t ms5611_init(void) {
	if (W_SUCCESS != a_write_cmd(MS5611_CMD_RESET)) {
		log_text(1, "ms5611", "ERROR: initialization failed during command reset.");
		return W_FAILURE;
	}

	if (W_SUCCESS != ms5611_prom_read()) {
		log_text(1, "ms5611", "ERROR: initialization failed during PROM read .");
		return W_FAILURE;
	}

	handle.initialized = true;
	log_text(1, "ms5611", "INFO: initialization successful");

	return W_SUCCESS;
}

/**
 * @brief Reads raw pressure and temperature data from the MS5611 sensor, applies compensation, and
 * stores the results in the provided struct.
 * @param result Pointer to store the raw pressure and temperature results
 * @note this function also applies the compensation algorithm, but does not convert units
 * @note see MS5611 datasheet page 7 - 8 for details on the calculation
 * (temperature in centidegrees prom_coef, pressure in centimbar)
 */
w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result) {
	uint32_t d1, d2; /* d1 is raw pressure reading, d2 is raw temperature reading */
	int32_t dt, temp; /* dt is temperature difference, temp is compensated temperature */
	int64_t off, sens, p; /* prom coefficients for first order */
	int64_t T2, off2, sens2; /* second-order compensation terms */

	if (NULL == result) {
		log_text(1, "ms5611", "ERROR: NULL pointer passed to ms5611_get_pressure");
		return W_INVALID_PARAM;
	}

	if (!handle.initialized) {
		log_text(1, "ms5611", "ERROR: attempted to read pressure before successful initialization");
		return W_FAILURE;
	}

	/* D2: temperature conversion */
	if (W_FAILURE == a_write_cmd(D2_CMD[handle.osr])) {
		log_text(1, "ms5611", "ERROR: failed to write temperature conversion command");
		return W_FAILURE;
	}

	delay_us(CONV_TIME_US[handle.osr]);

	if (W_FAILURE == a_read_adc(&d2)) {
		log_text(1, "ms5611", "ERROR: failed to read temperature ADC");
		return W_IO_ERROR;
	}

	/* D1: pressure conversion */
	if (W_FAILURE == a_write_cmd(D1_CMD[handle.osr])) {
		log_text(1, "ms5611", "ERROR: failed to write pressure conversion command");
		return W_FAILURE;
	}

	delay_us(CONV_TIME_US[handle.osr]);

	if (W_FAILURE == a_read_adc(&d1)) {
		log_text(1, "ms5611", "ERROR: failed to read pressure ADC");
		return W_FAILURE;
	}

	/* First-order compensation */
	dt = (int32_t)d2 - ((int32_t)handle.prom_coef[MS5611_COEFF_TREF] << 8);
	temp = second_comp_temp_threshold +
		   (int32_t)(((int64_t)dt * handle.prom_coef[MS5611_COEFF_TEMPSENS]) >> 23);
	off = ((int64_t)handle.prom_coef[MS5611_COEFF_OFF] << 16) +
		  (((int64_t)handle.prom_coef[MS5611_COEFF_TCO] * dt) >> 7);
	sens = ((int64_t)handle.prom_coef[MS5611_COEFF_SENS] << 15) +
		   (((int64_t)handle.prom_coef[MS5611_COEFF_TCS] * dt) >> 8);
	p = ((((int64_t)d1 * sens) >> 21) - off) >> 15;

	/* Second-order cold compensation */
	T2 = 0;
	off2 = 0;
	sens2 = 0;

	if (temp < second_comp_temp_threshold) {
		T2 = (int64_t)(dt * dt) >> 31;
		off2 = 5 * (int64_t)((temp - second_comp_temp_threshold) *
							 (temp - second_comp_temp_threshold)) >>
			   1;
		sens2 = 5 * (int64_t)((temp - second_comp_temp_threshold) *
							  (temp - second_comp_temp_threshold)) >>
				2;

		if (temp < second_comp_extreme_temp_threshold) {
			off2 += 7 * (int64_t)(temp - second_comp_extreme_temp_threshold) *
					(temp - second_comp_extreme_temp_threshold);
			sens2 += 11 * (int64_t)((temp - second_comp_extreme_temp_threshold) *
									(temp - second_comp_extreme_temp_threshold)) >>
					 1;
		}

		temp -= T2;
		off -= off2;
		sens -= sens2;
	}

	result->temperature_centideg = temp;
	result->pressure_centimbar = (int32_t)p;

	return W_SUCCESS;
}
