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

// modify this struct to toggle barometer settings
static ms5611_handle_t handle = {.C = {0}, // will be populated by prom read
								 .bus = I2C_BUS_4,
								 .addr =
									 MS5611_ADDRESS_CSB_HIGH, // default to addr with CSB pin high
								 .osr = MS5611_OSR_1024,
								 .initialized = false};

static void delay_us(uint32_t us) {
	/* Round up to nearest ms tick — FreeRTOS resolution is 1ms */
	vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
}

static w_status_t a_read(uint8_t reg, uint8_t *data, uint8_t len) {
	return i2c_read_reg(handle.bus, (uint8_t)handle.addr, reg, data, len);
}

static w_status_t a_write_cmd(uint8_t cmd) {
	return i2c_write_reg(handle.bus, (uint8_t)handle.addr, cmd, NULL, 0);
}

static w_status_t a_read_adc(uint32_t *out) {
	uint8_t buf[3];
	if (a_read(MS5611_CMD_ADC_READ, buf, 3) != W_SUCCESS) {
		return W_FAILURE;
	}
	*out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
	return W_SUCCESS;
}

// ensure that the calibration coefficients stored in the PROM are read correctly by the
// microcontroller. Reference repo in the design doc.
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
		if ((cnt % 2) == 1) /* check bit */
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

	if (n_rem == crc) {
		return W_SUCCESS;
	} else {
		log_text(1, "ms5611", "CRC check failed: expected %u, got %u", crc, n_rem);
		return W_FAILURE;
	}
}

static w_status_t ms5611_prom_read(void) {
	uint8_t i, buf[2];
	uint16_t C[8];
	w_status_t status = W_SUCCESS;

	// copy into a local var first to avoid modifying the handle with corrupt data on read failure
	for (i = 0; i < 8; i++) {
		status |= a_read(MS5611_CMD_PROM_READ_BASE + i * 2, buf, 2);
		C[i] = ((uint16_t)buf[0] << 8) | buf[1];
	}

	status |= a_ms5611_crc_check(C, (uint8_t)(C[7] & 0x0F));

	if (W_SUCCESS == status) {
		for (i = 0; i < 8; i++) {
			handle.C[i] = C[i];
		}

		log_text(
			1,
			"ms5611",
			"INFO: PROM read successful, coefficients: C1=%u, C2=%u, C3=%u, C4=%u, C5=%u, C6=%u",
			handle.C[1],
			handle.C[2],
			handle.C[3],
			handle.C[4],
			handle.C[5],
			handle.C[6]);
	} else {
		log_text(1, "ms5611", "ERROR: PROM read failed.");
	}

	return status;
}

// check iic and sanity of prom coefficients by doing crc checks
w_status_t ms5611_check_sanity(void) {
	w_status_t status = W_SUCCESS;

	if (!handle.initialized) {
		log_text(1, "ms5611", "WARN: sanity check called before successful initialization");
		return W_FAILURE;
	}

	uint8_t buf[2];
	uint16_t C[8];

	// read prom coefficients again
	for (uint8_t i = 0; i < 8; i++) {
		if (W_SUCCESS !=
			i2c_read_reg(
				handle.bus, (uint8_t)handle.addr, MS5611_CMD_PROM_READ_BASE + i * 2, buf, 2)) {
			log_text(1, "ms5611", "ERROR: failed to read PROM coefficient %u in sanity check", i);
			return W_FAILURE;
		}
		C[i] = ((uint16_t)buf[0] << 8) | buf[1];
	}

	// perform crc check on prom coefficients
	if (W_FAILURE == a_ms5611_crc_check(C, (uint8_t)(C[7] & 0x0F))) {
		log_text(1, "ms5611", "ERROR: CRC check failed in sanity check");
		return W_FAILURE;
	}

	return W_SUCCESS;
}

// init by reseting and read
w_status_t ms5611_init(void) {
	w_status_t status = W_SUCCESS;

	// send reset command to clear old calibration data and reset sensor state
	status |= a_write_cmd(MS5611_CMD_RESET);

	// set pin 2 to high to select i2c addr 0x77
	// status |= gpio_write(GPIO_PIN_MS5611, GPIO_LEVEL_HIGH, 10);

	// read prom coefficients
	status |= ms5611_prom_read();

	if (W_SUCCESS != status) {
		log_text(1, "ms5611", "ERROR: initialization failed during reset or PROM read");
		return W_FAILURE;
	} else {
		handle.initialized = true;
		log_text(1, "ms5611", "INFO: initialization successful");
		// check sanity of prom coefficients and etc
		status |= ms5611_check_sanity();
	}

	return status;
}

w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result) {
	uint32_t d1, d2;
	int32_t dt, temp;
	int64_t off, sens, p, off2, sens2;

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
		return W_FAILURE;
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
	dt = (int32_t)d2 - ((int32_t)handle.C[MS5611_COEFF_TREF] << 8);
	temp = 2000 + (int32_t)(((int64_t)dt * handle.C[MS5611_COEFF_TEMPSENS]) >> 23);
	off = ((int64_t)handle.C[MS5611_COEFF_OFF] << 16) +
		  (((int64_t)handle.C[MS5611_COEFF_TCO] * dt) >> 7);
	sens = ((int64_t)handle.C[MS5611_COEFF_SENS] << 15) +
		   (((int64_t)handle.C[MS5611_COEFF_TCS] * dt) >> 8);

	/* Second-order cold compensation */
	off2 = 0;
	sens2 = 0;

	if (temp < 2000) {
		off2 = 61 * (int64_t)(temp - 2000) * (temp - 2000) / 16;
		sens2 = 29 * (int64_t)(temp - 2000) * (temp - 2000) / 16;

		if (temp < -1500) {
			off2 += 17 * (int64_t)(temp + 1500) * (temp + 1500);
			sens2 += 9 * (int64_t)(temp + 1500) * (temp + 1500);
		}
	}

	off -= off2;
	sens -= sens2;
	p = ((((int64_t)d1 * sens) >> 21) - off) >> 15;

	result->temperature_centideg = temp;
	result->pressure_centimbar = (int32_t)p;

	return W_SUCCESS;
}