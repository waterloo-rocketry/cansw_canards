#include "drivers/MS5611/MS5611.h"
#include "drivers/timer/timer.h"

/* IIC address: CSB pin low = 0x77, CSB pin high = 0x76 */
typedef enum {
	MS5611_ADDRESS_CSB_LOW = 0x77,
	MS5611_ADDRESS_CSB_HIGH = 0x76
} ms5611_address_t;

// reset command
#define MS5611_CMD_RESET 0x1E

// pressure conversion commands (D1)
#define MS5611_CMD_CONVERT_D1_OSR256 0x40
#define MS5611_CMD_CONVERT_D1_OSR512 0x42
#define MS5611_CMD_CONVERT_D1_OSR1024 0x44
#define MS5611_CMD_CONVERT_D1_OSR2048 0x46
#define MS5611_CMD_CONVERT_D1_OSR4096 0x48

// temperature conversion commands (D2)
#define MS5611_CMD_CONVERT_D2_OSR256 0x50
#define MS5611_CMD_CONVERT_D2_OSR512 0x52
#define MS5611_CMD_CONVERT_D2_OSR1024 0x54
#define MS5611_CMD_CONVERT_D2_OSR2048 0x56
#define MS5611_CMD_CONVERT_D2_OSR4096 0x58

// ADC read command
#define MS5611_CMD_ADC_READ 0x00
#define MS5611_CMD_PROM_READ_BASE 0xA0 /* OR with (addr << 1) for addr 0..7 */

// calibration coefficient indices in PROM readout
#define MS5611_COEFF_SENS 1
#define MS5611_COEFF_OFF 2
#define MS5611_COEFF_TCS 3
#define MS5611_COEFF_TCO 4
#define MS5611_COEFF_TREF 5
#define MS5611_COEFF_TEMPSENS 6

// osr index in command arrays
typedef enum {
	MS5611_OSR_256 = 0,
	MS5611_OSR_512 = 1,
	MS5611_OSR_1024 = 2,
	MS5611_OSR_2048 = 3,
	MS5611_OSR_4096 = 4
} ms5611_osr_t;

typedef enum {
	MS5611_CONV_IDLE = 0,
	MS5611_CONV_TEMP,
	MS5611_CONV_PRESSURE
} ms5611_conv_state_t;

typedef struct {
	/* Calibration coefficients read from PROM */
	uint16_t prom_coef[8]; /* C[1]..C[6] used; C[0] = factory reserved empty space */

	i2c_bus_t bus;
	ms5611_address_t addr;

	/* Selected OSR for pressure and temperature conversions */
	ms5611_osr_t osr_pressure;
	ms5611_osr_t osr_temperature;

	/* Set true once init succeeds */
	bool initialized;

	/* Async polling state machine */
	ms5611_conv_state_t conv_state;
	uint32_t conv_start_tick;
	uint32_t d2_raw;

	/* Result buffer — written by state machine in get_raw_pressure(), read by caller */
	ms5611_raw_result_t cached_result;
	uint32_t temp_cached_timestamp_ms; // stores timestamp before pressure has finished converting
	uint32_t cached_timestamp_ms;
	bool has_valid_result;
} ms5611_handle_t;

/* Conversion time in microseconds - max values from datasheet for safety */
static const uint32_t CONV_TIME_US[] = {
	600,
	/* OSR 256  — datasheet max 0.60ms */ // this one is picked for tempreture
	1170, /* OSR 512  — datasheet max 1.17ms */
	2280,
	/* OSR 1024 — datasheet max 2.28ms */ // this one is picked for pressure
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

static const int16_t second_comp_temp_threshold_centi_degrees =
	2000; /* temperature (in centidegrees) below which second-order compensation is applied */
static const int16_t second_comp_extreme_temp_threshold_centi_degrees =
	-1500; /* temperature (in centidegrees) below which additional extreme cold compensation is
			  applied */

static uint32_t conv_us_to_ms(uint32_t time_us) {
	return time_us / 1000;
}

// modify this struct to toggle barometer settings
static ms5611_handle_t handle = {.prom_coef = {0}, // will be populated by prom read
								 .bus = I2C_BUS_5,
								 .addr =
									 MS5611_ADDRESS_CSB_LOW, // according to canard board schematic,
															 // CSB is tied to GND, so addr is 0x77
								 .osr_pressure = MS5611_OSR_1024,
								 .osr_temperature = MS5611_OSR_256,
								 .initialized = false};

/**
 * @brief Reads one byte of data from the MS5611 sensor over I2C.
 * @param cmd The command to read from the sensor.
 */
static w_status_t baro_read(uint8_t reg, uint8_t *data, uint8_t len) {
	return i2c_read_reg(handle.bus, (uint8_t)handle.addr, reg, data, len);
}

/**
 * @brief Writes a command to the MS5611 sensor.
 * @param cmd The command to write.
 */
static w_status_t baro_write_cmd(uint8_t cmd) {
	return i2c_write_data(handle.bus, (uint8_t)handle.addr, &cmd, 1);
}

/**
 * @brief Reads the ADC value from the MS5611 sensor.
 * @param out Pointer to store the read ADC value.
 */
static w_status_t baro_read_adc(uint32_t *out) {
	uint8_t prom_buf[3];
	if (baro_read(MS5611_CMD_ADC_READ, prom_buf, 3) != W_SUCCESS) {
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
	n_prom[7] = (0xFF00U & crc_read); /* set crc read */

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
	uint8_t i;
	uint8_t prom_buf[2];
	uint16_t prom_coef[8];
	w_status_t status = W_SUCCESS;

	// copy into a local var first to avoid modifying the handle with corrupt data on read failure
	for (i = 0; i < 8; i++) {
		status = baro_read(MS5611_CMD_PROM_READ_BASE + (i * 2), prom_buf, 2);
		if (status != W_SUCCESS) {
			log_text(1, "ms5611", "ERROR: failed to read PROM coefficient C%u", i);
			return W_FAILURE;
		}
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

/* Applies first and second-order compensation to d1 using the stored d2_raw, and writes the result
 * into the cache. Also sets conv_state back to IDLE. */
static void apply_compensation(uint32_t d1) {
	int32_t dt = (int32_t)handle.d2_raw - (((int32_t)handle.prom_coef[MS5611_COEFF_TREF]) << 8);
	int32_t temp = second_comp_temp_threshold_centi_degrees +
				   (int32_t)(((int64_t)dt * handle.prom_coef[MS5611_COEFF_TEMPSENS]) >> 23);
	int64_t off = (((int64_t)handle.prom_coef[MS5611_COEFF_OFF]) << 16) +
				  ((((int64_t)handle.prom_coef[MS5611_COEFF_TCO]) * dt) >> 7);
	int64_t sens = (((int64_t)handle.prom_coef[MS5611_COEFF_SENS]) << 15) +
				   ((((int64_t)handle.prom_coef[MS5611_COEFF_TCS]) * dt) >> 8);

	int64_t T2 = 0, off2 = 0, sens2 = 0;

	if (temp < second_comp_temp_threshold_centi_degrees) {
		T2 = ((int64_t)dt * dt) >> 31;
		off2 = (5 * ((int64_t)(temp - second_comp_temp_threshold_centi_degrees) *
					 (temp - second_comp_temp_threshold_centi_degrees))) >>
			   1;
		sens2 = (5 * ((int64_t)(temp - second_comp_temp_threshold_centi_degrees) *
					  (temp - second_comp_temp_threshold_centi_degrees))) >>
				2;

		if (temp < second_comp_extreme_temp_threshold_centi_degrees) {
			off2 += 7 * ((int64_t)(temp - second_comp_extreme_temp_threshold_centi_degrees) *
						 (temp - second_comp_extreme_temp_threshold_centi_degrees));
			sens2 += (11 * ((int64_t)(temp - second_comp_extreme_temp_threshold_centi_degrees) *
							(temp - second_comp_extreme_temp_threshold_centi_degrees))) >>
					 1;
		}

		temp -= (int32_t)T2;
		off -= off2;
		sens -= sens2;
	}

	int32_t p = (int32_t)((((int64_t)d1 * sens >> 21) - off) >> 15);

	handle.cached_result.temperature_centideg = temp;
	handle.cached_result.pressure_centimbar = p;

	handle.has_valid_result = true;
}

/**
 * @brief Initialized the sensor by resetting it, reading calibration coeff from PROM, and cache
 * baro data so the getter always returns valid data
 * @note blocks for 7 ms
 */
w_status_t ms5611_init(void) {
	handle.initialized = false;

	if (W_SUCCESS != baro_write_cmd(MS5611_CMD_RESET)) {
		log_text(1, "ms5611", "ERROR: initialization failed during command reset.");
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(3)); // 3ms wait time from AN520 datasheet

	if (W_SUCCESS != ms5611_prom_read()) {
		log_text(1, "ms5611", "ERROR: initialization failed during PROM read .");
		return W_FAILURE;
	}

	handle.conv_state = MS5611_CONV_IDLE;
	handle.conv_start_tick = 0;
	handle.d2_raw = 0;
	handle.has_valid_result = false;

	/* Prime the result cache with one blocking measurement so the getter always returns valid data
	 */
	if (W_SUCCESS != baro_write_cmd(D2_CMD[handle.osr_temperature])) {
		log_text(1, "ms5611", "ERROR: initialization failed during temperature conversion");
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS((CONV_TIME_US[handle.osr_temperature] + 999) / 1000)); // 1 ms

	if (W_SUCCESS != baro_read_adc(&handle.d2_raw)) {
		log_text(1, "ms5611", "ERROR: initialization failed reading temperature ADC");
		return W_FAILURE;
	}

	// get time stamp at the start of pressure conversion for accuracy
	if (W_SUCCESS != timer_get_ms(&handle.cached_timestamp_ms)) {
		log_text(1, "ms5611", "ERROR: initialization failed getting timestamp");
		return W_FAILURE;
	}

	if (W_SUCCESS != baro_write_cmd(D1_CMD[handle.osr_pressure])) {
		log_text(1, "ms5611", "ERROR: initialization failed during pressure conversion");
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS((CONV_TIME_US[handle.osr_pressure] + 999) / 1000)); // 3 ms

	uint32_t d1_prime;

	if (W_SUCCESS != baro_read_adc(&d1_prime)) {
		log_text(1, "ms5611", "ERROR: initialization failed reading pressure ADC");
		return W_FAILURE;
	}

	handle.cached_timestamp_ms += conv_us_to_ms(CONV_TIME_US[handle.osr_pressure]) / 2;
	apply_compensation(d1_prime);

	handle.initialized = true;
	log_text(1, "ms5611", "INFO: initialization successful");

	return W_SUCCESS;
}

/**
 * @brief Deinitializes the driver by clearing the PROM coefficients and initialize handle
 *  This is useful for resetting the driver state between tests.
 */
void ms5611_deinit(void) {
	handle.initialized = false;
	handle.conv_state = MS5611_CONV_IDLE;
	handle.has_valid_result = false;

	for (size_t i = 0; i < 8; ++i) {
		handle.prom_coef[i] = 0;
	}
}

/**
 * @brief Reads raw pressure and temperature data from the MS5611 sensor, applies compensation, and
 * stores the results in the provided struct.
 * @param result Pointer to store the raw pressure and temperature results
 * @note Non-blocking: advances an internal conversion state machine on each call and returns the
 * last completed measurement from a buffer. The cache is primed in ms5611_init(), so this always
 * returns valid (potentially stale) data on success. Data is at most ~15 ms old at 200 Hz.
 * @note see MS5611 datasheet page 7 - 8 for details on the calculation
 * (temperature in centidegrees, pressure in centimbar)
 */
w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result, uint32_t *timestamp_ms) {
	/* d1 is raw pressure reading, d2 is raw temperature reading */
	uint32_t d1;
	uint32_t d2;
	/* dt is temperature difference, temp is compensated temperature, p is pressure */
	int32_t dt;
	uint32_t temp;
	uint32_t p;
	/* prom coefficients for first order */
	int64_t off;
	int64_t sens;
	/* second-order compensation terms */
	int64_t T2;
	int64_t off2;
	int64_t sens2;

	if (NULL == result || NULL == timestamp_ms) {
		log_text(1, "ms5611", "ERROR: NULL pointer passed to ms5611_get_pressure");
		return W_INVALID_PARAM;
	}

	if (!handle.initialized) {
		log_text(1, "ms5611", "ERROR: attempted to read pressure before successful initialization");
		return W_FAILURE;
	}

	switch (handle.conv_state) {
		case MS5611_CONV_IDLE: { // get temperature cmd sent
			if (W_SUCCESS != baro_write_cmd(D2_CMD[handle.osr_temperature])) {
				log_text(1, "ms5611", "ERROR: failed to write temperature conversion command");
				return W_IO_ERROR;
			}

			handle.conv_start_tick = (uint32_t)xTaskGetTickCount();
			handle.conv_state = MS5611_CONV_TEMP;

			break;
		}

		case MS5611_CONV_TEMP: { // temp adc conv - need at least 1ms after the first call. Get
								 // pressure cmd sent and time stamp acquired
			TickType_t required =
				pdMS_TO_TICKS((CONV_TIME_US[handle.osr_temperature] + 999) / 1000); // 1ms

			if (((uint32_t)xTaskGetTickCount() - handle.conv_start_tick) < (uint32_t)required) {
				break; /* conversion not ready yet */
			}

			if (W_SUCCESS != baro_read_adc(&handle.d2_raw)) {
				log_text(1, "ms5611", "ERROR: failed to read temperature ADC");
				handle.conv_state = MS5611_CONV_IDLE;
				return W_IO_ERROR;
			}

			if (W_SUCCESS != timer_get_ms(&handle.temp_cached_timestamp_ms)) {
				log_text(1, "ms5611", "ERROR: failed to get timestamp");
				handle.conv_state = MS5611_CONV_IDLE;
				return W_FAILURE;
			}

			handle.temp_cached_timestamp_ms += CONV_TIME_US[handle.osr_pressure] / 2000;

			if (W_SUCCESS != baro_write_cmd(D1_CMD[handle.osr_pressure])) {
				log_text(1, "ms5611", "ERROR: failed to write pressure conversion command");
				handle.conv_state = MS5611_CONV_IDLE;
				return W_IO_ERROR;
			}

			handle.conv_start_tick = (uint32_t)xTaskGetTickCount();
			handle.conv_state = MS5611_CONV_PRESSURE;

			break;
		}

		case MS5611_CONV_PRESSURE: { // pressure adc conv - at least 3ms after the second call.
			TickType_t required =
				pdMS_TO_TICKS((CONV_TIME_US[handle.osr_pressure] + 999) / 1000); // 3 ms

			if (((uint32_t)xTaskGetTickCount() - handle.conv_start_tick) < (uint32_t)required) {
				break; /* conversion not ready yet */
			}

			uint32_t d1;

			if (W_SUCCESS != baro_read_adc(&d1)) {
				log_text(1, "ms5611", "ERROR: failed to read pressure ADC");
				handle.conv_state = MS5611_CONV_IDLE;
				return W_IO_ERROR;
			}

			apply_compensation(d1); // new pressure and temp is re-assigned here

			handle.cached_timestamp_ms = handle.temp_cached_timestamp_ms;
			handle.conv_state = MS5611_CONV_IDLE;

			break;
		}

		default: {
			handle.conv_state = MS5611_CONV_IDLE;
			break;
		}
	}

	*result = handle.cached_result;
	*timestamp_ms = handle.cached_timestamp_ms;

	return W_SUCCESS;
}
