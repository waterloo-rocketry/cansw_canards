#include "drivers/MS5611/MS5611.h"
#include "drivers/timer/timer.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Period between successive samples taken by ms5611_task. The blocking conversion reads themselves
 * take a few ms; this is the idle gap added on top. */
#define MS5611_TASK_PERIOD_MS 10

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
} ms5611_handle_t;

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

static const int32_t second_comp_temp_threshold_centi_degrees =
	2000; /* temperature (in centidegrees) below which second-order compensation is applied */
static const int32_t second_comp_extreme_temp_threshold_centi_degrees =
	-1500; /* temperature (in centidegrees) below which additional extreme cold compensation is
			  applied */
static const uint32_t RESET_WAIT_TIME_MS = 3;

static uint32_t conv_us_to_ms(uint32_t time_us) {
	return time_us / 1000;
}

/* Cache of the latest sample, written by ms5611_task and read by ms5611_get_raw_pressure.
 * Protected by s_data_mutex. */
static SemaphoreHandle_t s_data_mutex = NULL;
static ms5611_raw_result_t s_latest_result = {0};
static uint32_t s_latest_timestamp_ms = 0;
static w_status_t s_latest_status = W_FAILURE; /* status of the most recent read attempt */

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

/**
 * @brief Initialized the sensor by resetting it, reading calibration coeff from PROM, and perform
 * crc check on the coefficients.
 */
w_status_t ms5611_init(void) {
	// make sure can't reinitialize the driver
	if (handle.initialized) {
		log_text(1, "ms5611", "ERROR: attempted to reinitialize driver.");
		return W_FAILURE;
	}

	if (NULL == s_data_mutex) {
		s_data_mutex = xSemaphoreCreateMutex();
		if (NULL == s_data_mutex) {
			log_text(1, "ms5611", "ERROR: failed to create data mutex");
			return W_FAILURE;
		}
	}

	if (W_SUCCESS != baro_write_cmd(MS5611_CMD_RESET)) {
		log_text(1, "ms5611", "ERROR: initialization failed during command reset.");
		return W_IO_ERROR;
	}

	vTaskDelay(pdMS_TO_TICKS(RESET_WAIT_TIME_MS)); // 3ms wait time from AN520 datasheet

	if (W_SUCCESS != ms5611_prom_read()) {
		log_text(1, "ms5611", "ERROR: initialization failed during PROM read .");
		return W_FAILURE;
	}

	handle.initialized = true;
	log_text(1, "ms5611", "INFO: initialization successful");

	return W_SUCCESS;
}

/**
 * @brief Deinitializes the driver by clearing the PROM coefficients and setting initialized to
 * false. This is useful for resetting the driver state between tests.
 */
void ms5611_deinit(void) {
	if (!(handle.initialized)) {
		log_text(1,
				 "ms5611",
				 "ERROR: attempted to uninitialize the driver before successful initialization");
		return;
	}

	handle.initialized = false;
	for (size_t i = 0; i < 8; ++i) {
		handle.prom_coef[i] = 0;
	}

	s_latest_status = W_FAILURE;
	s_latest_timestamp_ms = 0;
}

/**
 * @brief Reads raw pressure and temperature data from the MS5611 sensor, applies compensation, and
 * stores the results in the provided struct.
 * @param result Pointer to store the raw pressure and temperature results
 * @note this function also applies the compensation algorithm, but does not convert units
 * @note see MS5611 datasheet page 7 - 8 for details on the calculation
 * (temperature in centidegrees prom_coef, pressure in centimbar)
 * @note This performs blocking I2C transactions and conversion delays. It is called only from
 * ms5611_task; external callers use the non-blocking ms5611_get_raw_pressure instead.
 */
static w_status_t ms5611_read_raw_pressure(ms5611_raw_result_t *result, uint32_t *timestamp_ms) {
	/* d1 is raw pressure reading, d2 is raw temperature reading */
	uint32_t d1;
	uint32_t d2;
	/* dt is temperature difference, temp is compensated temperature, p is pressure */
	int32_t dt;
	int32_t temp;
	uint32_t p;
	/* prom coefficients for first order */
	int64_t off;
	int64_t sens;
	/* second-order compensation terms */
	int64_t T2;
	int64_t off2;
	int64_t sens2;

	if (NULL == result) {
		log_text(1, "ms5611", "ERROR: NULL pointer passed to ms5611_get_pressure");
		return W_INVALID_PARAM;
	}

	if (!(handle.initialized)) {
		log_text(1, "ms5611", "ERROR: attempted to read pressure before successful initialization");
		return W_FAILURE;
	}

	/* D2: temperature conversion */
	if (W_FAILURE == baro_write_cmd(D2_CMD[handle.osr_temperature])) {
		log_text(1, "ms5611", "ERROR: failed to write temperature conversion command");
		return W_IO_ERROR;
	}

	delay_us(2); // 600 us

	if (W_FAILURE == baro_read_adc(&d2)) {
		log_text(1, "ms5611", "ERROR: failed to read temperature ADC");
		return W_IO_ERROR;
	}

	if (W_SUCCESS != timer_get_ms(timestamp_ms)) {
		log_text(1, "ms5611", "ERROR: failed to get timestamp");
		return W_FAILURE;
	}

	/* D1: pressure conversion */
	if (W_FAILURE == baro_write_cmd(D1_CMD[handle.osr_pressure])) {
		log_text(1, "ms5611", "ERROR: failed to write pressure conversion command");
		return W_IO_ERROR;
	}

	delay_us(CONV_TIME_US[handle.osr_pressure]); // 2280 us

	if (W_FAILURE == baro_read_adc(&d1)) {
		log_text(1, "ms5611", "ERROR: failed to read pressure ADC");
		return W_IO_ERROR;
	}

	/* First-order compensation */
	dt = (int32_t)d2 - (((int32_t)handle.prom_coef[MS5611_COEFF_TREF]) << 8);
	temp = second_comp_temp_threshold_centi_degrees +
		   (int32_t)(((int64_t)dt * handle.prom_coef[MS5611_COEFF_TEMPSENS]) >> 23);
	off = (((int64_t)handle.prom_coef[MS5611_COEFF_OFF]) << 16) +
		  ((((int64_t)handle.prom_coef[MS5611_COEFF_TCO]) * dt) >> 7);
	sens = (((int64_t)handle.prom_coef[MS5611_COEFF_SENS]) << 15) +
		   ((((int64_t)handle.prom_coef[MS5611_COEFF_TCS]) * dt) >> 8);

	/* Second-order cold compensation */
	T2 = 0;
	off2 = 0;
	sens2 = 0;

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

		temp -= T2;
		off -= off2;
		sens -= sens2;
	}

	p = (int32_t)((((int64_t)d1 * sens >> 21) - off) >> 15);

	result->temperature_centideg = temp;
	result->pressure_centimbar = (int32_t)p;

	*timestamp_ms += (conv_us_to_ms(CONV_TIME_US[handle.osr_pressure]) / 2);

	return W_SUCCESS;
}

/**
 * @brief Non-blocking getter. Returns the most recent sample cached by ms5611_task without
 * performing any I2C or conversion delay.
 */
w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result, uint32_t *timestamp_ms) {
	if (NULL == result || NULL == timestamp_ms) {
		log_text(1, "ms5611", "ERROR: NULL pointer passed to ms5611_get_raw_pressure");
		return W_INVALID_PARAM;
	}

	if ((!(handle.initialized))|| (NULL == s_data_mutex)) {
		return W_FAILURE;
	}

	w_status_t status = W_FAILURE;

	if (pdTRUE == xSemaphoreTake(s_data_mutex, 0)) {
		status = s_latest_status;
		if (W_SUCCESS == status) {
			*result = s_latest_result;
			*timestamp_ms = s_latest_timestamp_ms;
		}
		xSemaphoreGive(s_data_mutex);
	}

	return status;
}

/**
 * @brief FreeRTOS task that periodically performs the blocking read and caches the latest result.
 */
void ms5611_task(void *argument) {
	(void)argument;

	ms5611_raw_result_t result;
	TickType_t timestamp_ms;

	while (1) {
		w_status_t status = ms5611_read_raw_pressure(&result, &timestamp_ms);
		TickType_t xLastWakeTime = xTaskGetTickCount();

		if (pdTRUE == xSemaphoreTake(s_data_mutex, portMAX_DELAY)) {
			s_latest_status = status;
			if (W_SUCCESS == status) {
				s_latest_result = result;
				s_latest_timestamp_ms = timestamp_ms;
			}
			xSemaphoreGive(s_data_mutex);
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MS5611_TASK_PERIOD_MS));
	}
}
