#include "drivers/MS5611/MS5611.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include <limits.h>
#include <stdio.h>

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	return i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
}

w_status_t ms5611_init(void) {
	// TODO: Implement initialization sequence, including reading calibration coefficients from PROM
	w_status_t status = W_SUCCESS;
	// send reset command to clear old calibration data and reset sensor state
	// set tempreture osr
	// set pressure osr
	// set pin 2 to high to select i2c addr 0x77
	// read prom coefficients and store in struct
	// perform crc check on prom coefficients to ensure they were read correctly

	// check sanity??

	return status;
}

//?????
static w_status_t ms5611_check_sanity(void) {
	w_status_t status = W_SUCCESS;
	w_status_t i2c_status = W_SUCCESS;
	w_status_t device_status = W_SUCCESS;

	const uint8_t expected_MS5611 = MS5611_CMD_PROM_READ_BASE;

	uint8_t who_am_i;

	i2c_status |= i2c_read_reg(I2C_BUS_4, MS5611_ADDR, MS5611_CMD_PROM_READ_BASE, &who_am_i, 1);

	if (expected_MS5611 != who_am_i) {
		device_status |= W_FAILURE;
	}

	if ((i2c_status != W_SUCCESS) || (device_status != W_SUCCESS)) {
		return W_FAILURE;
	}
	return W_SUCCESS;
}

// ensure that the calibration coefficients stored in the PROM are read correctly by the
// microcontroller. reference repo in the design doc
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

	if (n_rem == 0) {
		return W_SUCCESS;
	} else {
		return W_FAILURE;
	}
}

static w_status_t ms5611_prom_read(uint32_t *adc_value) {
	// TODO: Implement reading the 24-bit ADC value from the sensor after conversion is complete
}

static w_status_t ms5611_read_pressure_and_temperature(double *pressure_pa, double *temperature_c) {
	// TODO: Implement the full sequence of starting conversions, waiting for completion, reading
	// ADC values, and applying calibration coefficients to compute actual pressure and temperature
}

w_status_t ms5611_get_data(double *pressure_pa, double *temperature_c) {
	// TODO: Implement a public function that calls ms5611_read_pressure_and_temperature and returns
	// the results in the desired units (e.g. Pa for pressure, °C for temperature)
}

static w_status_t pressure_compensate_temperature(double temperature_c,
												  double *compensated_pressure_pa) {
	// TODO: Implement temperature compensation for pressure reading based on the calibration
	// coefficients and raw temperature reading. This typically involves applying a formula that
	// uses the raw temperature to adjust the raw pressure reading to account for temperature
	// effects on the sensor's measurements.
}

static w_status_t pressure_compensate_pressure(double pressure_pa,
											   double *compensated_pressure_pa) {
	// TODO: Implement pressure compensation based on the calibration coefficients. This typically
	// involves applying a formula that uses the raw pressure reading and the calibration
	// coefficients to compute a compensated pressure value that accounts for sensor non-linearities
	// and other factors.
}

static w_status_t ms5611_run(void) {
	// TODO: Implement the main run function for the MS5611 sensor that will be called in the main
	// loop. This function should call ms5611_get_data to retrieve the latest pressure and
	// temperature readings, and then apply any necessary compensation before returning the final
	// values. It should also handle any errors that may occur during the reading process and return
	// appropriate status codes.
}