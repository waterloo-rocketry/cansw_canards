#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/logger/log.h"
#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/i2c/i2c.h"
#include "drivers/timer/timer.h"

// I2C bus and slave address
static const i2c_bus_t IIS2MDC_BUS = I2C_BUS_4;
static const uint8_t IIS2MDC_I2C_ADDR = 0x1E;

// Register addresses
// TODO: Calibration needs to be done in a magnetic field to get offset values written into the
// offset registers.
static const uint32_t IIS2MDC_REG_OFFSET_X_L = 0x45;
static const uint32_t IIS2MDC_REG_WHO_AM_I = 0x4F;
static const uint32_t IIS2MDC_REG_CFG_A = 0x60;
static const uint32_t IIS2MDC_REG_CFG_B = 0x61;
static const uint32_t IIS2MDC_REG_CFG_C = 0x62;
static const uint32_t IIS2MDC_REG_STATUS = 0x67;
static const uint32_t IIS2MDC_REG_OUTX_L = 0x68;

// SUB MSB that enables auto-increment for multi-byte reads. Since all registers are less than 0x80
// we can freely use this bit
static const uint32_t IIS2MDC_SUB_AUTO_INC = 0x80;

// Expected WHO_AM_I return value
static const uint32_t IIS2MDC_WHO_AM_I_VAL = 0x40;

// Checks if new data is available
static const uint32_t IIS2MDC_STATUS_ZYXDA = (1 << 3);

// Resets config registers
static const uint32_t IIS2MDC_CFG_A_SOFT_RESET = (1 << 5);

// Enables self-testing
static const uint8_t IIS2MDC_CFG_C_SELF_TEST = (1 << 1);

// Self-test parameters (AN5080)
static const float64_t IIS2MDC_SELF_TEST_MIN_GAUSS = 0.015;
static const float64_t IIS2MDC_SELF_TEST_MAX_GAUSS = 0.500;
static const uint32_t IIS2MDC_SELF_TEST_SAMPLES = 50;
static const uint32_t IIS2MDC_SELF_TEST_POWERUP_MS = 20;
static const uint32_t IIS2MDC_SELF_TEST_SETTLE_MS = 60;
static const uint32_t IIS2MDC_SELF_TEST_TIMEOUT_MS = 20;
static const uint32_t IIS2MDC_SELF_TEST_POLLING_PERIOD_MS = 1;

/* Init configuration:
 CFG_REG_A = 0x8C  COMP_TEMP_EN=1, LP=0(high-res), ODR=11 (100 Hz), MD=00 (continuous)
 CFG_REG_B = 0x02  OFF_CANC=1 (continuous offset cancellation)
 CFG_REG_C = 0x10  BDU=1 (not sure if this is still needed with the current design scope?)
 */
static const uint32_t IIS2MDC_INIT_CFG_A = 0x8C;
static const uint32_t IIS2MDC_INIT_CFG_B = 0x02;
static const uint32_t IIS2MDC_INIT_CFG_C = 0x10;

// conversion factor from raw register values to gauss
static const float64_t IIS2MDC_SENSITIVITY_GAUSS_PER_LSB = 0.0015;

/**
 * @brief Helper function to read one or more consecutive bytes over I2C
 * @note IIS2MDC auto-increments the sub-address if MSB is 1, so multi-byte reads only need the
 * start register.
 */
static w_status_t iis2mdc_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
	if (len > 1) {
		reg |= IIS2MDC_SUB_AUTO_INC;
	}
	return i2c_read_reg(IIS2MDC_BUS, IIS2MDC_I2C_ADDR, reg, data, len);
}

/**
 * @brief Helper function to write a single byte over I2C
 */
static w_status_t iis2mdc_write_reg(uint8_t reg, uint8_t val) {
	return i2c_write_reg(IIS2MDC_BUS, IIS2MDC_I2C_ADDR, reg, &val, 1);
}

/**
 * @brief Helper function to convert six output register bytes into raw counts and gauss.
 * @note Each output register is uint8, two registers form a piece of data
 * for an axis. This is casted to uint16 for raw data and int16 for gauss.
 */
static void iis2mdc_convert_sample(const uint8_t *buf, iis2mdc_raw_data_t *raw, vector3d_t *data) {
	raw->x = (uint16_t)((((uint16_t)buf[1]) << 8) | buf[0]);
	raw->y = (uint16_t)((((uint16_t)buf[3]) << 8) | buf[2]);
	raw->z = (uint16_t)((((uint16_t)buf[5]) << 8) | buf[4]);

	data->x = ((float64_t)((int16_t)raw->x)) * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
	data->y = ((float64_t)((int16_t)raw->y)) * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
	data->z = ((float64_t)((int16_t)raw->z)) * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
}

/**
 * @brief Polls status register until a new sample is ready (ZYXDA), waits 1ms between polls
 * @note Bounded by IIS2MDC_SELF_TEST_TIMEOUT_MS (20ms, 2 sample period at 100hz)
 */
static w_status_t self_test_wait_data_ready(void) {
	uint8_t status = 0;
	uint32_t start_ms = 0;

	if (W_SUCCESS != timer_get_ms(&start_ms)) {
		return W_FAILURE;
	}

	uint32_t now_ms = start_ms;

	// within this loop, it checks if new data is ready by reading the status register every 1ms. If
	// data is not ready, updates the current time. Loop exits if the time elapsed exceeds the
	// timeout threshold, returning W_IO_TIMEOUT to indicate no new data was available during the
	// timeout period.
	while ((now_ms - start_ms) < IIS2MDC_SELF_TEST_TIMEOUT_MS) {
		if (W_SUCCESS != iis2mdc_read_reg(IIS2MDC_REG_STATUS, &status, 1)) {
			return W_FAILURE;
		}
		// ZYXDA is bit 3 of STATUS_REG, it is 1 when a fresh sample is available.
		// ANDing status with IIS2MDC_STATUS_ZYXDA (1 << 3) masks off every other bit, leaving
		// only bit 3. Comparing that masked value against IIS2MDC_STATUS_ZYXDA is true only when
		// bit 3 is set, so data_ready is 1 when a sample is ready and 0 when it is not.
		if ((status & IIS2MDC_STATUS_ZYXDA) == IIS2MDC_STATUS_ZYXDA) {
			return W_SUCCESS;
		}
		vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SELF_TEST_POLLING_PERIOD_MS));

		if (W_SUCCESS != timer_get_ms(&now_ms)) {
			return W_FAILURE;
		}
	}
	return W_IO_TIMEOUT; // no new sample within the timeout
}

/**
 * @brief Waits for a fresh sample, then reads and converts the output registers to gauss.
 * @note Used for self-test only, not intended for regular data reads
 */
static w_status_t self_test_read_sample(vector3d_t *out) {
	uint8_t buf[6];
	iis2mdc_raw_data_t raw;

	if (W_SUCCESS != self_test_wait_data_ready()) {
		return W_FAILURE;
	}
	if (W_SUCCESS != iis2mdc_read_reg(IIS2MDC_REG_OUTX_L, buf, 6)) {
		return W_FAILURE;
	}
	iis2mdc_convert_sample(buf, &raw, out);
	return W_SUCCESS;
}

/**
 * @brief Discards the first sample, then averages 50 data samples
 */
static w_status_t self_test_collect_average(vector3d_t *avg) {
	vector3d_t sample;
	float64_t sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

	// discard the first sample after a mode change
	if (W_SUCCESS != self_test_read_sample(&sample)) {
		return W_FAILURE;
	}

	for (uint32_t i = 0; i < IIS2MDC_SELF_TEST_SAMPLES; i++) {
		if (W_SUCCESS != self_test_read_sample(&sample)) {
			return W_FAILURE;
		}
		sum_x += sample.x;
		sum_y += sample.y;
		sum_z += sample.z;
	}

	avg->x = sum_x / ((float64_t)IIS2MDC_SELF_TEST_SAMPLES);
	avg->y = sum_y / ((float64_t)IIS2MDC_SELF_TEST_SAMPLES);
	avg->z = sum_z / ((float64_t)IIS2MDC_SELF_TEST_SAMPLES);
	return W_SUCCESS;
}

/**
 * @brief Runs the on-chip self-test (ST AN5080 document for procedure)
 * @note Waits for turn-on, averages the field with self-test off, then on, and checks the
 *       delta against the datasheet range (15-500 mgauss) Restores normal config before returning.
 */
static w_status_t iis2mdc_self_test(void) {
	vector3d_t avg_off, avg_on;

	// discard the first sample, then average with self-test disabled
	if (W_SUCCESS != self_test_collect_average(&avg_off)) {
		log_text(1, "iis2mdc", "ERROR: self-test baseline read failed");
		return W_FAILURE;
	}

	// enable self-test and wait 60ms for field to settle (specified in AN 5080)
	if (W_SUCCESS !=
		iis2mdc_write_reg(IIS2MDC_REG_CFG_C, IIS2MDC_INIT_CFG_C | IIS2MDC_CFG_C_SELF_TEST)) {
		log_text(1, "iis2mdc", "ERROR: failed to enable self-test");
		return W_FAILURE;
	}
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SELF_TEST_SETTLE_MS));

	w_status_t read_status = self_test_collect_average(&avg_on);

	// restore normal config regardless of the read outcome
	if (W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_C, IIS2MDC_INIT_CFG_C)) {
		log_text(1, "iis2mdc", "ERROR: failed to restore configs after self-test");
		return W_FAILURE;
	}
	if (W_SUCCESS != read_status) {
		log_text(1, "iis2mdc", "ERROR: self-test read failed");
		return W_FAILURE;
	}

	// per-axis delta
	float64_t dx = fabs(avg_on.x - avg_off.x);
	float64_t dy = fabs(avg_on.y - avg_off.y);
	float64_t dz = fabs(avg_on.z - avg_off.z);

	if ((dx < IIS2MDC_SELF_TEST_MIN_GAUSS) || (dx > IIS2MDC_SELF_TEST_MAX_GAUSS) ||
		(dy < IIS2MDC_SELF_TEST_MIN_GAUSS) || (dy > IIS2MDC_SELF_TEST_MAX_GAUSS) ||
		(dz < IIS2MDC_SELF_TEST_MIN_GAUSS) || (dz > IIS2MDC_SELF_TEST_MAX_GAUSS)) {
		log_text(1, "iis2mdc", "ERROR: self-test out of range: x=%f y=%f z=%f", dx, dy, dz);
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Performs a sanity check by verifying device identity and running a self test
 * @return W_SUCCESS if the device passes self test and identity verification
 */
static w_status_t iis2mdc_sanity_check(void) {
	uint8_t id;

	if (W_SUCCESS != iis2mdc_read_reg(IIS2MDC_REG_WHO_AM_I, &id, 1)) {
		log_text(1, "iis2mdc", "ERROR: failed to read WHO_AM_I");
		return W_FAILURE;
	}
	if (IIS2MDC_WHO_AM_I_VAL != id) {
		log_text(1,
				 "iis2mdc",
				 "ERROR: WHO_AM_I mismatch: expected %u, got %u",
				 IIS2MDC_WHO_AM_I_VAL,
				 id);
		return W_FAILURE;
	}

	if (W_SUCCESS != iis2mdc_self_test()) {
		log_text(1, "iis2mdc", "ERROR: self-test failed");
		return W_FAILURE;
	}

	return W_SUCCESS;
}

w_status_t iis2mdc_init(void) {
	// wait for stable output after power-up before any access to registers
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SELF_TEST_POWERUP_MS));

	// soft reset clears config registers
	if (W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_A, IIS2MDC_CFG_A_SOFT_RESET)) {
		log_text(1, "iis2mdc", "ERROR: soft reset failed");
		return W_FAILURE;
	}

	// wait before writing to registers again after soft reset
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SELF_TEST_POWERUP_MS));

	if ((W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_A, IIS2MDC_INIT_CFG_A)) ||
		(W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_B, IIS2MDC_INIT_CFG_B)) ||
		(W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_C, IIS2MDC_INIT_CFG_C))) {
		log_text(1, "iis2mdc", "ERROR: failed to write configuration registers");
		return W_FAILURE;
	}

	if (W_SUCCESS != iis2mdc_sanity_check()) {
		log_text(1, "iis2mdc", "ERROR: sanity check failed");
		return W_FAILURE;
	}

	return W_SUCCESS;
}

// with BDU enabled (CFG_REG_C) data corruption per-axis is avoided but it is still possible for
// cross-axis data corruption. In the future async implementation using the interrupt pin, this
// should not be a problem.

// In the current design scope using synchronous polling reads, since the polling rate is 200hz and
// the maximum ODR of the mag is 100hz, stale data is returned if no new data is available instead
// of returning timeout or failure.
w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data,
							uint32_t *timestamp_ms) {
	uint8_t buf[6] = {0};

	if ((NULL == data) || (NULL == raw_data) || (NULL == timestamp_ms)) {
		log_text(1, "iis2mdc", "ERROR: NULL pointer cannot be used as input to get_data function");
		return W_FAILURE;
	}

	if (W_SUCCESS != iis2mdc_read_reg(IIS2MDC_REG_OUTX_L, buf, 6)) {
		log_text(1, "iis2mdc", "ERROR: failed to read output registers");
		return W_FAILURE;
	}

	iis2mdc_convert_sample(buf, raw_data, data);

	if (W_SUCCESS != timer_get_ms(timestamp_ms)) {
		log_text(1, "iis2mdc", "ERROR: failed to get timestamp");
		return W_FAILURE;
	}
	return W_SUCCESS;
}
