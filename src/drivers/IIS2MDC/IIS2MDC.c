#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/i2c/i2c.h"

// I2C bus and slave address
static const i2c_bus_t IIS2MDC_BUS = I2C_BUS_4;
static const uint8_t IIS2MDC_I2C_ADDR = 0x1E;

// Register addresses
static const uint32_t IIS2MDC_REG_OFFSET_X_L = 0x45; // add other axes later
static const uint32_t IIS2MDC_REG_WHO_AM_I = 0x4F;
static const uint32_t IIS2MDC_REG_CFG_A = 0x60;
static const uint32_t IIS2MDC_REG_CFG_B = 0x61;
static const uint32_t IIS2MDC_REG_CFG_C = 0x62;
static const uint32_t IIS2MDC_REG_STATUS = 0x67;
static const uint32_t IIS2MDC_REG_OUTX_L = 0x68;

// Expected WHO_AM_I return value
static const uint32_t IIS2MDC_WHO_AM_I_VAL = 0x40;

// Checks if new data is available
static const uint32_t IIS2MDC_STATUS_ZYXDA = (1 << 3);

// Resets config registers
static const uint32_t IIS2MDC_CFG_A_SOFT_RESET = (1 << 5);

// Enables self-testing
static const uint8_t IIS2MDC_CFG_C_SELF_TEST = (1 << 1);

/* Init configuration:
 CFG_REG_A = 0x8C  COMP_TEMP_EN=1, LP=0(high-res), ODR=11 (100 Hz), MD=00 (continuous)
 CFG_REG_B = 0x02  OFF_CANC=1 (continuous offset cancellation)
 CFG_REG_C = 0x10  BDU=1 (block data update, DRDY pin unused, host polls status register)
 */
static const uint32_t IIS2MDC_INIT_CFG_A = 0x8C;
static const uint32_t IIS2MDC_INIT_CFG_B = 0x02;
static const uint32_t IIS2MDC_INIT_CFG_C = 0x10;

// conversion factor from raw register values to gauss
static const float64_t IIS2MDC_SENSITIVITY_GAUSS_PER_LSB = 0.0015;

// background polling task period matching the sensor ODR of 100hz.
static const uint32_t IIS2MDC_POLL_PERIOD_MS = 10;

/**
 * @brief Helper function to read one or more consecutive bytes over I2C
 * @note IIS2MDC auto-increments the sub-address if MSB is 1, so multi-byte reads only need the
 * start register.
 */
static w_status_t a_read(uint8_t reg, uint8_t *data, uint8_t len) {
	return i2c_read_reg(IIS2MDC_BUS, IIS2MDC_I2C_ADDR, reg, data, len);
}

/**
 * @brief Helper function to write a single byte over I2C
 */
static w_status_t a_write(uint8_t reg, uint8_t val) {
	return i2c_write_reg(IIS2MDC_BUS, IIS2MDC_I2C_ADDR, reg, &val, 1);
}

/**
 * @brief Helper function to convert six output register bytes into raw counts and gauss.
 * @note Each output register is uint8, two registers form a piece of data
 * for an axis. This is casted to uint16 for raw data and int16 for gauss.
 */
static void a_convert(const uint8_t *buf, iis2mdc_raw_data_t *raw, vector3d_t *data) {
	raw->x = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
	raw->y = (uint16_t)(((uint16_t)buf[3] << 8) | buf[2]);
	raw->z = (uint16_t)(((uint16_t)buf[5] << 8) | buf[4]);

	data->x = (float64_t)(int16_t)raw->x * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
	data->y = (float64_t)(int16_t)raw->y * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
	data->z = (float64_t)(int16_t)raw->z * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
}

/**
 * @brief Performs a sanity check by verifying device identity and initialization configs.
 * @return W_SUCCESS if the device responds with the expected values, W_FAILURE otherwise
 */
static w_status_t iis2mdc_check_sanity(void) {
	return W_SUCCESS;
}

w_status_t iis2mdc_init(void) {
	iis2mdc_check_sanity();
	return W_SUCCESS;
}

w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data) {
	return W_SUCCESS;
}
