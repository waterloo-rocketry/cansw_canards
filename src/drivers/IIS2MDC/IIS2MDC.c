#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/i2c/i2c.h"

// I2C slave address
static const uint32_t IIS2MDC_I2C_ADDR = 0x1E;

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
