#ifndef IIS2MDC_H
#define IIS2MDC_H

#include "rocketlib/include/common.h"
#include <stdint.h>

// I2C slave address
#define IIS2MDC_I2C_ADDR 0x1E

// Expected WHO_AM_I return value
#define IIS2MDC_WHO_AM_I_VAL 0x40

// Checks status register to see if new data is available
#define IIS2MDC_STATUS_ZYXDA (1 << 3)

// Resets config registers
#define IIS2MDC_CFG_A_SOFT_RESET (1 << 5)

/* Init configuration:
 CFG_REG_A = 0x8C  COMP_TEMP_EN=1, LP=0(high-res), ODR=11 (100 Hz), MD=00 (continuous)
 CFG_REG_B = 0x02  OFF_CANC=1 (continuous offset cancellation)
 CFG_REG_C = 0x10  BDU=1 (block data update, DRDY pin unused, host polls status register)
 */
#define IIS2MDC_INIT_CFG_A 0x8C
#define IIS2MDC_INIT_CFG_B 0x02
#define IIS2MDC_INIT_CFG_C 0x10

// conversion factor from raw register values to gauss
const float IIS2MDC_SENSITIVITY_GAUSS_PER_LSB = 0.0015;

/**
 * @brief Raw magnetometer data for each axis.
 * @note Sensitivity is 1.5 mgauss/LSB, multiply raw by 0.0015 to convert to gauss.
 */
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} iis2mdc_raw_data_t;

/**
 * @brief Converted magnetometer data for each axis in gauss.
 */
typedef struct {
	float x;
	float y;
	float z;
} iis2mdc_data_t;

/**
 * @brief Initializes the IIS2MDC magnetometer.
 * @note Performs a soft reset, verifies WHO_AM_I, and applies configs.
 * @return Status of the operation
 */
w_status_t iis2mdc_init(void);

/**
 * @brief Performs a sanity check by verifying device identity and initialization configs.
 * @return W_SUCCESS if the device responds with the expected values, W_FAILURE otherwise
 */
w_status_t iis2mdc_check_sanity(void);

/**
 * @brief Retrieves magnetic field reading for all three axes.
 * @note Polls STATUS_REG until a new sample is available then reads output registers
 * @param[out] data Pointer to store the converted magnetic field values
 * @param[out] raw_data Pointer to store the raw magnetometer readings
 * @return Status of the operation, timeout if no data arrived in time.
 */
w_status_t iis2mdc_get_data(iis2mdc_data_t *data, iis2mdc_raw_data_t *raw_data);

#endif // IIS2MDC_H
