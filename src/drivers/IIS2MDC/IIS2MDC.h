#ifndef IIS2MDC_H
#define IIS2MDC_H

#include <stdint.h>

#include "common/math/math.h"
#include "rocketlib/include/common.h"

/**
 * @brief Raw magnetometer data for each axis.
 * @note Sensitivity is 1.5 mgauss/LSB, multiply raw by 0.0015 to convert to gauss.
 */
typedef struct {
	float64_t x;
	float64_t y;
	float64_t z;
} iis2mdc_raw_data_t;

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
w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data);

#endif // IIS2MDC_H
