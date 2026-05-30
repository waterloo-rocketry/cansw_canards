#ifndef IIS2MDC_H
#define IIS2MDC_H

#include <stdint.h>

#include "common/math/math.h"
#include "rocketlib/include/common.h"

/**
 * @brief Raw magnetometer register data for each axis
 * @note Sensitivity is 1.5 mgauss/LSB, multiply raw by 0.0015 to convert to gauss
 */
typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} iis2mdc_raw_data_t;

/**
 * @brief Initializes the IIS2MDC magnetometer.
 * @note Performs a soft reset, applies the configuration registers, verifies WHO_AM_I, and
 * runs the on-chip self-test. Blocks for ~100 ms during the self-test.
 * @return Status of the operation
 */
w_status_t iis2mdc_init(void);

/**
 * @brief Reads the magnetometer's output registers and converts to gauss
 * @param[out] data Pointer to store the converted magnetic field values in gauss
 * @param[out] raw_data Pointer to store the raw unsigned 16-bit counts
 * @param[out] timestamp_ms Pointer to store the read timestamp (ms since startup)
 * @return W_SUCCESS on successful read and conversion of most recent sample
 */
w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data, uint32_t *timestamp_ms);

#endif // IIS2MDC_H
