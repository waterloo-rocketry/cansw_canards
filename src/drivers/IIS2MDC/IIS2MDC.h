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
 * @note Performs a soft reset, verifies WHO_AM_I, and applies configs.
 * @return Status of the operation
 */
w_status_t iis2mdc_init(void);

/**
 * @brief Retrieves magnetic field reading for all three axes.
 * @note Non-blocking: checks STATUS_REG.Zyxda once and returns W_FAILURE immediately if
 *       no new data is available. On success, reads six output registers
 *       and converts raw counts to gauss.
 * @param[out] data Pointer to store the converted magnetic field values in gauss
 * @param[out] raw_data Pointer to store the raw unsigned 16-bit register values
 * @return W_SUCCESS if a new sample was read, W_FAILURE if not (should probably distinguish
 * within failure cases later)
 */
w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data);

#endif // IIS2MDC_H
