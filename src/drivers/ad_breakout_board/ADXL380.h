#ifndef ADXL380_H
#define ADXL380_H

#include "rocketlib/include/common.h"
#include "drivers/altimu-10/altimu-10.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief this is initializes the ADXL380
 * @return the status of the function call
 */
w_status_t adxl380_init();

/**
 * @brief this gets the raw acceleration data from the ADXL380
 * @param raw_x pointer to the raw x value
 * @param raw_y pointer to the raw y value
 * @param raw_z pointer to the raw z value
 * @return the status of the function call
 */
w_status_t adxl380_get_raw_accel(uint16_t *raw_x, uint16_t *raw_y, uint16_t *raw_z);

/**
 * @brief this gets the acceleration data (raw and processed) from the ADXL380
 * @param data pointer to the vector form of the acceleration
 * @param raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_accel_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data);

#endif