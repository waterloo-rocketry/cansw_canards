#ifndef ADXL380_H
#define ADXL380_H

#include "drivers/altimu-10/altimu-10.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} adxl380_raw_accel_data_t;

/**
 * @brief this is initializes the ADXL380
 * @return the status of the function call
 */
w_status_t adxl380_init();

/**
 * @brief this gets the raw acceleration data from the ADXL380
 * @param p_raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_raw_accel(adxl380_raw_accel_data_t *p_raw_data);

/**
 * @brief this gets the acceleration data (raw and processed) from the ADXL380
 * @param data pointer to the vector form of the acceleration
 * @param p_raw_data pointer to all of the raw data for each access
 * @return the status of the function call
 */
w_status_t adxl380_get_accel_data(vector3d_t *data, adxl380_raw_accel_data_t *p_raw_data);

#endif