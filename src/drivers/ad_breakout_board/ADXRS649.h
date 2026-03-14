#ifndef ADXRS649_H
#define ADXRS649_H

#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief initialize and start up the ADXRS649 AD Gyro and ADS1219
 * @return the status at which the ADXRS649 initalization goes
 */
w_status_t adxrs649_init();

/**
 * @brief get the spin rate data from the ADXRS649 Gyro
 * @param data is a pointer to where the data will be stored (deg/sec)
 * @return the status of the get data function
 */
w_status_t adxrs649_get_gyro_data(float *data);
#endif