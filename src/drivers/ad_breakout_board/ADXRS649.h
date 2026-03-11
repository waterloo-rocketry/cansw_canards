#ifndef ADXRS649_H
#define ADXRS649_H

#include "rocketlib/include/common.h"

/**
 * @brief initialize and start up the ADXRS649 AD Gyro and ADS1219
 * @return the status at which the ADXRS649 initalization goes
 */
w_status_t adxrs649_init();

#endif