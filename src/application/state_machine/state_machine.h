#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "rocketlib/include/common.h"

/**
 * unified format to pass sensor data between imu handler and estimator
 */
typedef struct {
	uint32_t gyro_rads; // etc
	vector3d_t stuff;
} all_sensors_data_t;

/**
 * run in 500 hz freertos task
 */
void state_machine_task(void *args);

#endif
