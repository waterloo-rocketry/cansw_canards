

#ifndef AD_BREAKOUT_BOARD_H
#define AD_BREAKOUT_BOARD_H

#include "rocketlib/include/common.h"
#include "drivers/altimu-10/altimu-10.h"
#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint32_t timestamp_imu;
	vector3d_t accelerometer; // TODO: based on the ADXL
	float z_rate; // rad/sec
    bool is_dead;
} ad_breakout_board_mesurement_t;

typedef struct __attribute__((packed)) {
	altimu_raw_imu_data_t accelerometer; // TODO: based on the ADXL
    // TODO: check if need the raw version of the data
	int32_t z_rate; // rad/sec
} ad_breakout_board_raw_mesurement_t;


/**
 * @brief initalize both the breakout board sensor drivers
 * @return the status of initalization
 */
w_status_t ad_beakout_board_init();

/**
 * @brief the FreeRTOS Task for getting the sensor data
 */
void ad_breakout_board_task(void *argument);

/**
 * @brief to read both the accelerometer and gyro data
 * @param data this is a pointer to converted data
 * @param raw_data pointer to raw data
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(ad_breakout_board_mesurement_t *data, altimu_raw_imu_data_t *raw_data);
#endif