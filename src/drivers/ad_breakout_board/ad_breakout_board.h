#ifndef AD_BREAKOUT_BOARD_H
#define AD_BREAKOUT_BOARD_H

#include <stdint.h>

#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "rocketlib/include/common.h"

typedef struct {
	uint32_t timestamp;
	float64_t z_rate; // rad/sec
	bool is_dead;
} ad_gyro_mesurement_t;

typedef struct {
	uint32_t timestamp;
	vector3d_t accelerometer; // TODO: based on the ADXL
	bool is_dead;
} ad_accelerometer_mesurement_t;

typedef struct {
	adxl380_raw_accel_data_t accelerometer; // TODO: based on the ADXL
} ad_accelerometer_raw_mesurement_t;

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
 * @param g_gyro_data this is a pointer to converted gyro data
 * @param g_accel_data pointer to state of converted accel data
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(ad_gyro_mesurement_t *g_gyro_data,
									  ad_accelerometer_mesurement_t *g_accel_data);
#endif
