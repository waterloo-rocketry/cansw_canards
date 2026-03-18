#include "rocketlib/include/common.h"

#ifndef AD_BREAKOUT_BOARD_H
#define AD_BREAKOUT_BOARD_H

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
 * @param data this is a pointer to which the sensor data is stored
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(data_type *data);
#endif