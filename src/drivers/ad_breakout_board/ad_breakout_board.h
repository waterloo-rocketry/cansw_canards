#ifndef AD_BREAKOUT_BOARD_H
#define AD_BREAKOUT_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "common/math/math.h"
#include "drivers/ad_breakout_board/ADXL380.h"
#include "rocketlib/include/common.h"

/**
 * @brief the FreeRTOS Task for getting the sensor data
 */
void ad_breakout_board_task(void *argument);

/**
 * @brief to read both the accelerometer and gyro data
 * @param p_gyro_data this is a pointer to converted gyro data
 * @param p_accel_data pointer to state of converted accel data
 * @param p_gyro_timestamp_ms the the timestamp at which the data was collected for gyro
 * @param p_accel_timestamp_ms the the timestamp at which the data was collected for accel
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(navigator_1d_meas_t *p_gyro_data,
									  navigator_3d_meas_t *p_accel_data,
									  uint32_t *p_gyro_timestamp_ms,
									  uint32_t *p_accel_timestamp_ms);

#endif
