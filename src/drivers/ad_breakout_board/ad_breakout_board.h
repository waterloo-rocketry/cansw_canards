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
 * @brief to read both the accelerometer data
 * @param p_accel_data this is a pointer to converted accelerometer data
 * @param timestamp_ms return pointer for the data timestamp
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_accel_data(vector3d_t *p_accel_data, uint32_t *timestamp_ms);

/**
 * @brief to read both the gyro data
 * @param p_gyro_data this is a pointer to converted gyro data
 * @param timestamp_ms return pointer for the data timestamp
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_gyro_data(float64_t *p_gyro_data, uint32_t *timestamp_ms);

#endif
