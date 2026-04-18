#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "drivers/altimu-10/altimu-10.h"
#include "rocketlib/include/common.h"

/**
 * @brief Initialize the IMU handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void);

/**
 * @brief IMU handler task function for FreeRTOS
 * Should be created during system startup
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument);

/**
 * @brief Reports the current status of the IMU handler module
 * @return CAN board status bitfield
 * @details Logs initialization status, sampling statistics, and error conditions
 * for both IMUs being managed by the handler
 */
uint32_t imu_handler_get_status(void);

#endif // IMU_HANDLER_H
