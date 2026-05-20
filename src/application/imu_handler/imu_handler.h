#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "common/gnc/gnc_types.h"
#include "drivers/altimu-10/altimu-10.h"

#include "application/health_checks/health_checks.h"
#include "rocketlib/include/common.h"

/**
 * raw data read from pololu device registers
 */
typedef struct __attribute__((packed)) {
	altimu_raw_imu_data_t raw_acc;
	altimu_raw_imu_data_t raw_gyro;
	altimu_raw_imu_data_t raw_mag;
	altimu_raw_baro_data_t raw_baro;
} raw_pololu_data_t;

/**
 * @brief Initialize the IMU handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void);

/**
 * @brief Curate fresh data from all sensors. This function executes instantly (non-blocking)
 * to avoid delaying the control loop.
 * @note !!! data that exceeds the sensor's freshness requirement is marked as dead !!!
 *
 * @param output pointer to update with the latest data
 * @return Status of the execution
 */
w_status_t imu_handler_get_fresh_meas(all_sensors_data_t *output);

/**
 * @brief Reports the current status of the IMU handler module
 * @return CAN board status bitfield
 * @details Logs initialization status, sampling statistics, and error conditions
 * for both IMUs being managed by the handler
 */
health_status_t imu_handler_get_status(void);

#endif // IMU_HANDLER_H
