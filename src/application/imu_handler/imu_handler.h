#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "drivers/altimu-10/altimu-10.h"
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

/**
 * @brief Public function to get the latest IMU data for use by the flight phase
 * @param all_imu_data Pointer to store the output data
 * @return Status of the execution
 */
w_status_t imu_handler_get_data_for_flight_phase(estimator_all_imus_input_t *all_imu_data);

#endif // IMU_HANDLER_H
