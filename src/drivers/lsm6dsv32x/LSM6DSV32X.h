#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#include <stdint.h>

#include "rocketlib/include/common.h"

#include "application/health_checks/health_checks.h"
#include "common/math/math.h"

/**
 * raw data from imu/mag registers
 */
typedef struct __attribute__((packed)) {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint32_t timestamp_ms;
} lsm6dsv32x_raw_imu_data_t;

/**
 * @brief Initializes the bit registers for lsm6dsv32x
 * @note Must be called to wake up imu
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init(void);

/**
 * @brief ISR for the interrupt pin that begins DMA data transfer
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_int1_isr_handler(void);

/**
 * @brief Retrieves all 12 bytes of imu data
 * @param[out] acc_data    Processed accelerometer data (gravities)
 * @param[out] gyro_data   Processed gyroscope data (deg/s)
 * @param[out] raw_acc     Raw accelerometer data
 * @param[out] raw_gyro    Raw gyroscope data
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_get_gyro_acc_data(vector3d_t *acc_data, vector3d_t *gyro_data,
										lsm6dsv32x_raw_imu_data_t *raw_acc,
										lsm6dsv32x_raw_imu_data_t *raw_gyro);

/**
 * @brief Get and report the lsm6dsv32x status for the health check system
 *
 * Gets the lsm6dsv32x health status and logs relevant information.
 * Follows the module_get_status naming convention used by health_checks.
 *
 * @return CAN board status bitfield
 */
health_status_t lsm6dsv32x_get_status(void);

#endif
