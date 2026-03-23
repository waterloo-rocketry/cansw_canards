#include "application/logger/log.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/imus/LSM6DSV32X_regmap.h"
#include <limits.h>
#include <stdio.h>

enum {
	IMU_READ_BUFFER = 0,
	IMU_WRITE_BUFFER = 1,

	IMU_DATA_STALE = 1,
	IMU_DATA_READY = 0
};

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t dev_addr;
	uint8_t dual_buffer[2][12];
	TaskHandle_t task_to_notify;
	volatile bool stale_data;
	float timestamp[2];

} imu_ctx_t;

/**
 * raw data from imu/mag registers
 */
typedef struct __attribute__((packed)) {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	float timestamp;
} lsm6dsv32x_raw_imu_data_t;

/**
 * @brief Sainity Checks the IMU
 * @note only checks the WHO am I register
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_check_sanity();

/**
 * @brief Initializes the bit registers for lsm6dsv32x
 * @note Must be called to wake up imu
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init();

/**
 * @brief ISR for the interupt pin that begins DMA data transfor
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_int1_isr_handler();

/**
 * @brief Retrives all 12 bytes of imu data
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
 * @brief Retrives all 6 bytes of gyro
 * @param[out] gyro_data   Processed gyroscope data (deg/s)
 * @param[out] raw_gyro    Raw gyroscope data
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_get_gyro_data(vector3d_t *gyro_data, lsm6dsv32x_raw_imu_data_t *raw_gyro);

/**
 * @brief Retrives all 12 bytes of gyro data
 * @param[out] acc_data    Processed accelerometer data (gravities)
 * @param[out] raw_acc     Raw accelerometer data
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_get_acc_data(vector3d_t *acc_data, lsm6dsv32x_raw_imu_data_t *raw_acc);