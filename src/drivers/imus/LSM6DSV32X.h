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
 * @brief Initializes the bit registers for lsm6dsv32x
 * @note Must be called to wake up imu
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init();

