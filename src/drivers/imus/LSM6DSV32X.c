#include "application/logger/log.h"
#include "drivers/altimu-10/LIS3MDL_regmap.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/LSM6DSO_regmap.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include <limits.h>
#include <stdio.h>

// AltIMU device addresses and configuration
#define LSM6DSV32X_ADDR 0x6B // addr sel pin HIGH IMU

// sensor ranges. these must be selected using the i2c init regs
static const double ALTIMU_ACC_RANGE = 16.0; // g
static const double ALTIMU_GYRO_RANGE = 2000.0; // dps
static const double ALTIMU_MAG_RANGE = 16.0; // Gauss

// lps22df baro range is actually smaller than this, but we deemed better to "extend" the range and
// gamble on possible undefined values, as thats better than saturating for estimator
static const double ALTIMU_BARO_MAX = 110000; // conservative max Pa at sea level
static const double ALTIMU_BARO_MIN = 6000; // min Pa we can possibly reach (60000 ft apogee)

// AltIMU conversion factors - based on config settings below
static const double ACC_FS = ALTIMU_ACC_RANGE / INT16_MAX; // g / LSB
static const double GYRO_FS = ALTIMU_GYRO_RANGE / INT16_MAX; // dps / LSB
static const double MAG_FS = ALTIMU_MAG_RANGE / INT16_MAX; // gauss / LSB
static const double BARO_FS = 100.0 / 4096.0; // fixed scale: 100 Pa / 4096 LSB
static const double TEMP_FS = 1.0 / 100.0; // fixed scale: 1 deg C / 100 LSB

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	return i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
}

/**
 * @brief Initializes the Pololu AltIMU incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t altimu_init() {
	w_status_t status = W_SUCCESS;

	// Drive addr sel pin HIGH to use each device's "default" i2c addr
	status |= gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);

	// LSM6DSV32X: https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf

	// Accel ODR: 960 Hz
	// Accel mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL1_XL, 0x9);

	// Gyro ODR: 960 Hz
	// Gyro mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL2_G, 0x9);

	// set FIFO watermark at a singular sample
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL1, 0x01);

	// keep settings default for now.. might change
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL2, 0x00);

	// gyro and accel batch data rate: 960 Hz (same as ODR)
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL3, 0x99);

	// Batch timestamps for every sample
	// Dont batch any temperature data
	// FIFO mode: continous (overwrite old data when full)
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL4, 0x9);

	// Trigger INT1 when the FIFO is full
	status |= write_1_byte(LSM6DSV32X_ADDR, INT1_CTRL, 0x20);

	// set gyro range to +-4000dps
	// set LPF bandwith to 100
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL6_G, 0x4C);

	// set gyro range to +-4000dps
	// set LPF bandwith to 100
	// disable 2 channel mode, is higher resolution data usefull to store in other registers for
	// logging purposes??
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL8_XL, 0x4C);

	if (status != W_SUCCESS) {
		log_text(1, "LSM6DSV32X", "initfail");
	}

	return status;
}

