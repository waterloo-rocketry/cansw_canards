#include "drivers/imus/LSM6DSV32X.h"
#include "application/logger/log.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/imus/LSM6DSV32X_regmap.h"
#include "drivers/timer/timer.h"
#include <limits.h>
#include <stdio.h>

// this probably should not go here, cant fing hardware map??
#define IMU_INT1_PORT GPIOD
#define IMU_INT1_PIN GPIO_PIN_7

// AltIMU device addresses and configuration
#define LSM6DSV32X_ADDR 0x6B << 1 // addr sel pin HIGH IMU

// sensor ranges. these must be selected using the i2c init regs
static const double ACC_RANGE = 32.0; // g
static const double GYRO_RANGE = 4000.0; // dps

// AltIMU conversion factors - based on config settings below
static const double ACC_FS = ACC_RANGE / INT16_MAX; // g / LSB
static const double GYRO_FS = GYRO_RANGE / INT16_MAX; // dps / LSB

// struct to hold all context info about the state of the imu
typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t dev_addr;
	uint8_t dual_buffer[2][12];
	TaskHandle_t task_to_notify;
	volatile bool stale_data;
	float timestamp[2];

} imu_ctx_t;

static imu_ctx_t lsm6dsv32x_ctx;

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	return i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
}

/**
 * @brief Initializes the bit registers for
 * @note Must be called after bit registers are configured, called before flight!!!
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init() {
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

	// Disable 2 channel mode
	// Accel full scale range +-32g
	// Accel low pass cutoff frequency ODR/4 (240hz)
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL8_XL, 0x07);

	// accel slope filter: low pass
	// accel user offset bit weight: 2^-10 g/LSB
	// enable user accel user offset
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL9_XL, 0x29);

	if (status != W_SUCCESS) {
		log_text(1, "LSM6DSV32X", "initfail");
	}

	return status;
}

/**
 * @brief Interupt handler for the int1 pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == IMU_INT1_PIN) {
		timer_get_ms(&lsm6dsv32x_ctx.timestamp[IMU_WRITE_BUFFER]);

		// begin dma read to the main buffer
		HAL_I2C_Mem_Read_DMA(lsm6dsv32x_ctx.hi2c,
							 LSM6DSV32X_ADDR,
							 FIFO_READ_BEGIN,
							 I2C_MEMADD_SIZE_8BIT,
							 lsm6dsv32x_ctx.dual_buffer[IMU_WRITE_BUFFER],
							 12);
	}
}

/**
 * @brief handler for after the DMA is completed
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == lsm6dsv32x_ctx.hi2c) {
		memcpy(lsm6dsv32x_ctx.dual_buffer[IMU_READ_BUFFER],
			   lsm6dsv32x_ctx.dual_buffer[IMU_WRITE_BUFFER],
			   12);
		lsm6dsv32x_ctx.stale_data = IMU_DATA_READY;
	}
}

/**
 * @brief Retrives all 12 bytes of imu data
 * @param[out] acc_data    Processed accelerometer data (gravities)
 * @param[out] gyro_data   Processed gyroscope data (deg/s)
 * @param[out] raw_acc     Raw accelerometer data
 * @param[out] raw_gyro    Raw gyroscope data
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_get_gyro_acc_data(vector3d_t *acc_data, vector3d_t *gyro_data,
										altimu_raw_imu_data_t *raw_acc,
										altimu_raw_imu_data_t *raw_gyro, float *timestamp) {
	w_status_t status = W_SUCCESS;
	uint8_t raw_bytes[12]; // copy the bytes so they are safe while doing calculations

	if (lsm6dsv32x_ctx.stale_data == IMU_DATA_READY) {
		taskENTER_CRITICAL();

		// enter a critical section while copying the data
		memcpy(raw_bytes, lsm6dsv32x_ctx.dual_buffer[IMU_READ_BUFFER], 12);
		*timestamp = lsm6dsv32x_ctx.timestamp[IMU_READ_BUFFER];

		// set current data to stale once the buffer is read and coppied into the function
		lsm6dsv32x_ctx.stale_data = IMU_DATA_STALE;

		taskEXIT_CRITICAL();

		// Parse gyroscope raw data (first 6 bytes)
		raw_gyro->x = (uint16_t)(((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
		raw_gyro->y = (uint16_t)(((uint16_t)raw_bytes[3] << 8) | raw_bytes[2]);
		raw_gyro->z = (uint16_t)(((uint16_t)raw_bytes[5] << 8) | raw_bytes[4]);

		// Parse accelerometer raw data (next 6 bytes)
		raw_acc->x = (uint16_t)(((uint16_t)raw_bytes[7] << 8) | raw_bytes[6]);
		raw_acc->y = (uint16_t)(((uint16_t)raw_bytes[9] << 8) | raw_bytes[8]);
		raw_acc->z = (uint16_t)(((uint16_t)raw_bytes[11] << 8) | raw_bytes[10]);

		// Convert to physical units
		gyro_data->x = (int16_t)raw_gyro->x * GYRO_FS;
		gyro_data->y = (int16_t)raw_gyro->y * GYRO_FS;
		gyro_data->z = (int16_t)raw_gyro->z * GYRO_FS;

		acc_data->x = (int16_t)raw_acc->x * ACC_FS;
		acc_data->y = (int16_t)raw_acc->y * ACC_FS;
		acc_data->z = (int16_t)raw_acc->z * ACC_FS;
	} else {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

