#include "main.h"
#include "stm32h7xx_hal.h"
#include <limits.h>
#include <stdio.h>

#include "application/logger/log.h"
#include "drivers/imus/LSM6DSV32X.h"
// #include "drivers/altimu-10/LPS_regmap.h"
// #include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/imus/LSM6DSV32X_regmap.h"
#include "drivers/timer/timer.h"

// device addresses and configuration
#define LSM6DSV32X_ADDR 0x6A // addr sel pin LOW

// external definition of h2ic for the i2c bus 1;
extern I2C_HandleTypeDef hi2c1;

// sensor ranges. these must be selected using the i2c init regs
static const float ACC_RANGE = 32.0; // g
static const float GYRO_RANGE = 4000.0; // dps

// AltIMU conversion factors - based on config settings below
static const float ACC_FS = ACC_RANGE / INT16_MAX; // g / LSB
static const float GYRO_FS = GYRO_RANGE / INT16_MAX; // dps / LSB

static imu_ctx_t lsm6dsv32x_ctx = {0};

static const uint8_t CTX_BUFFER_SIZE = sizeof(lsm6dsv32x_ctx.dual_buffer[IMU_READ_BUFFER]);

// TODO: update to use the proper i2c bus

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	return i2c_write_reg(I2C_BUS_1, addr, reg, &data, 1);
}

/**
 * @brief Sainity check for the IMU
 * @note just checks the who am i bit
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_check_sanity() {
	w_status_t i2c_status = W_SUCCESS;
	w_status_t device_status = W_SUCCESS;

	uint8_t expected_lsm6dsv32x = 0x70;
	uint8_t who_am_i = 0;

	i2c_status |= i2c_read_reg(I2C_BUS_1, LSM6DSV32X_ADDR, LSM6DSV32X_WHO_AM_I, &who_am_i, 1);
	if (expected_lsm6dsv32x != who_am_i) {
		device_status |= W_FAILURE;
	}

	if ((W_SUCCESS == device_status) && (W_SUCCESS == i2c_status)) {
		return W_SUCCESS;
	} else {
		return W_FAILURE;
	}
}

/**
 * @brief Initializes the bit registers for
 * @note Must be called after bit registers are configured, called before flight!!!
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init() {
	lsm6dsv32x_ctx.bus_status = IMU_BUS_FREE;
	lsm6dsv32x_ctx.hi2c = &hi2c1;
	lsm6dsv32x_ctx.stale_data = IMU_DATA_STALE;

	w_status_t status = W_SUCCESS;

	// Drive addr sel pin HIGH to use each device's "default" i2c addr
	// status |= gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);

	// LSM6DSV32X: https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf

	// turn off INT1 to start. So that we can read a rising edge
	status |= write_1_byte(LSM6DSV32X_ADDR, INT1_CTRL, 0x00);

	// have time for this to load
	vTaskDelay(1);

	// Accel ODR: 960 Hz
	// Accel mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL1_XL, 0x09);

	// Gyro ODR: 960 Hz
	// Gyro mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL2_G, 0x09);

	// Accel ODR: 960 Hz
	// Accel mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL1_XL, 0x9);

	// Gyro ODR: 960 Hz
	// Gyro mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL2_G, 0x09);

	// set FIFO watermark at a singular sample
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL1, 0x00);

	// keep settings default for now.. might change
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL2, 0x00);

	// gyro and accel batch data rate: 960 Hz (same as ODR)
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL3, 0x00);

	// Batch timestamps for every sample
	// Dont batch any temperature data
	// FIFO mode: continous (overwrite old data when full)
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL4, 0x00);

	// set gyro range to +-4000dps
	// set LPF bandwith to 241 (ODR/4)
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL6_G, 0x0C);

	// enable gryro LPF
	// dont touch imput impedance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL7_G, 0x01);

	// Disable 2 channel mode
	// Accel full scale range +-32g
	// Accel low pass cutoff frequency ODR/4 (240hz)
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL8_XL, 0x07);

	// accel slope filter: low pass
	// accel user offset bit weight: 2^-10 g/LSB
	// disable offset, untill imu calibration
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL9_XL, 0x29);

	// register the I2C callback for the end of the DMA read to call the dma complete handker
	// function
	HAL_I2C_RegisterCallback(
		lsm6dsv32x_ctx.hi2c, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, lsm6dsv32x_dma_complete_handle);

	
	// pulse drdy
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL4, 0x02);

	status |= lsm6dsv32x_check_sanity();



	// Trigger INT1 when the FIFO is full
	status |= write_1_byte(LSM6DSV32X_ADDR, INT1_CTRL, 0x03);

	uint8_t i2c_read_data[12] = {0};

	status |= i2c_read_reg(I2C_BUS_1, LSM6DSV32X_ADDR, 0x22, i2c_read_data, 12);

	if (status != W_SUCCESS) {
		log_text(1, "LSM6DSV32X", "initfail");
	}

	return status;
}

w_status_t lsm6dsv32x_int1_isr_handler() {
	w_status_t status = W_SUCCESS;

	// set the bus to occupied meanining that data is
	// actively being streamed via DMA into the Buffer from the IMU
	lsm6dsv32x_ctx.bus_status = IMU_BUS_BUSY;

	// store the timestamp when the data is recieved
	status |= timer_get_ms(&lsm6dsv32x_ctx.timestamp[IMU_WRITE_BUFFER]);

	// begin dma read to the main buffer
	// status |= 

	HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_DMA(lsm6dsv32x_ctx.hi2c,
								   LSM6DSV32X_ADDR,
								   0x22,
								   I2C_MEMADD_SIZE_8BIT,
								   lsm6dsv32x_ctx.dual_buffer[IMU_WRITE_BUFFER],
								   CTX_BUFFER_SIZE);
	if (hal_status != HAL_OK) {
		return W_FAILURE;
	}

	return status;
}

/**
 * @brief Interupt handler for the int1 pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ((GPIO_Pin == IMU_INT1_Pin) && (IMU_BUS_FREE == lsm6dsv32x_ctx.bus_status)) {
		lsm6dsv32x_int1_isr_handler();
	}
}

// void  HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
// 										lsm6dsv32x_dma_complete_handle(hi2c);
// 								}

/**
 * @brief handler for after the DMA is completed
 */
void lsm6dsv32x_dma_complete_handle(I2C_HandleTypeDef *hi2c) {
	if (hi2c != lsm6dsv32x_ctx.hi2c) {
		return;
	}

	lsm6dsv32x_ctx.bus_status = IMU_BUS_FREE;

	memcpy(lsm6dsv32x_ctx.dual_buffer[IMU_READ_BUFFER],
		   lsm6dsv32x_ctx.dual_buffer[IMU_WRITE_BUFFER],
		   CTX_BUFFER_SIZE);

	lsm6dsv32x_ctx.stale_data = IMU_DATA_READY;
}

// TODO: make below function into a seperate module, interupts or gpio or dma

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
										lsm6dsv32x_raw_imu_data_t *raw_gyro) {
	w_status_t status = W_SUCCESS;
	uint8_t raw_bytes[12]; // copy the bytes so they are safe while doing calculations

	if (IMU_DATA_READY == lsm6dsv32x_ctx.stale_data) {
		taskENTER_CRITICAL();

		// enter a critical section while copying the data
		memcpy(raw_bytes, lsm6dsv32x_ctx.dual_buffer[IMU_READ_BUFFER], CTX_BUFFER_SIZE);
		//*timestamp = lsm6dsv32x_ctx.timestamp[IMU_READ_BUFFER];

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
