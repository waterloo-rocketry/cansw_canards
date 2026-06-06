#include "FreeRTOS.h"
#include "i2c.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/i2c/i2c.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X_regmap.h"
#include "drivers/timer/timer.h"

typedef enum {
	LSM6DSV32X_READ_BUFFER = 0,
	LSM6DSV32X_WRITE_BUFFER = 1
} lsm6dsv32x_buffer_t;

typedef enum {
	LSM6DSV32X_DATA_STALE = 1,
	LSM6DSV32X_DATA_READY = 0
} lsm6dsv32x_data_state_t;

typedef enum {
	LSM6DSV32X_BUS_BUSY = 0,
	LSM6DSV32X_BUS_FREE = 1
} lsm6dsv32x_bus_state;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t dev_addr;
	uint8_t dual_buffer[2][12];
	uint32_t timestamp_ms[2];
	volatile lsm6dsv32x_bus_state bus_status;
	bool switched_callback;

} lsm6dsv32x_ctx_t;

// device addresses and configuration
static const uint16_t LSM6DSV32X_ADDR = 0x6A; // addr sel pin LOW

// external definition of h2ic for the i2c bus 1;
extern I2C_HandleTypeDef hi2c1;

// sensor ranges. these must be selected using the i2c init regs
static const float64_t ACC_RANGE = 32.0; // g
static const float64_t GYRO_RANGE = 4000.0; // dps

// LSM6DSV32X conversion factors based on datasheet
static const float64_t ACC_FS = 0.000976; // g / LSB
static const float64_t GYRO_FS = 0.140; // dps / LSB

static const uint8_t LSM6DSV32X_EXPECTED_WHO_AM_I = 0x70;

static lsm6dsv32x_ctx_t lsm6dsv32x_ctx = {.switched_callback = false};

static const uint8_t CTX_BUFFER_SIZE = sizeof(lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER]);

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
	return i2c_write_reg(I2C_BUS_1, addr, reg, &data, 1);
}

/**
 * @brief Sanity check for the IMU
 * @note just checks the who am i bit
 * @return Status of the operation
 */
static w_status_t lsm6dsv32x_check_sanity() {
	if (lsm6dsv32x_ctx.switched_callback) {
		log_text(1, "LSM6DSV32X", "ERROR: Attempting to reinitialize after switching callback.");
		return W_FAILURE;
	}

	w_status_t i2c_status = W_SUCCESS;
	w_status_t device_status = W_SUCCESS;

	uint8_t who_am_i = 0;

	i2c_status |= i2c_read_reg(I2C_BUS_1, LSM6DSV32X_ADDR, LSM6DSV32X_WHO_AM_I, &who_am_i, 1);
	if (LSM6DSV32X_EXPECTED_WHO_AM_I != who_am_i) {
		device_status |= W_FAILURE;
	}

	if ((W_SUCCESS == device_status) && (W_SUCCESS == i2c_status)) {
		return W_SUCCESS;
	} else {
		return W_FAILURE;
	}
}

// not static to allow to be called in unit test
/**
 * @brief handler for after the DMA is completed
 */
void lsm6dsv32x_dma_complete_handle(I2C_HandleTypeDef *hi2c) {
	if (hi2c != lsm6dsv32x_ctx.hi2c) {
		return;
	}

	lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_FREE;

	memcpy(lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER],
		   lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_WRITE_BUFFER],
		   CTX_BUFFER_SIZE);
}

/**
 * @brief Initializes the bit registers for
 * @note Must be called after bit registers are configured, called before flight!!!
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init() {
	if (lsm6dsv32x_ctx.switched_callback) {
		log_text(1, "LSM6DSV32X", "ERROR: Attempting to reinitialize after switching callback.");
		return W_FAILURE;
	}
	lsm6dsv32x_ctx.hi2c = &hi2c1;

	w_status_t status = W_SUCCESS;

	// LSM6DSV32X: https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf

	// Accel ODR: 960 Hz
	// Accel mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL1_XL, 0x09);

	// Gyro ODR: 960 Hz
	// Gyro mode: high performance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL2_G, 0x09);

	// turn off all FIFO commands
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL1, 0x00);
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL2, 0x00);
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL3, 0x00);
	status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL4, 0x00);

	// set gyro range to +-4000dps
	// set LPF bandwith to 241 (ODR/4)
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL6_G, 0x0C);

	// enable gryro LPF
	// dont touch input impedance
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL7_G, 0x01);

	// Disable 2 channel mode
	// Accel full scale range +-32g
	// Accel low pass cutoff frequency ODR/4 (240hz)
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL8_XL, 0x07);

	// accel slope filter: low pass
	// accel user offset bit weight: 2^-10 g/LSB
	// disable offset, until imu calibration
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL9_XL, 0x29);

	// pulse drdy, so we can get rising edge
	status |= write_1_byte(LSM6DSV32X_ADDR, CTRL4, 0x02);

	status |= lsm6dsv32x_check_sanity();

	// Trigger INT1 when we get either data ready
	status |= write_1_byte(LSM6DSV32X_ADDR, INT1_CTRL, 0x01);

	// load all settings and end use of all i2c driver before switch to dma
	vTaskDelay(1);

	lsm6dsv32x_ctx.switched_callback = true;

	// register the I2C callback for the end of the DMA read to call the dma complete handler
	// function
	HAL_I2C_RegisterCallback(
		lsm6dsv32x_ctx.hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, lsm6dsv32x_dma_complete_handle);

	// AFTER THIS POINT NEVER USE OLD I2C OR VERY BAD ERRORS

	if (status != W_SUCCESS) {
		log_text(1, "LSM6DSV32X", "initfail");
	}

	// only open the i2c bus once fully ready
	lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_FREE;

	return status;
}

/**
 * @brief ISR for the interrupt pin that begins DMA data transfer
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_int1_isr_handler() {
	if (!lsm6dsv32x_ctx.switched_callback) {
		log_text(
			1, "LSM6DSV32X", "ERROR: Attempting to complete DMA callback before new callback.");
		return W_FAILURE;
	}

	w_status_t status = W_SUCCESS;

	// set the bus to occupied meanining that data is
	// actively being streamed via DMA into the Buffer from the IMU
	lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_BUSY;

	// store the timestamp when the data is received
	status |= timer_get_ms(&lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_WRITE_BUFFER]);

	// begin dma read to the main buffer
	const uint8_t shifted_address = (LSM6DSV32X_ADDR << 1);
	HAL_StatusTypeDef hal_status =
		HAL_I2C_Mem_Read_DMA(lsm6dsv32x_ctx.hi2c,
							 shifted_address,
							 OUTX_L_G,
							 I2C_MEMADD_SIZE_8BIT,
							 lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_WRITE_BUFFER],
							 CTX_BUFFER_SIZE);
	if (hal_status != HAL_OK) {
		lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_FREE; // so that we can attempt send again
		status |= W_FAILURE;
	}

	return status;
}

// TODO: make below function into a separate module, interrupts or gpio or dma
/**
 * @brief interrupt handler for the int1 pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ((IMU_INT1_Pin == GPIO_Pin) && (LSM6DSV32X_BUS_FREE == lsm6dsv32x_ctx.bus_status) &&
		lsm6dsv32x_ctx.switched_callback) {
		lsm6dsv32x_int1_isr_handler();
	}
}

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
										lsm6dsv32x_raw_imu_data_t *raw_gyro) {
	w_status_t status = W_SUCCESS;
	uint8_t raw_bytes[12]; // copy the bytes so they are safe while doing calculations

	if (lsm6dsv32x_ctx.switched_callback) {
		taskENTER_CRITICAL();

		// enter a critical section while copying the data
		memcpy(raw_bytes, lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER], CTX_BUFFER_SIZE);
		raw_acc->timestamp_ms = lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_READ_BUFFER];

		taskEXIT_CRITICAL();

		// Parse gyroscope raw data (first 6 bytes)
		raw_gyro->x = (uint16_t)(((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
		raw_gyro->y = (uint16_t)(((uint16_t)raw_bytes[3] << 8) | raw_bytes[2]);
		raw_gyro->z = (uint16_t)(((uint16_t)raw_bytes[5] << 8) | raw_bytes[4]);

		raw_gyro->timestamp_ms =
			raw_acc->timestamp_ms; // grab time from accel since it's the smae timestamp

		// Parse accelerometer raw data (next 6 bytes)
		raw_acc->x = (uint16_t)(((uint16_t)raw_bytes[7] << 8) | raw_bytes[6]);
		raw_acc->y = (uint16_t)(((uint16_t)raw_bytes[9] << 8) | raw_bytes[8]);
		raw_acc->z = (uint16_t)(((uint16_t)raw_bytes[11] << 8) | raw_bytes[10]);

		// Convert to physical units
		gyro_data->x = ((float64_t)((int16_t)raw_gyro->x)) * GYRO_FS;
		gyro_data->y = ((float64_t)((int16_t)raw_gyro->y)) * GYRO_FS;
		gyro_data->z = ((float64_t)((int16_t)raw_gyro->z)) * GYRO_FS;

		acc_data->x = ((float64_t)((int16_t)raw_acc->x)) * ACC_FS;
		acc_data->y = ((float64_t)((int16_t)raw_acc->y)) * ACC_FS;
		acc_data->z = ((float64_t)((int16_t)raw_acc->z)) * ACC_FS;
	} else {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}
