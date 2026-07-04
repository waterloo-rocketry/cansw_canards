#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "i2c.h" // for hi2c4
#include "main.h" // for INT_MAG_Pin
#include "task.h"

#include "application/logger/log.h"
#include "drivers/i2c/i2c.h"
#include "drivers/iis2mdc/IIS2MDC.h"
#include "drivers/timer/timer.h"

// I2C bus and slave address
static const i2c_bus_t IIS2MDC_BUS = I2C_BUS_4;
static const uint8_t IIS2MDC_I2C_ADDR = 0x1E;

// HAL takes the 8-bit left shifted address for HAL_I2C_Mem_Read_DMA calls.
static const uint16_t IIS2MDC_HAL_ADDR = (uint16_t)(IIS2MDC_I2C_ADDR << 1);

// Register addresses
// TODO: Calibration needs to be done in a magnetic field to get offset values written into the
// offset registers.
static const uint32_t IIS2MDC_REG_OFFSET_X_L = 0x45;
static const uint32_t IIS2MDC_REG_WHO_AM_I = 0x4F;
static const uint32_t IIS2MDC_REG_CFG_A = 0x60;
static const uint32_t IIS2MDC_REG_CFG_B = 0x61;
static const uint32_t IIS2MDC_REG_CFG_C = 0x62;
static const uint32_t IIS2MDC_REG_OUTX_L = 0x68;

// SUB MSB that enables auto-increment for multi-byte reads. Since all registers are less than 0x80
// we can freely use this bit
static const uint32_t IIS2MDC_SUB_AUTO_INC = 0x80;

// Expected WHO_AM_I return value
static const uint32_t IIS2MDC_WHO_AM_I_VAL = 0x40;

// Resets config registers
static const uint32_t IIS2MDC_CFG_A_SOFT_RESET = (1 << 5);

// Enables self-testing
static const uint8_t IIS2MDC_CFG_C_SELF_TEST = (1 << 1);

// Output data rate in hz, matching the init config
static const uint32_t IIS2MDC_ODR_HZ = 100;

// One ODR period, scales with IIS2MDC_ODR_HZ.
static const uint32_t IIS2MDC_SAMPLE_PERIOD_MS = 1000U / IIS2MDC_ODR_HZ;

// Self-test parameters (AN5080)
static const float64_t IIS2MDC_SELF_TEST_MIN_GAUSS = 0.015;
static const float64_t IIS2MDC_SELF_TEST_MAX_GAUSS = 0.500;
static const uint32_t IIS2MDC_SELF_TEST_SAMPLES = 50;
static const uint32_t IIS2MDC_POWERUP_MS = 30;
static const uint32_t IIS2MDC_SELF_TEST_SETTLE_MS = 60;
// Wait at most 3 sample periods before declaring no fresh data
static const uint32_t IIS2MDC_SELF_TEST_TIMEOUT_MS = 3U * IIS2MDC_SAMPLE_PERIOD_MS;
static const uint32_t IIS2MDC_SELF_TEST_POLLING_PERIOD_MS = 1;

// Delay before switching to DMA and ending use of I2C driver
static const uint32_t IIS2MDC_SETTINGS_LOAD_DELAY_MS = 1;

/* Init configuration:
 CFG_REG_A = 0x8C  COMP_TEMP_EN=1, LP=0(high-res), ODR=11 (100 Hz), MD=00 (continuous)
 CFG_REG_B = 0x02  OFF_CANC=1 (continuous offset cancellation)
 CFG_REG_C = 0x11  Block data updates to keep data coherent, DRDY_ON_PIN routes data ready to
 interrupt pin
 */
static const uint32_t IIS2MDC_INIT_CFG_A = 0x8C;
static const uint32_t IIS2MDC_INIT_CFG_B = 0x02;
static const uint32_t IIS2MDC_INIT_CFG_C = 0x11;

// conversion factor from raw register values to gauss
static const float64_t IIS2MDC_SENSITIVITY_GAUSS_PER_LSB = 0.0015;

// Guards against starting a new DMA read while one is still in progress
// Also used by iis2mdc_get_data to reject reads while the DMA is mid-burst
static volatile bool iis2mdc_dma_busy = false;

// Outcome of the most recent I2C/DMA operation, shown by iis2mdc_get_status as a
// MODULE_ERR_I2C_FAIL health error. Set to W_SUCCESS when a DMA read is launched and
// to W_IO_ERROR if the launch or timestamp fails.
static volatile w_status_t iis2mdc_latest_status = W_SUCCESS;

// cache for newest data sample. BDMA writes raw_buf directly during HAL_I2C_Mem_Read_DMA;
// timestamp_ms and validity check are done by iis2mdc_dma_complete once the burst finishes.
// Conversions happen in iis2mdc_get_data
typedef struct {
	uint8_t raw_buf[6]; // raw bytes written by the BDMA channel
	uint32_t timestamp_ms; // when this sample was published
	bool valid; // true once at least one sample has been cached
} iis2mdc_cache_t;

static iis2mdc_cache_t iis2mdc_cache __attribute__((section(".sram4"))) = {0};

/* Enum for the state of the driver.
 * UNINIT: before init runs, or after a failed init
 * CHECKING: sanity check in progress
 * ASYNC_DMA_ACTIVE: registered the DMA-complete callback on hi2c4,
 * only the ISR may use hi2c4 at this point, I2C helpers (read/write) not allowed
 */
typedef enum {
	IIS2MDC_STATE_UNINIT = 0,
	IIS2MDC_STATE_CHECKING = 1,
	IIS2MDC_STATE_ASYNC_DMA_ACTIVE = 2
} iis2mdc_state_t;

static volatile iis2mdc_state_t iis2mdc_state = IIS2MDC_STATE_UNINIT;

// Running tally of in-flight failures.
static iis2mdc_health_t iis2mdc_health = {0};

/**
 * @brief Helper function to read one or more consecutive bytes over I2C
 * @note IIS2MDC auto-increments the sub-address if MSB is 1, so multi-byte reads only need the
 * start register. Rejected once the DMA-complete callback is registered.
 */
static w_status_t iis2mdc_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
	if (IIS2MDC_STATE_ASYNC_DMA_ACTIVE == iis2mdc_state) {
		iis2mdc_health.i2c_after_callback_switch++;
		// log_text(1,
		// 		 LOG_LVL_FATAL,
		// 		 "iis2mdc",
		// 		 "ERROR: I2C register read attempted after async pipeline active");
		return W_FAILURE;
	}
	if (len > 1) {
		reg |= IIS2MDC_SUB_AUTO_INC;
	}
	return i2c_read_reg(IIS2MDC_BUS, IIS2MDC_I2C_ADDR, reg, data, len);
}

/**
 * @brief Helper function to write a single byte over I2C
 * @note Rejected once the DMA-complete callback is registered.
 */
static w_status_t iis2mdc_write_reg(uint8_t reg, uint8_t val) {
	if (IIS2MDC_STATE_ASYNC_DMA_ACTIVE == iis2mdc_state) {
		iis2mdc_health.i2c_after_callback_switch++;
		// log_text(1,
		// 		 LOG_LVL_FATAL,
		// 		 "iis2mdc",
		// 		 "ERROR: I2C register write attempted after async pipeline active");
		return W_FAILURE;
	}
	return i2c_write_reg(IIS2MDC_BUS, IIS2MDC_I2C_ADDR, reg, &val, 1);
}

/**
 * @brief Helper function to convert six output register bytes into raw counts and gauss.
 * @note Each output register is uint8, two registers form a piece of data
 * for an axis. This is casted to uint16 for raw data and int16 for gauss.
 */
static void iis2mdc_convert_sample(const uint8_t *buf, iis2mdc_raw_data_t *raw, vector3d_t *data) {
	raw->x = (uint16_t)((((uint16_t)buf[1]) << 8) | buf[0]);
	raw->y = (uint16_t)((((uint16_t)buf[3]) << 8) | buf[2]);
	raw->z = (uint16_t)((((uint16_t)buf[5]) << 8) | buf[4]);

	data->x = ((float64_t)((int16_t)raw->x)) * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
	data->y = ((float64_t)((int16_t)raw->y)) * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
	data->z = ((float64_t)((int16_t)raw->z)) * IIS2MDC_SENSITIVITY_GAUSS_PER_LSB;
}

/**
 * @brief Polls GPIO pin until it asserts (a new sample is ready), waits 1ms between
 *        polls.
 * @note Bounded by IIS2MDC_SELF_TEST_TIMEOUT_MS (3x sample period). Rejected when the async DMA
 *       pipeline is active.
 */
static w_status_t self_test_wait_data_ready(void) {
	if (IIS2MDC_STATE_ASYNC_DMA_ACTIVE == iis2mdc_state) {
		iis2mdc_health.i2c_after_callback_switch++;
		// log_text(1,
		// 		 LOG_LVL_FATAL,
		// 		 "iis2mdc",
		// 		 "ERROR: wait_data_ready called after async DMA pipeline active");
		return W_FAILURE;
	}

	uint32_t start_ms = 0;
	if (W_SUCCESS != timer_get_ms(&start_ms)) {
		return W_FAILURE;
	}

	uint32_t now_ms = start_ms;

	while ((now_ms - start_ms) < IIS2MDC_SELF_TEST_TIMEOUT_MS) {
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(INT_MAG_GPIO_Port, INT_MAG_Pin)) {
			return W_SUCCESS;
		}
		vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SELF_TEST_POLLING_PERIOD_MS));

		if (W_SUCCESS != timer_get_ms(&now_ms)) {
			return W_FAILURE;
		}
	}
	return W_IO_TIMEOUT; // no new sample within the timeout
}

/**
 * @brief Waits for a fresh sample, then reads and converts the output registers to gauss.
 * @note Used for self-test only, not intended for regular data reads
 */
static w_status_t self_test_read_sample(vector3d_t *out) {
	uint8_t buf[6];
	iis2mdc_raw_data_t raw;

	if (W_SUCCESS != self_test_wait_data_ready()) {
		return W_FAILURE;
	}
	if (W_SUCCESS != iis2mdc_read_reg(IIS2MDC_REG_OUTX_L, buf, 6)) {
		return W_FAILURE;
	}
	iis2mdc_convert_sample(buf, &raw, out);
	return W_SUCCESS;
}

/**
 * @brief Discards the first sample, then averages 50 data samples
 */
static w_status_t self_test_collect_average(vector3d_t *avg) {
	vector3d_t sample;
	float64_t sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

	// discard the first sample after a mode change
	if (W_SUCCESS != self_test_read_sample(&sample)) {
		return W_FAILURE;
	}

	for (uint32_t i = 0; i < IIS2MDC_SELF_TEST_SAMPLES; i++) {
		if (W_SUCCESS != self_test_read_sample(&sample)) {
			return W_FAILURE;
		}
		sum_x += sample.x;
		sum_y += sample.y;
		sum_z += sample.z;

		// Yield one ODR period so the next iteration sees a fresh sample
		vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SAMPLE_PERIOD_MS));
	}

	avg->x = sum_x / ((float64_t)IIS2MDC_SELF_TEST_SAMPLES);
	avg->y = sum_y / ((float64_t)IIS2MDC_SELF_TEST_SAMPLES);
	avg->z = sum_z / ((float64_t)IIS2MDC_SELF_TEST_SAMPLES);
	return W_SUCCESS;
}

/**
 * @brief Runs the on-chip self-test (ST AN5080 document for procedure)
 * @note Waits for turn-on, averages the field with self-test off, then on, and checks the
 *       delta against the datasheet range (15-500 mgauss) Restores normal config before returning.
 */
static w_status_t iis2mdc_self_test(void) {
	vector3d_t avg_off, avg_on;

	// discard the first sample, then average with self-test disabled
	if (W_SUCCESS != self_test_collect_average(&avg_off)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "self-test baseline read failed");
		return W_FAILURE;
	}

	// enable self-test and wait 60ms for field to settle (specified in AN 5080)
	if (W_SUCCESS !=
		iis2mdc_write_reg(IIS2MDC_REG_CFG_C, IIS2MDC_INIT_CFG_C | IIS2MDC_CFG_C_SELF_TEST)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "failed to enable self-test");
		return W_FAILURE;
	}
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SELF_TEST_SETTLE_MS));

	w_status_t read_status = self_test_collect_average(&avg_on);

	// restore normal config regardless of the read outcome
	if (W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_C, IIS2MDC_INIT_CFG_C)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "failed to restore configs after self-test");
		return W_FAILURE;
	}
	if (W_SUCCESS != read_status) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "self-test read failed");
		return W_FAILURE;
	}

	// per-axis delta
	float64_t dx = fabs(avg_on.x - avg_off.x);
	float64_t dy = fabs(avg_on.y - avg_off.y);
	float64_t dz = fabs(avg_on.z - avg_off.z);

	if ((dx < IIS2MDC_SELF_TEST_MIN_GAUSS) || (dx > IIS2MDC_SELF_TEST_MAX_GAUSS) ||
		(dy < IIS2MDC_SELF_TEST_MIN_GAUSS) || (dy > IIS2MDC_SELF_TEST_MAX_GAUSS) ||
		(dz < IIS2MDC_SELF_TEST_MIN_GAUSS) || (dz > IIS2MDC_SELF_TEST_MAX_GAUSS)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "self-test out of range: x=%f y=%f z=%f", dx, dy, dz);
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Performs a sanity check by verifying device identity and running a self test
 * @note Marks state CHECKING so init re entries are rejected
 * @return W_SUCCESS if the device passes self test and identity verification
 */
static w_status_t iis2mdc_sanity_check(void) {
	// Checks to make sure sanity check is not already in progress or that async DMA is not already
	// active.
	if (IIS2MDC_STATE_UNINIT != iis2mdc_state) {
		log_text(1,
				 LOG_LVL_FATAL,
				 "iis2mdc",
				 "ERROR: sanity check called from invalid state %u",
				 iis2mdc_state);
		return W_FAILURE;
	}

	uint8_t id = 0;
	w_status_t status = W_SUCCESS;
	iis2mdc_state = IIS2MDC_STATE_CHECKING;

	if (W_SUCCESS != iis2mdc_read_reg(IIS2MDC_REG_WHO_AM_I, &id, 1)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "ERROR: failed to read WHO_AM_I");
		status = W_FAILURE;
	} else if (IIS2MDC_WHO_AM_I_VAL != id) {
		log_text(1,
				 LOG_LVL_FATAL,
				 "iis2mdc",
				 "WHO_AM_I mismatch: expected %u, got %u",
				 IIS2MDC_WHO_AM_I_VAL,
				 id);
		status = W_FAILURE;
	} else if (W_SUCCESS != iis2mdc_self_test()) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "ERROR: self-test failed");
		status = W_FAILURE;
	}

	iis2mdc_state = IIS2MDC_STATE_UNINIT;
	return status;
}

w_status_t iis2mdc_handle_drdy_irq(void) {
	// stand down unless ASYNC_DMA_ACTIVE
	if (IIS2MDC_STATE_ASYNC_DMA_ACTIVE != iis2mdc_state) {
		return W_FAILURE;
	}
	// stand down if last DMA is still in progress
	if (iis2mdc_dma_busy) {
		return W_FAILURE;
	}
	iis2mdc_dma_busy = true;
	iis2mdc_latest_status = W_SUCCESS; //clears error if last drdy failed, will retry on next drdy

	if (HAL_OK != HAL_I2C_Mem_Read_DMA(&hi2c4,
									   IIS2MDC_HAL_ADDR,
									   IIS2MDC_REG_OUTX_L | IIS2MDC_SUB_AUTO_INC,
									   I2C_MEMADD_SIZE_8BIT,
									   iis2mdc_cache.raw_buf,
									   sizeof(iis2mdc_cache.raw_buf))) {
		iis2mdc_health.dma_read_fails++;
		iis2mdc_latest_status = W_IO_ERROR;
		iis2mdc_dma_busy = false; // failed to start, next DRDY will retry
	}
	return W_SUCCESS;
}

/**
 * @brief I2C DMA completion. Completes the sample by adding timestamp and verifying validity
 * @note Registered on hi2c4 via HAL_I2C_RegisterCallback in iis2mdc_init. Byte-combining,
 *       sign interpretation, and scaling to gauss are deferred to iis2mdc_get_data.
 */
static void iis2mdc_dma_complete(I2C_HandleTypeDef *hi2c) {
	if (hi2c != &hi2c4) {
		return;
	}
	// this function needs DMA to be active
	if (IIS2MDC_STATE_ASYNC_DMA_ACTIVE != iis2mdc_state) {
		return;
	}

	uint32_t timestamp_ms = 0;
	if (W_SUCCESS != timer_get_ms(&timestamp_ms)) {
		iis2mdc_health.timestamp_fails++;
		iis2mdc_latest_status = W_IO_ERROR;
		iis2mdc_dma_busy = false;
		return;
	}

	iis2mdc_cache.timestamp_ms = timestamp_ms;
	iis2mdc_cache.valid = true;

	iis2mdc_dma_busy = false;
}

w_status_t iis2mdc_init(void) {
	// reject reinitialization
	if (IIS2MDC_STATE_UNINIT != iis2mdc_state) {
		log_text(1,
				 LOG_LVL_FATAL,
				 "iis2mdc",
				 "ERROR: init called from non-UNINIT state %u",
				 iis2mdc_state);
		return W_FAILURE;
	}

	// wait for stable output after power-up before any access to registers
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_POWERUP_MS));

	// soft reset clears config registers
	if (W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_A, IIS2MDC_CFG_A_SOFT_RESET)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "soft reset failed");
		return W_FAILURE;
	}

	// wait before writing to registers again after soft reset
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_POWERUP_MS));

	if ((W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_A, IIS2MDC_INIT_CFG_A)) ||
		(W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_B, IIS2MDC_INIT_CFG_B)) ||
		(W_SUCCESS != iis2mdc_write_reg(IIS2MDC_REG_CFG_C, IIS2MDC_INIT_CFG_C))) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "failed to write configuration registers");
		return W_FAILURE;
	}

	if (W_SUCCESS != iis2mdc_sanity_check()) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "sanity check failed");
		return W_FAILURE;
	}

	// load all settings and end use of i2c driver before switching to DMA
	vTaskDelay(pdMS_TO_TICKS(IIS2MDC_SETTINGS_LOAD_DELAY_MS));

	// register completion callback on the mag's I2C handle.
	if (HAL_OK !=
		HAL_I2C_RegisterCallback(&hi2c4, HAL_I2C_MEM_RX_COMPLETE_CB_ID, iis2mdc_dma_complete)) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "ERROR: failed to register DMA complete callback");
		return W_FAILURE;
	}

	iis2mdc_state = IIS2MDC_STATE_ASYNC_DMA_ACTIVE;

	if (W_SUCCESS != iis2mdc_handle_drdy_irq()) {
		log_text(1, LOG_LVL_FATAL, "iis2mdc", "ERROR: failed to clear interrupt");
		return W_FAILURE;
	}
	return W_SUCCESS;
}

w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data,
							uint32_t *timestamp_ms) {
	if ((NULL == data) || (NULL == raw_data) || (NULL == timestamp_ms)) {
		log_text(1,
				 LOG_LVL_WARN,
				 "iis2mdc",
				 "NULL pointer cannot be used as input to get_data function");
		return W_FAILURE;
	}

	if ((IIS2MDC_STATE_ASYNC_DMA_ACTIVE != iis2mdc_state) || !(iis2mdc_cache.valid)) {
		iis2mdc_health.get_data_unavailable++;
		return W_IO_ERROR;
	}

	uint8_t local_buf[6];
	taskENTER_CRITICAL();
	if (iis2mdc_dma_busy) {
		taskEXIT_CRITICAL();
		iis2mdc_health.get_data_unavailable++;
		return W_IO_ERROR;
	}
	memcpy(local_buf, iis2mdc_cache.raw_buf, sizeof(local_buf));
	*timestamp_ms = iis2mdc_cache.timestamp_ms;
	taskEXIT_CRITICAL();

	iis2mdc_convert_sample(local_buf, raw_data, data);

	return W_SUCCESS;
}

health_status_t iis2mdc_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_IIS2MDC, .error_bitfield = 0};

	// I2C/DMA failure
	if (W_IO_ERROR == iis2mdc_latest_status) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_I2C_FAIL;
	}

	// driver not initialized
	if (IIS2MDC_STATE_UNINIT == iis2mdc_state) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_NOT_INIT;
	}

	log_text(10,
			 LOG_LVL_INFO,
			 "iis2mdc",
			 "state=%u, dma_busy=%d, cache_valid=%d, dma_read_fails=%lu, timestamp_fails=%lu",
			 iis2mdc_state,
			 iis2mdc_dma_busy,
			 iis2mdc_cache.valid,
			 iis2mdc_health.dma_read_fails,
			 iis2mdc_health.timestamp_fails);

	log_text(10,
			 LOG_LVL_INFO,
			 "iis2mdc",
			 "get_data_unavailable=%lu, i2c_after_callback_switch=%lu, latest_status=%d",
			 iis2mdc_health.get_data_unavailable,
			 iis2mdc_health.i2c_after_callback_switch,
			 iis2mdc_latest_status);

	return status;
}
