#ifndef IIS2MDC_H
#define IIS2MDC_H

#include <stdint.h>

#include "application/health_checks/health_checks.h"
#include "common/math/math.h"
#include "rocketlib/include/common.h"

/**
 * @brief Raw magnetometer register data for each axis
 * @note Sensitivity is 1.5 mgauss/LSB, multiply raw by 0.0015 to convert to gauss
 */
typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} iis2mdc_raw_data_t;

/**
 * Status variables describing the current health of the magnetometer module
 * @note Self-test/init-time checks are pre-flight so not counted here.
 */
typedef struct {
	uint32_t dma_read_fails; // HAL_I2C_Mem_Read_DMA failed to start in the DRDY handler
	uint32_t timestamp_fails; // timer_get_ms failed in the DMA-complete callback
	uint32_t get_data_unavailable; // could not get data
	uint32_t i2c_after_callback_switch; // I2C functions called after callback switch
} iis2mdc_health_t;

/**
 * @brief Runs in interrupt context on the IIS2MDC DRDY pin.
 * @note starts a non-blocking DMA read of the six output registers, the DMA-completion callback
 *       converts and publishes the result. Disabled while the module isn't in ready state
 *  	 (uninitialized or under a sanity check).
 */
w_status_t iis2mdc_handle_drdy_irq(void);

/**
 * @brief Initializes the IIS2MDC magnetometer.
 * @note Performs a soft reset, applies the configuration registers, verifies WHO_AM_I, and
 * runs the on-chip self-test. Blocks for 60 ms settle + 50 samples times IIS2MDC_SAMPLE_PERIOD_MS
 * during the self-test.
 * @return Status of the operation
 */
w_status_t iis2mdc_init(void);

/**
 * @brief Reads the magnetometer's output registers and converts to gauss
 * @param[out] data Pointer to store the converted magnetic field values in gauss
 * @param[out] raw_data Pointer to store the raw unsigned 16-bit counts
 * @param[out] timestamp_ms Pointer to store the read timestamp (ms since startup)
 * @return W_SUCCESS on successful read and conversion of most recent sample
 */
w_status_t iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data, uint32_t *timestamp_ms);

/**
 * @brief Get and report the magnetometer status for the health check system
 * @return CAN board status bitfield
 */
health_status_t iis2mdc_get_status(void);

#endif // IIS2MDC_H
