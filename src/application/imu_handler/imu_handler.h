#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "common/gnc/gnc_types.h"
#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"

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

typedef struct { // all of these should be just directly register values
	lsm6dsv32x_raw_imu_data_t raw_board_accel;
	lsm6dsv32x_raw_imu_data_t raw_board_gyro;
	uint32_t raw_board_baro;
	iis2mdc_raw_data_t board_mag;
} raw_board_meas_t;

typedef struct {
	// board
	uint32_t last_board_imu_timestamp_ms;
	uint32_t last_baro_timestamp_ms;
	uint32_t last_mag_timestamp_ms;

	uint32_t last_ad_accel_timestamp_ms;
	uint32_t last_ad_gyro_timestamp_ms;

	uint32_t last_mti_acc_timestamp_ms;
	uint32_t last_mti_gyr_timestamp_ms;
	uint32_t last_mti_mag_timestamp_ms;
	uint32_t last_mti_pres_timestamp_ms;

} imu_handler_ctx_t;

/**
 * @brief Initialize the IMU handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void);

/**
 * @brief Curate fresh data from all sensors. This function executes instantly (non-blocking)
 * to avoid delaying the control loop.
 * @note !!! data that exceeds the sensor's freshness requirement is marked as dead !!!
 *
 * @param ctx pointer to the ctx storing the previously updated times for the sensors
 * @param output pointer to update with the latest data
 * @return Status of the execution
 */
w_status_t imu_handler_get_fresh_meas(imu_handler_ctx_t *ctx, all_sensors_data_t *output);

/**
 * @brief Reports the current status of the IMU handler module
 * @return CAN board status bitfield
 * @details Logs initialization status, sampling statistics, and error conditions
 * for both IMUs being managed by the handler
 */
uint32_t imu_handler_get_status(void);

#endif // IMU_HANDLER_H
