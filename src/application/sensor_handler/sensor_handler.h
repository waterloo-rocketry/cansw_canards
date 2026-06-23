#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "common/gnc/gnc_types.h"
#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/MS5611/MS5611.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"

#include "application/health_checks/health_checks.h"
#include "rocketlib/include/common.h"

typedef struct { // all of these should be just directly register values
	lsm6dsv32x_raw_imu_data_t raw_board_accel;
	lsm6dsv32x_raw_imu_data_t raw_board_gyro;
	ms5611_raw_result_t raw_board_baro;
	iis2mdc_raw_data_t raw_board_mag;
} raw_board_meas_t;

typedef struct {
	// the last successful read timestamps
	uint32_t last_board_imu_timestamp_ms;
	uint32_t last_baro_timestamp_ms;
	uint32_t last_mag_timestamp_ms;

	uint32_t last_ad_accel_timestamp_ms;
	uint32_t last_ad_gyro_timestamp_ms;

	uint32_t last_mti_acc_timestamp_ms;
	uint32_t last_mti_gyr_timestamp_ms;
	uint32_t last_mti_mag_timestamp_ms;
	uint32_t last_mti_pres_timestamp_ms;

	uint32_t last_motor_encoder_timestamp_ms;
} sensor_handler_ctx_t;

/**
 * @brief Initialize the Sensor handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t sensor_handler_init(void);

/**
 * @brief Curate fresh data from all sensors. This function executes instantly (non-blocking)
 * to avoid delaying the control loop.
 * @note !!! data that exceeds the sensor's freshness requirement is marked as dead !!!
 *
 * @param ctx pointer to the ctx storing the previously updated times for the sensors
 * @param output pointer to update with the latest data
 * @return Status of the execution
 */
w_status_t sensor_handler_get_fresh_meas(sensor_handler_ctx_t *ctx, all_sensors_data_t *output);

/**
 * @brief Reports the current status of the Sensor handler module
 * @return CAN board status bitfield
 * @details Logs initialization status, sampling statistics, and error conditions
 * for both Sensors being managed by the handler
 */
health_status_t sensor_handler_get_status(void);

#endif // SENSOR_HANDLER_H
