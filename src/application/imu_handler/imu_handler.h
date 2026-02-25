#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "drivers/altimu-10/altimu-10.h"
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

/**
 * Enum representing an identifier for each imu
 */
typedef enum {
	MOVELLA_IMU,
	POLOLU_IMU,
	ST_IMU
} imu_identifier_t;


/**
 * @brief Initialize the IMU handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void);

/**
 * @brief IMU handler task function for FreeRTOS
 * Should be created during system startup
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument);

/**
 * @brief Reports the current status of the IMU handler module
 * @return CAN board status bitfield
 * @details Logs initialization status, sampling statistics, and error conditions
 * for both IMUs being managed by the handler
 */
uint32_t imu_handler_get_status(void);

/**
 * @brief Reports the magnitude of the acceleration depending on the specified type of IMU
 * @param imu_type defines the type of IMU used
 * @return magnitude of acceleration as a double
 * @details accessor for the acceleration based on request
 */
double imu_handler_get_acceleration(imu_identifier_t imu_type);

#endif // IMU_HANDLER_H
