#ifndef CAN_SENSOR_H
#define CAN_SENSOR_H

#include "rocketlib.h"
#include "application/can_handler/can_handler.h"
#include "drivers/timer/timer.h"
#include <stdint.h>


/**
 * @brief Structure to hold GPS position data
 * @note lat_dir is in 'N' or 'S', lon_dir is in 'E' or 'W'
 */
typedef struct __attribute__((packed)) {
	uint8_t lat_degrees;
	uint8_t lat_minutes;
	uint16_t lat_dminutes_h;
    uint8_t lat_dir; 
    uint8_t lon_degrees;
	uint8_t lon_minutes;
	uint16_t lon_dminutes_h;
    uint8_t lon_dir;
    uint32_t *altitude;
    uint8_t *daltitude;
} gps_position_data_t;

/**
 * @brief Structure to hold GPS timestamp data
 * @note hours, minutes and seconds are in UTC time
 */
typedef struct __attribute__((packed)) {
    uint8_t *utc_hours;
    uint8_t *utc_minutes;
    uint8_t *utc_seconds;
} gps_timestamp_data_t;


/** 
 * @brief Initializes the CAN sensor driver and sets up the necessary callbacks for handling incoming CAN messages related to sensors.
 * @return status of the operation
 */
w_status_t can_sensor_init();

//------getter functions -------
/**
 * @brief getter function for the CANARD_IIS2MDC_MAG message, returns the magnetometer readings and timestamp
 * @param mag_data [out] pointer to store the magnetometer readings and timestamp
 * @return status of the operation
 */
w_status_t get_canard_IIS2MDC_MAG(vector3d_t *mag_data, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps timestamp message
 * @param timestamp_data [out] pointer to store the GPS timestamp data
 * @return status of the operation
 */
w_status_t get_gps_timestamp(gps_timestamp_data_t *timestamp_data, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps message, returns the gps position data
 * @param position_data [out] pointer to store the gps position data
 * @note gps latitude, longitude and altitude are sent from different messages, so this function will return the latest data received from each message
 * @return status of the operation
 */
w_status_t get_gps_position(gps_position_data_t * position_data, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps info message, returns the number of satellites and quality of the GPS fix
 * @param num_sat [out] pointer to store the number of satellites
 * @param quality [out] pointer to store the quality of the GPS fix
 * @return status of the operation
 */ 
w_status_t get_gps_info(uint8_t *num_sat, uint8_t *quality, uint32_t *timestamp_ms);

/**
 * @brief getter function for the CANARD_MTI630_BARO message, returns the barometer readings and timestamp
 * @param baro_data [out] pointer to store the barometer readings and timestamp
 * @return status of the operation
 */
w_status_t get_ms5611_baro(float32_t *baro_pres_pa, float32_t *baro_temp_deg, uint32_t *timestamp_ms);
#endif

