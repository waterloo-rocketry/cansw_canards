#ifndef CAN_SENSOR_H
#define CAN_SENSOR_H

#include <stdint.h>
#include "rocketlib.h"
#include "application/can_handler/can_handler.h"
#include "drivers/timer/timer.h"
#include "common/math/math.h"

/**
 * @brief Structure to hold GPS position data
 * @note lat_dir is in 'N' or 'S', lon_dir is in 'E' or 'W'
 */
typedef struct __attribute__((packed)) {
	uint8_t degrees;
	uint8_t minutes;
	uint16_t dminutes_h;
    uint8_t dir;
} gps_position_data_t;

/**
 * @brief Structure to hold GPS timestamp data
 * @note hours, minutes and seconds are in UTC time
 */
typedef struct __attribute__((packed)) {
    uint8_t utc_hours;
    uint8_t utc_minutes;
    uint8_t utc_seconds;
    uint8_t utc_dseconds;
} gps_timestamp_data_t;


/** 
 * @brief Initializes the CAN sensor driver and sets up the necessary callbacks for handling incoming CAN messages related to sensors.
 * @return status of the operation
 */
/**
 * @brief Structure to hold GPS timestamp data
 * @note hours, minutes and seconds are in UTC time
 */
typedef struct __attribute__((packed)) {
    uint8_t utc_hours;
    uint8_t utc_minutes;
    uint8_t utc_seconds;
    uint8_t utc_dseconds;
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
 * @param timestamp_ms [out] pointer to store the timestamp of the message received
 * @return status of the operation
 */
w_status_t can_get_canard_IIS2MDC_MAG(vector3d_t *mag_data, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps timestamp message
 * @param timestamp_data [out] pointer to store the GPS timestamp data
 * @param timestamp_ms [out] pointer to store the timestamp of the message recieved
 * @return status of the operation
 */
w_status_t can_get_gps_timestamp(gps_timestamp_data_t *timestamp_data, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps latitude data
 * @param latitude [out] pointer to store the gps latitude
 * @param timestamp_ms [out] pointer to store the timestamp of the message recieved
 * @return status of the operation
 */
w_status_t can_get_gps_latitude(gps_position_data_t *latitude, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps longitude data
 * @param longitude [out] pointer to store the gps longitude  
 * @param timestamp_ms [out] pointer to store the timestamp of the message recieved
 * @return status of the operation
 */
w_status_t can_get_gps_longitude(gps_position_data_t *longitude, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps altitude data
 * @param altitude [out] pointer to store the gps altitude
 * @param daltitude [out] pointer to store the gps daltitude
 * @param timestamp_ms [out] pointer to store the timestamp of the message recieved
 * @return status of the operation
 */
w_status_t can_get_gps_altitude(uint32_t *altitude, uint8_t *daltitude, uint32_t *timestamp_ms);

/**
 * @brief getter function for the gps info message, returns the number of satellites and quality of the GPS fix
 * @param num_sat [out] pointer to store the number of satellites
 * @param quality [out] pointer to store the quality of the GPS fix
 * @param timestamp_ms [out] pointer to store the timestamp of the message recieved
 * @return status of the operation
 */ 
w_status_t can_get_gps_info(uint8_t *num_sat, uint8_t *quality, uint32_t *timestamp_ms);

/**
 * @brief getter function for the CANARD_MTI630_BARO message, returns the barometer readings and timestamp
 * @param baro_data [out] pointer to store the barometer readings and timestamp
 * @param timestamp_ms [out] pointer to store the timestamp of the message recieved
 * @return status of the operation
 */
w_status_t can_get_ms5611_baro(float32_t *baro_pres_pa, float32_t *baro_temp_deg, uint32_t *timestamp_ms);
#endif