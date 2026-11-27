#ifndef SGPS_CAN_H
#define SGPS_CAN_H

#include <stdint.h>
#include "rocketlib.h"

#include "application/can_handler/can_handler.h"
#include "drivers/timer/timer.h"
#include "common/math/math.h"


/**
 * @brief enum for gps direction type sent over CAN in uint8_t
 * @note for longitude direction it should only be E or W, for latitude direction it should be N or S
 */
enum gps_direction_t {
    N = 78,
    S = 83,
    E = 69,
    W = 87
};

/**
 * @brief Structure to hold GPS longitude data
 * @note lon_dir is 'E' or 'W'
 */
typedef struct {
	uint8_t lon_degrees;
	uint8_t lon_minutes;
	uint16_t lon_dminutes_h;
    gps_direction_t lon_dir;
} gps_longitude_data_t;

/**
 * @brief Structure to hold GPS latitude data
 * @note lat_dir is in 'N' or 'S'
 */
typedef struct {
	uint8_t lat_degrees;
	uint8_t lat_minutes;
	uint16_t lat_dminutes_h;
    gps_direction_t lat_dir;
} gps_latitude_data_t;

/**
 * @brief GPS time of day carried by a GPS CAN message.
 * @note this is the gps' own time not of the canard board
 */
typedef struct {
    uint8_t utc_hours;
    uint8_t utc_minutes;
    uint8_t utc_seconds;
    uint8_t utc_dseconds;
} gps_utc_time_t;


/** 
 * @brief Initializes the CAN sensor driver and registers up the necessary callbacks for handling incoming CAN messages related to sensors.
 * @return status of the operation
 */
w_status_t sgps_can_init(void);

/**
 * @brief read the freshest magnetometer vector provided over CAN
 * @param[out] mag_data outputs the three-axis magnetic-field reading
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_IIS2MDC(vector3d_t *mag_data, uint32_t *timestamp_ms);

/**
 * @brief reads the latest gps UTC time received over CAN
 * @param[out] utc_time_data  outputs the GPS UTC time data
 * @param[out] timestamp_ms  outputs the timestamp of when we first recived the message over CAN
 * @note the utc_time_data gives out the time from the gps board while the timestamp is the time of the board that this function is running off
 * @return status of the operation
 */
w_status_t can_get_gps_utc_time(gps_utc_time_t *utc_time_data, uint32_t *timestamp_ms);

/**
 * @brief read freshest GPS latitude in the degrees/minutes representation of the CAN message
 * @param[out] latitude outputs latitude in degrees, minutes, deciminutes, and direction
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_gps_latitude(gps_latitude_data_t *latitude, uint32_t *timestamp_ms);

/**
 * @brief read freshest GPS longitude in the degrees/minutes representation of the CAN message
 * @param[out] longitude outputs longitude in degrees, minutes, deciminutes, and direction
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_gps_longitude(gps_longitude_data_t *longitude, uint32_t *timestamp_ms);

/**
 * @brief read freshest GPS altitude in the degrees/minutes representation of the CAN message
 * @param[out] altitude outputs gps altitude
 * @param[out] daltitude  outputs the gps deci-altitude
 * @param[out] timestamp_ms  outputs the timestamp of when we first recived the message over CAN
 * @note there is lacking documentation on the daltitude data 
 * @return status of the operation
 */
w_status_t can_get_gps_altitude(uint32_t *altitude, uint8_t *daltitude, uint32_t *timestamp_ms);

/**
 * @brief read freshest gps info received over CAN
 * @param[out] num_sat  outputs the number of satellites
 * @param[out] quality outputs the quality of the GPS fix
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */ 
w_status_t can_get_gps_info(uint8_t *num_sat, uint8_t *quality, uint32_t *timestamp_ms);

/**
 * @brief read freshest barometric pressure and temperature received over CAN
 * @param[out] baro_pressure_pa outputs barometer pressure in Pa
 * @param[out] baro_temp_deg outputs barometer temperature in degrees Celsius
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_ms5611(float32_t *baro_pressure_pa, float32_t *baro_temp_deg, uint32_t *timestamp_ms);
#endif