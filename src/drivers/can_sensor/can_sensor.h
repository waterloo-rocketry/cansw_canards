#ifndef CAN_SENSOR_H
#define CAN_SENSOR_H

#include "rocketlib.h"
#include "application/can_handler/can_handler.h"
#include "drivers/timer/timer.h"
#include <stdint.h>

/// @brief datatype to hold mag data
typedef struct __attribute__((packed)) {
	uint16_t x;
	uint16_t y;
	uint16_t z;
    uint32_t timestamp_ms;
} mag_data_t;

/// @note dir: N/S for latitude, E/W for longitude
typedef struct __attribute__((packed)) {
	uint8_t degrees;
	uint8_t minutes;
	uint16_t dminutes_h;
    uint8_t dir; 
} gps_position_data_t;


/// @brief Initializes the CAN sensor driver and sets up the necessary callbacks for handling incoming CAN messages related to sensors.
/// @return status of the operation
w_status_t can_sensor_init();

/// @brief callback for the analog 3D 16-bit sensor data message and write the message to corresponding buffer
/// @param msg [in] pointer to the incoming CAN message
/// @note magnetometer both use this messge type 
/// @return status of the operation
w_status_t can_analog_3d_16bit_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps timestamp message and write the message to corresponding buffer
/// @param msg [in] pointer to the incoming CAN message
/// @return status of the operation
w_status_t can_gps_timestamp_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps latitude message and write the message to corresponding buffer
/// @param msg  [in] pointer to the incoming CAN message
/// @return status of the operation
w_status_t can_gps_latitude_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps longitude message and write the message to corresponding buffer
/// @param msg [in] pointer to the incoming CAN message
/// @return status of the operation
w_status_t can_gps_longitude_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps altitude message and write the message to corresponding buffer
/// @param msg [in] pointer to the incoming CAN message
/// @return status of the operation
w_status_t can_gps_altitude_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps info message and write the message to corresponding buffer
/// @param msg [in] pointer to the incoming CAN message
/// @return status of the operation
w_status_t can_gps_info_isr_callback(const can_msg_t *msg);

/// @brief callback for the CANARD_MTI630_BARO message and write the message to corresponding buffer
/// @param msg [in] pointer to the incoming CAN message
////@note there is two different barometer messages, Baro reading 0 with 0x37 code and baro reading 1 with 0x38, I'm passing both to this callback (not sure if this is fine)
/// @return status of the operation
w_status_t can_CANARD_MTI630_BARO_isr_callback(const can_msg_t *msg);

//------getter functions -------

/// @brief getter function for the CANARD_IIS2MDC_MAG message, returns the magnetometer readings and timestamp
/// @param output_data [out] pointer to store the magnetometer readings and timestamp
/// @note there is already a iis2mdc driver, do i even need this? because that driver is using I2C to get the readings, but this is using CAN to get the readings
/// @return status of the operation
w_status_t get_canard_IIS2MDC_MAG(mag_data_t *output_data);

/// @brief getter function for the CANARD_MTI630_MAG message, returns the magnetometer readings and timestamp
/// @param output_data [out] pointer to store the magnetometer readings and timestamp
/// @return status of the operation
w_status_t get_canard_MTI630_MAG(mag_data_t *output_data);

/// @brief getter function for the gps timestamp message
/// @param utc_hours [out] UTC hours
/// @param utc_minutes [out] UTC minutes
/// @param utc_seconds [out] UTC seconds
/// @return status of the operation
w_status_t get_gps_timestamp(uint8_t *utc_hours, 
                             uint8_t *utc_minutes, 
                             uint8_t *utc_seconds);

/// @brief getter function for the gps position message, returns the latitude, longitude, altitude, and delta altitude
/// @param latitude [out] latitude data
/// @param longitude[out] longitude data  
/// @param altitude [out] altitude data
/// @param daltitude [out] delta altitude data
/// @return status of the operation
w_status_t get_gps_position(gps_position_data_t *latitude,
                            gps_position_data_t *longitude,
                            uint32_t *altitude, 
                            uint8_t *daltitude);

/// @brief getter function for the gps info message
/// @param num_sat [out] number of satellites
/// @param quality [out] signal quality
/// @return status of the operation
w_status_t get_gps_info(uint8_t *num_sat, 
                        uint8_t *quality);

/// @brief getter function for the CANARD_MTI630_BARO message, returns the two barometer readings and timestamp
/// @param pressure0 [out] first barometer reading
/// @param pressure1  [out] second barometer reading
/// @param timestamp_ms [out] timestamp of the barometer readings
/// @note first theres two different baro messages, also There is MS5611 barometer but i think theres alreay a driver for that, so this is just for the CANARD_MTI630_BARO message
/// @return status of the operation
w_status_t get_canard_MTI630_BARO(uint16_t *pressure0, 
                                  uint16_t *pressure1, 
                                  uint32_t *timestamp_ms);



#endif

