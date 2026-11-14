#ifndef CAN_SENSOR_H
#define CAN_SENSOR_H

#include "rocketlib.h"
#include "application/can_handler/can_handler.h"

//initializes the driver
//adds callbacks to listen for corresponding canbus message
w_status_t can_sensor_init();

/// @brief callback for the analog 3D 16-bit sensor data message and write the message to corresponding buffer
/// @param msg 
/// @note barometer and magnetometer both use this messge type 
/// @return status of the operation
w_status_t can_analog_3d_16bit_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps timestamp message and write the message to corresponding buffer
/// @param msg 
/// @return status of the operation
w_status_t can_gps_timestamp_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps latitude message and write the message to corresponding buffer
/// @param msg  
/// @return status of the operation
w_status_t can_gps_latitude_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps longitude message and write the message to corresponding buffer
/// @param msg 
/// @return status of the operation
w_status_t can_gps_longitude_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps altitude message and write the message to corresponding buffer
/// @param msg 
/// @return status of the operation
w_status_t can_gps_altitude_isr_callback(const can_msg_t *msg);

/// @brief callback for the gps info message and write the message to corresponding buffer
/// @param msg
/// @return status of the operation
w_status_t can_gps_info_isr_callback(const can_msg_t *msg);

//------getter functions -------
/*question on getter functions: do we even care about the 
 timestamp of the sensor data? Shiming said that 
 we'll just use the timestamp of the board 
 so I wont include the timestamp in the getter functions for now.

*/
/// @brief gets the analog 3D 16-bit sensor data from the current buffer 
/// @return status of the operation
w_status_t get_sensor_analog_3d_16bit(can_dem_3d_sensor_id_t *sensor_id,
                                      uint16_t *output_data_x, 
                                      uint16_t *output_data_y, 
                                      uint16_t *output_data_z);

w_status_t get_gps_timestamp(uint8_t *utc_hours, 
                             uint8_t *utc_minutes, 
                             uint8_t *utc_seconds);

w_status_t get_gps_latitude(uint8_t *degrees,
                           uint8_t *minutes, 
                           uint16_t *dminutes_h, 
                           uint8_t *dir_ns);

w_status_t get_gps_longitude(uint8_t *degrees,
                            uint8_t *minutes, 
                            uint16_t *dminutes_h, 
                            uint8_t *dir_ew);

w_status_t get_gps_altitude(uint32_t *altitude, 
                            uint8_t *daltitude);

w_status_t get_gps_info(uint8_t *num_sat, 
                        uint8_t *quality);





#endif

