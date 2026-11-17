#include "FreeRTOS.h"
#include "common/math/math.h"
#include "drivers/can_sensor/can_sensor.h"
#include "rocketlib.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"


bool isInit = true;

//buffers
/*
we need buffers for 
IIS2MC_MAG
MTI60_MAG
gps_timestamp
gps_pos (latitude, longitude and altitude)
gps_info
mti630 baro (theres baro0 and baro1)
*/




w_status_t can_sensor_init() {
    isInit = true;
    if (!(can_handler_register_callback(MSG_SENSOR_3D_ANALOG16, can_analog_3d_16bit_isr_callback) == W_SUCCESS &&
        can_handler_register_callback(MSG_GPS_TIMESTAMP, can_gps_timestamp_isr_callback) == W_SUCCESS &&
        can_handler_register_callback(MSG_GPS_LATITUDE, can_gps_latitude_isr_callback) == W_SUCCESS &&
        can_handler_register_callback(MSG_GPS_LONGITUDE, can_gps_longitude_isr_callback) == W_SUCCESS &&
        can_handler_register_callback(MSG_GPS_ALTITUDE, can_gps_altitude_isr_callback) == W_SUCCESS &&
        can_handler_register_callback(MSG_GPS_INFO, can_gps_info_isr_callback) == W_SUCCESS &&
        can_handler_register_callback(MSG_SENSOR_ANALOG16, can_CANARD_MTI630_BARO_isr_callback) == W_SUCCESS)) {
        log_text(1, LOG_LVL_WARN, "CAN_SENSOR",
		         "Failed to register sensor callback to CANBUS handler");
		return W_FAILURE;
    }
    //what else needs to be here
    isInit = false;
    return W_SUCCESS;
}

w_status_t can_analog_3d_16bit_isr_callback(const can_msg_t *msg) {
    if (isInit) {
        return W_FAILURE;
    }
    uint16_t x,y,z;
    can_dem_3d_sensor_id_t sensor_id;
    if (get_3d_analog_sensor_data_16bit(msg, sensor_id, x,y,z) != W_SUCCESS) {
        return W_FAILURE;
    }

    //we only care if its these two sensor type
    switch (sensor_id)
    {
    case DEM_3D_SENSOR_CANARD_IIS2MDC_MAG:
        
        break;
    
    case DEM_3D_SENSOR_CANARD_MTI630_MAG:
        break;
    }
    
        

}