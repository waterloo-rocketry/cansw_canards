#include "FreeRTOS.h"

#include "common/math/math.h"
#include "drivers/can_sensor/can_sensor.h"

/*
callbacks needed:
SENSOR_3D_ANALOG16 -> IIS2MDC_MAG
GPS_TIMESTAMP 
GPS_LATITUDE 
GPS_LONGITUDE 
GPS_ALTITUDE 
GPS_INFO 
SENSOR_2D_ANALOG24 ->MS5611 Baro
*/

vector3d_t IIS2MDC_MAG_buffer;
gps_timestamp_data_t gps_timestamp_buffer;

static w_status_t IIS2MDC_MAG_callback(const can_msg_t * msg) {
    can_dem_3d_sensor_id_t sensor_id;
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t z = 0;
    if (get_3d_analog_sensor_data_16bit(msg, sensor_id, x,y,z)  != W_SUCCESS) {
        return W_FAILURE;
    }
    if (sensor_id == DEM_3D_SENSOR_CANARD_IIS2MDC_MAG) {
        IIS2MDC_MAG_buffer.x = x;
        IIS2MDC_MAG_buffer.y = y;
        IIS2MDC_MAG_buffer.z = z;
    }
    return W_SUCCESS;
}

static w_status_t GPS_TIMESTAMP_callback (const can_msg_t * msg) {
    uint8_t utc_hours = 0;
    uint8_t utc_minutes = 0;
    uint8_t utc_seconds = 0;
    uint8_t utc_dseconds = 0;
    if (get_gps_time(msg, utc_hours, utc_minutes, utc_seconds, utc_dseconds) != W_SUCCESS) {
        return W_FAILURE;
    }
    gps_timestamp_buffer.utc_hours = utc_hours;
    gps_timestamp_buffer.utc_minutes = utc_minutes;
    gps_timestamp_buffer.utc_seconds = utc_seconds;
    gps_timestamp_buffer.utc_dseconds = utc_dseconds;
    return W_SUCCESS;
}


