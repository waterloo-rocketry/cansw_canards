#include "FreeRTOS.h"

#include "common/math/math.h"
#include "drivers/can_sensor/can_sensor.h"
#include "application/telemetry/telemetry.h"

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

typedef struct {
    uint8_t num_sat;
    uint8_t quality;
}gps_info_t;

typedef struct {
    float32_t baro_pres_pa;
    float32_t baro_temp_deg;
}baro_ms5611_t;


vector3d_t IIS2MDC_MAG_buffer;
gps_timestamp_data_t gps_timestamp_buffer;
gps_position_data_t gps_position_buffer;
gps_info_t gps_info_buffer; 
baro_ms5611_t baro_ms5611_buffer;

//may need bools to keep track if buffer has actual data to prevent getter from reading before we write the first value

/**
 * @brief callback function for the IIS2MDC mag sensor; raised when corresponding can message is read and writes to buffer
 * @param msg incoming can message
 * @note the mag sensor has some encoding before being sent to the can, so we need to decode here
 * @return status of the operation
 */
static w_status_t IIS2MDC_MAG_callback(const can_msg_t * msg) {
    can_dem_3d_sensor_id_t sensor_id;
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t z = 0;
    if (get_3d_analog_sensor_data_16bit(msg, &sensor_id, &x, &y, &z)  != W_SUCCESS) {
        return W_FAILURE;
    }
    if (sensor_id == DEM_3D_SENSOR_CANARD_IIS2MDC_MAG) {
        taskENTER_CRITICAL();
        IIS2MDC_MAG_buffer.x = (float64_t)((x - TELEMETRY_INT16_OFFSET)/1000.0);
        IIS2MDC_MAG_buffer.y = (float64_t)((y - TELEMETRY_INT16_OFFSET)/1000.0);
        IIS2MDC_MAG_buffer.z = (float64_t)((z - TELEMETRY_INT16_OFFSET)/1000.0);
        taskEXIT_CRITICAL();
    }
    return W_SUCCESS;
}

/**
 * @brief callback function for the gps timesstamp can message and writes message to buffer.
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t GPS_TIMESTAMP_callback (const can_msg_t * msg) {
    uint8_t utc_hours = 0;
    uint8_t utc_minutes = 0;
    uint8_t utc_seconds = 0;
    uint8_t utc_dseconds = 0;
    if (get_gps_time(msg, &utc_hours, &utc_minutes, &utc_seconds, &utc_dseconds) != W_SUCCESS) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_timestamp_buffer.utc_hours = utc_hours;
    gps_timestamp_buffer.utc_minutes = utc_minutes;
    gps_timestamp_buffer.utc_seconds = utc_seconds;
    gps_timestamp_buffer.utc_dseconds = utc_dseconds;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief callback function for the gps altitude can message and writes message to buffer.
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t GPS_ALTITUDE_callback (const can_msg_t *msg) {
    uint32_t altitude = 0;
    uint8_t daltitude = 0;
    if (get_gps_alt(msg, &altitude, &daltitude) != W_SUCCESS) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_position_buffer.altitude = altitude;
    gps_position_buffer.daltitude = daltitude;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief callback function for the gps latitude can message and writes message to buffer.
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t GPS_LATITUDE_callback (const can_msg_t *msg) {
    uint8_t lat_degrees = 0;
	uint8_t lat_minutes = 0;
	uint16_t lat_dminutes_h = 0;
    uint8_t lat_dir = 0; 
    if (get_gps_lat(msg, &lat_degrees, &lat_minutes, &lat_dminutes_h, &lat_dir) != W_SUCCESS) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_position_buffer.lat_degrees = lat_degrees;
    gps_position_buffer.lat_minutes = lat_minutes;
    gps_position_buffer.lat_dminutes_h = lat_dminutes_h;
    gps_position_buffer.lat_dir = lat_dir;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief callback function for the gps longitude can message and writes message to buffer. 
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t GPS_LONGITUDE_callback (const can_msg_t *msg) {
    uint8_t lon_degrees = 0;
	uint8_t lon_minutes = 0;
	uint16_t lon_dminutes_h = 0;
    uint8_t lon_dir = 0; 
    if (get_gps_lon(msg, &lon_degrees, &lon_minutes, &lon_dminutes_h, &lon_dir) != W_SUCCESS) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_position_buffer.lon_degrees = lon_degrees;
    gps_position_buffer.lon_minutes = lon_minutes;
    gps_position_buffer.lon_dminutes_h = lon_dminutes_h;
    gps_position_buffer.lon_dir = lon_dir;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief callback function for the gps info can message and writes message to buffer. 
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t GPS_INFO_callback (const can_msg_t *msg) {
    uint8_t num_sat = 0;
    uint8_t quality = 0;

    if (get_gps_info(msg, &num_sat, &quality) != W_SUCCESS) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_info_buffer.num_sat = num_sat;
    gps_info_buffer.quality = quality;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief callback function for the ms5611 barometer can message and writes message to buffer. 
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t MS5611_BARO_callback(const can_msg_t * msg) {
    can_dem_2d_sensor_id_t sensor_id;
    uint32_t baro_pres_pa = 0;
    uint32_t baro_temp_deg = 0;
    if (get_2d_analog_sensor_data_24bit(msg, &sensor_id, &baro_pres_pa, &baro_temp_deg)  != W_SUCCESS) {
        return W_FAILURE;
    }
    if (sensor_id == DEM_2D_SENSOR_CANARD_MS5611_BARO_TEMP) {
        taskENTER_CRITICAL();
        baro_ms5611_buffer.baro_pres_pa = (float32_t)baro_pres_pa;
        baro_ms5611_buffer.baro_temp_deg = (float32_t)baro_temp_deg;
        taskEXIT_CRITICAL();
    }
    return W_SUCCESS;
}

/**
 * @brief initiates this current driver and registers all callback function with corresponding can message type
 * @param none
 * @return status of the operation
 */
w_status_t can_sensor_init() {
    if (can_handler_register_callback(MSG_SENSOR_3D_ANALOG16,IIS2MDC_MAG_callback) != W_SUCCESS ||      
        can_handler_register_callback(MSG_GPS_TIMESTAMP, GPS_TIMESTAMP_callback) != W_SUCCESS || 
        can_handler_register_callback(MSG_GPS_ALTITUDE, GPS_ALTITUDE_callback) != W_SUCCESS || 
        can_handler_register_callback(MSG_GPS_LATITUDE, GPS_LATITUDE_callback) != W_SUCCESS || 
        can_handler_register_callback(MSG_GPS_LONGITUDE, GPS_LONGITUDE_callback) != W_SUCCESS || 
        can_handler_register_callback(MSG_GPS_INFO, GPS_INFO_callback) != W_SUCCESS ||
        can_handler_register_callback(MSG_SENSOR_2D_ANALOG24, MS5611_BARO_callback) != W_SUCCESS) {
            return W_FAILURE;
    }
    return W_SUCCESS;
}





