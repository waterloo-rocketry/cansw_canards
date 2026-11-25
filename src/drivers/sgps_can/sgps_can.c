#include "FreeRTOS.h"
#include "task.h"

#include "common/math/math.h"
#include "drivers/sgps_can/sgps_can.h"
#include "application/telemetry/telemetry.h"
#include "drivers/timer/timer.h"
#include "application/can_handler/can_telemetry_scaling.h"

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

typedef struct {
    uint32_t altitude;
    uint8_t daltitude;
}gps_altitude_t;


static vector3d_t IIS2MDC_MAG_buffer;
static uint32_t IIS2MDC_MAG_timestamp;
static bool IIS2MDC_is_valid = false;

static gps_timestamp_data_t gps_timestamp_buffer;
static uint32_t gps_timestamp_timestamp;
static bool gps_timestamp_is_valid = false;

static gps_altitude_t gps_altitude_buffer;
static uint32_t gps_altitude_timestamp;
static bool gps_altitude_is_valid = false;

static gps_position_data_t gps_lat_buffer;
static uint32_t gps_lat_timestamp;
static bool gps_lat_is_valid = false;

static gps_position_data_t gps_lon_buffer;
static uint32_t gps_lon_timestamp;
static bool gps_lon_is_valid = false;

static gps_info_t gps_info_buffer; 
static uint32_t gps_info_timestamp;
static bool gps_info_is_valid = false;

static baro_ms5611_t baro_ms5611_buffer;
static uint32_t baro_ms5611_timestamp;
static bool baro_ms5611_is_valid = false;


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
    if (DEM_3D_SENSOR_CANARD_IIS2MDC_MAG == sensor_id) {
        taskENTER_CRITICAL();
        IIS2MDC_MAG_buffer.x = (float64_t)((x - TELEMETRY_INT16_OFFSET) / can_scale_factor[SCALE_BOARD_MAG].scale);
        IIS2MDC_MAG_buffer.y = (float64_t)((y - TELEMETRY_INT16_OFFSET) / can_scale_factor[SCALE_BOARD_MAG].scale);
        IIS2MDC_MAG_buffer.z = (float64_t)((z - TELEMETRY_INT16_OFFSET) / can_scale_factor[SCALE_BOARD_MAG].scale);
        taskEXIT_CRITICAL();
        timer_get_ms(&IIS2MDC_MAG_timestamp);
        IIS2MDC_is_valid = true;
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
    timer_get_ms(&gps_timestamp_timestamp);
    gps_timestamp_is_valid = true;
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
    gps_altitude_buffer.altitude = altitude;
    gps_altitude_buffer.daltitude = daltitude;
    taskEXIT_CRITICAL();
    timer_get_ms(&gps_altitude_timestamp);
    gps_altitude_is_valid = true;
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
    gps_lat_buffer.degrees = lat_degrees;
    gps_lat_buffer.minutes = lat_minutes;
    gps_lat_buffer.dminutes_h = lat_dminutes_h;
    gps_lat_buffer.dir = lat_dir;
    taskEXIT_CRITICAL();
    timer_get_ms(&gps_lat_timestamp);
    gps_lat_is_valid = true;
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
    gps_lon_buffer.degrees = lon_degrees;
    gps_lon_buffer.minutes = lon_minutes;
    gps_lon_buffer.dminutes_h = lon_dminutes_h;
    gps_lon_buffer.dir = lon_dir;
    taskEXIT_CRITICAL();
    timer_get_ms(&gps_lon_timestamp);
    gps_lon_is_valid = true;
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
    timer_get_ms(&gps_info_timestamp);
    gps_info_is_valid = true;
    return W_SUCCESS;
}

/**
 * @brief callback function for the ms5611 barometer can message and writes message to buffer. 
 * @param msg incoming can message
 * @note board baro scaling is equal to 1
 * @return status of the operation
 */
static w_status_t MS5611_BARO_callback(const can_msg_t * msg) {
    can_dem_2d_sensor_id_t sensor_id;
    uint32_t baro_pres_pa = 0;
    uint32_t baro_temp_deg = 0;
    if (get_2d_analog_sensor_data_24bit(msg, &sensor_id, &baro_pres_pa, &baro_temp_deg)  != W_SUCCESS) {
        return W_FAILURE;
    }
    if (DEM_2D_SENSOR_CANARD_MS5611_BARO_TEMP == sensor_id) {
        taskENTER_CRITICAL();
        baro_ms5611_buffer.baro_pres_pa = (float32_t)baro_pres_pa;
        baro_ms5611_buffer.baro_temp_deg = (float32_t)baro_temp_deg;
        taskEXIT_CRITICAL();
        timer_get_ms(&baro_ms5611_timestamp);
        baro_ms5611_is_valid = true;
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


/**
 * @brief getter function for the IIS2MDC mag data
 * @param mag_data[out] latest magnetometer data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_canard_IIS2MDC_MAG(vector3d_t *mag_data, uint32_t *timestamp_ms) {

    if ((NULL == mag_data) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    if (!IIS2MDC_is_valid) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    mag_data->x = IIS2MDC_MAG_buffer.x;
    mag_data->y = IIS2MDC_MAG_buffer.y;
    mag_data->z = IIS2MDC_MAG_buffer.z;
    *timestamp_ms = IIS2MDC_MAG_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}


/**
 * @brief getter function for the gps timestamp
 * @param timestamp_data[out] latest gps timestamp data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_gps_timestamp(gps_timestamp_data_t *timestamp_data, uint32_t *timestamp_ms) {
    if ((NULL == timestamp_data) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    if (!gps_timestamp_is_valid) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    timestamp_data->utc_hours = gps_timestamp_buffer.utc_hours;
    timestamp_data->utc_minutes = gps_timestamp_buffer.utc_minutes;
    timestamp_data->utc_seconds = gps_timestamp_buffer.utc_seconds;
    timestamp_data->utc_dseconds = gps_timestamp_buffer.utc_dseconds;
    *timestamp_ms = gps_timestamp_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief getter function for the gps latitude
 * @param latitude[out] latest gps latitude data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_gps_latitude(gps_position_data_t *latitude, uint32_t *timestamp_ms) {
    if(!gps_lat_is_valid) {
        return W_FAILURE;
    }
    if ((NULL == latitude) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    latitude->degrees = gps_lat_buffer.degrees;
    latitude->minutes = gps_lat_buffer.minutes;
    latitude->dminutes_h = gps_lat_buffer.dminutes_h;
    latitude->dir = gps_lat_buffer.dir;
    *timestamp_ms = gps_lat_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS();
}

/**
 * @brief getter function for the gps longitude
 * @param longitude[out] latest gps longitude data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_gps_longitude(gps_position_data_t *longitude, uint32_t *timestamp_ms) {
    if(!gps_lon_is_valid) {
        return W_FAILURE;
    }
    if ((NULL == longitude) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    longitude->degrees = gps_lon_buffer.degrees;
    longitude->minutes = gps_lon_buffer.minutes;
    longitude->dminutes_h = gps_lon_buffer.dminutes_h;
    longitude->dir = gps_lon_buffer.dir;
    *timestamp_ms = gps_lon_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS();
}

/**
 * @brief getter function for the gps altitude
 * @param altitude[out] latest gps altitude data from buffer
 * @param daltitude [out] latest gps daltitude data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_gps_altitude(uint32_t *altitude, uint8_t *daltitude, uint32_t *timestamp_ms) {
    if (!gps_altitude_is_valid) {
        return W_FAILURE;
    }
    if ((NULL == altitude) || (NULL == daltitude) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    *altitude = gps_altitude_buffer.altitude;
    *daltitude = gps_altitude_buffer.daltitude;
    *timestamp_ms = gps_altitude_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief getter function for the gps info data
 * @param num_sat[out] latest number of satellite data from buffer
 * @param quality [out] latest quality data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_gps_info(uint8_t *num_sat, uint8_t *quality, uint32_t *timestamp_ms) {
    if (!gps_info_is_valid) {
        return W_FAILURE;
    }
    if ((NULL == num_sat) || (NULL == quality) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    *num_sat = gps_info_buffer.num_sat;
    *quality = gps_info_buffer.quality;
    *timestamp_ms = gps_info_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief getter function for the MS5611 Barometer
 * @param baro_pres_pa[out] latest barometer pressure  data from buffer
 * @param baro_temp_deg [out] latest barometer temperature data from buffer
 * @param timestamp_ms[out] timestamp of corresponding latest data
 * @return status of the operation
 */
w_status_t can_get_ms5611_baro(float32_t *baro_pres_pa, float32_t *baro_temp_deg, uint32_t *timestamp_ms) {
    if (!baro_ms5611_is_valid) {
        return W_FAILURE;
    }
    if ((NULL == baro_pres_pa) || (NULL == baro_temp_deg) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    *baro_pres_pa = baro_ms5611_buffer.baro_pres_pa;
    *baro_temp_deg = baro_ms5611_buffer.baro_temp_deg;
    *timestamp_ms = baro_ms5611_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}




