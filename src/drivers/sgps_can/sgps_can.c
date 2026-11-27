#include "FreeRTOS.h"
#include "task.h"

#include "common/math/math.h"
#include "drivers/sgps_can/sgps_can.h"
#include "application/telemetry/telemetry.h"
#include "drivers/timer/timer.h"
#include "application/can_handler/can_telemetry_scaling.h"


typedef struct {
    uint8_t num_sat;
    uint8_t quality;
}gps_info_t;

typedef struct {
    float32_t baro_pressure_pa;
    float32_t baro_temp_deg;
}baro_ms5611_t;

typedef struct {
    uint32_t altitude;
    uint8_t daltitude;
}gps_altitude_t;


static vector3d_t IIS2MDC_MAG_buffer;
static uint32_t IIS2MDC_MAG_timestamp;
static bool IIS2MDC_is_valid = false;

static gps_utc_time_t gps_utc_time_buffer;
static uint32_t gps_utc_time_timestamp;
static bool gps_utc_time_is_valid = false;

static gps_altitude_t gps_altitude_buffer;
static uint32_t gps_altitude_timestamp;
static bool gps_altitude_is_valid = false;

static gps_latitude_data_t gps_lat_buffer;
static uint32_t gps_lat_timestamp;
static bool gps_lat_is_valid = false;

static gps_longitude_data_t gps_lon_buffer;
static uint32_t gps_lon_timestamp;
static bool gps_lon_is_valid = false;

static gps_info_t gps_info_buffer; 
static uint32_t gps_info_timestamp;
static bool gps_info_is_valid = false;

static baro_ms5611_t baro_ms5611_buffer;
static uint32_t baro_ms5611_timestamp;
static bool baro_ms5611_is_valid = false;

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
        if (W_SUCCESS != timer_get_ms(&IIS2MDC_MAG_timestamp)) {
            return W_FAILURE;
        }
        taskENTER_CRITICAL();
        IIS2MDC_MAG_buffer.x = (float64_t)((x - TELEMETRY_INT16_OFFSET)) / can_scale_factor[SCALE_BOARD_MAG].scale;
        IIS2MDC_MAG_buffer.y = (float64_t)((y - TELEMETRY_INT16_OFFSET)) / can_scale_factor[SCALE_BOARD_MAG].scale;
        IIS2MDC_MAG_buffer.z = (float64_t)((z - TELEMETRY_INT16_OFFSET)) / can_scale_factor[SCALE_BOARD_MAG].scale;
        taskEXIT_CRITICAL();
        IIS2MDC_is_valid = true;
    }
    return W_SUCCESS;
}

/**
 * @brief callback function for the gps timesstamp can message and writes message to buffer.
 * @param msg incoming can message
 * @return status of the operation
 */
static w_status_t GPS_UTC_TIME_callback (const can_msg_t * msg) {
    uint8_t utc_hours = 0;
    uint8_t utc_minutes = 0;
    uint8_t utc_seconds = 0;
    uint8_t utc_dseconds = 0;
    if (get_gps_time(msg, &utc_hours, &utc_minutes, &utc_seconds, &utc_dseconds) != W_SUCCESS) {
        return W_FAILURE;
    }
    if (W_SUCCESS != timer_get_ms(&gps_utc_time_timestamp)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_utc_time_buffer.utc_hours = utc_hours;
    gps_utc_time_buffer.utc_minutes = utc_minutes;
    gps_utc_time_buffer.utc_seconds = utc_seconds;
    gps_utc_time_buffer.utc_dseconds = utc_dseconds;
    taskEXIT_CRITICAL();
    gps_utc_time_is_valid = true;
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
    if (W_SUCCESS != timer_get_ms(&gps_altitude_timestamp)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_altitude_buffer.altitude = altitude;
    gps_altitude_buffer.daltitude = daltitude;
    taskEXIT_CRITICAL();

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
    if (W_SUCCESS != timer_get_ms(&gps_lat_timestamp)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_lat_buffer.degrees = lat_degrees;
    gps_lat_buffer.minutes = lat_minutes;
    gps_lat_buffer.dminutes_h = lat_dminutes_h;
    gps_lat_buffer.dir = lat_dir;
    taskEXIT_CRITICAL();
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
    if (W_SUCCESS != timer_get_ms(&gps_lon_timestamp)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_lon_buffer.degrees = lon_degrees;
    gps_lon_buffer.minutes = lon_minutes;
    gps_lon_buffer.dminutes_h = lon_dminutes_h;
    gps_lon_buffer.dir = lon_dir;
    taskEXIT_CRITICAL();
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
    if (W_SUCCESS != timer_get_ms(&gps_info_timestamp)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    gps_info_buffer.num_sat = num_sat;
    gps_info_buffer.quality = quality;
    taskEXIT_CRITICAL();
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
    uint32_t baro_pressure_pa = 0;
    uint32_t baro_temp_deg = 0;
    if (get_2d_analog_sensor_data_24bit(msg, &sensor_id, &baro_pressure_pa, &baro_temp_deg)  != W_SUCCESS) {
        return W_FAILURE;
    }
    if (DEM_2D_SENSOR_CANARD_MS5611_BARO_TEMP == sensor_id) {
        if (W_SUCCESS != timer_get_ms(&baro_ms5611_timestamp)) {
            return W_FAILURE;
        }
        taskENTER_CRITICAL();
        baro_ms5611_buffer.baro_pressure_pa = (float32_t)baro_pressure_pa;
        baro_ms5611_buffer.baro_temp_deg = (float32_t)baro_temp_deg;
        taskEXIT_CRITICAL();
        baro_ms5611_is_valid = true;
    }
    return W_SUCCESS;
}

/**
 * @brief initiates this current driver and registers all callback function with corresponding can message type
 * @param none
 * @return status of the operation
 */
w_status_t sgps_can_init(void) {
    if (can_handler_register_callback(MSG_SENSOR_3D_ANALOG16,IIS2MDC_MAG_callback) != W_SUCCESS ||      
        can_handler_register_callback(MSG_GPS_TIMESTAMP, GPS_UTC_TIME_callback) != W_SUCCESS || 
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
 * @brief read the freshest magnetometer vector provided over CAN
 * @param[out] mag_data outputs the three-axis magnetic-field reading
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_IIS2MDC(vector3d_t *mag_data, uint32_t *timestamp_ms) {

    if ((NULL == mag_data) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    if (!IIS2MDC_is_valid) {
        return W_NO_DATA;
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
 * @brief reads the latest gps UTC time received over CAN
 * @param[out] utc_time_data  outputs the GPS UTC time data
 * @param[out] timestamp_ms  outputs the timestamp of when we first recived the message over CAN
 * @note the utc_time_data gives out the time from the gps board while the timestamp is the time of the board that this function is running off
 * @return status of the operation
 */
w_status_t can_get_gps_utc_time(gps_utc_time_t *utc_time_data, uint32_t *timestamp_ms) {
    if ((NULL == utc_time_data) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    if (!gps_utc_time_is_valid) {
        return W_NO_DATA;
    }
    taskENTER_CRITICAL();
    utc_time_data->utc_hours = gps_utc_time_buffer.utc_hours;
    utc_time_data->utc_minutes = gps_utc_time_buffer.utc_minutes;
    utc_time_data->utc_seconds = gps_utc_time_buffer.utc_seconds;
    utc_time_data->utc_dseconds = gps_utc_time_buffer.utc_dseconds;
    *timestamp_ms = gps_utc_time_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief read freshest GPS latitude in the degrees/minutes representation of the CAN message
 * @param[out] latitude outputs latitude in degrees, minutes, deciminutes, and direction
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_gps_latitude(gps_latitude_data_t *latitude, uint32_t *timestamp_ms) {
    if(!gps_lat_is_valid) {
        return W_FAILURE;
    }
    if ((NULL == latitude) || (NULL == timestamp_ms)) {
        return W_NO_DATA;
    }
    taskENTER_CRITICAL();
    latitude->degrees = gps_lat_buffer.degrees;
    latitude->minutes = gps_lat_buffer.minutes;
    latitude->dminutes_h = gps_lat_buffer.dminutes_h;
    latitude->dir = gps_lat_buffer.dir;
    *timestamp_ms = gps_lat_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}

/**
 * @brief read freshest GPS longitude in the degrees/minutes representation of the CAN message
 * @param[out] longitude outputs longitude in degrees, minutes, deciminutes, and direction
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_gps_longitude(gps_longitude_data_t *longitude, uint32_t *timestamp_ms) {
    if(!gps_lon_is_valid) {
        return W_NO_DATA;
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
    return W_SUCCESS;
}

/**
 * @brief read freshest GPS altitude in the degrees/minutes representation of the CAN message
 * @param[out] altitude outputs gps altitude
 * @param[out] daltitude  outputs the gps deci-altitude
 * @param[out] timestamp_ms  outputs the timestamp of when we first recived the message over CAN
 * @note there is lacking documentation on the daltitude data 
 * @return status of the operation
 */
w_status_t can_get_gps_altitude(uint32_t *altitude, uint8_t *daltitude, uint32_t *timestamp_ms) {
    if (!gps_altitude_is_valid) {
        return W_NO_DATA;
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
 * @brief read freshest gps info received over CAN
 * @param[out] num_sat  outputs the number of satellites
 * @param[out] quality outputs the quality of the GPS fix
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */ 
w_status_t can_get_gps_info(uint8_t *num_sat, uint8_t *quality, uint32_t *timestamp_ms) {
    if (!gps_info_is_valid) {
        return W_NO_DATA;
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
 * @brief read freshest barometric pressure and temperature received over CAN
 * @param[out] baro_pressure_pa outputs barometer pressure in Pa
 * @param[out] baro_temp_deg outputs barometer temperature in degrees Celsius
 * @param[out] timestamp_ms outputs the timestamp of when we first recived the message over CAN
 * @return status of the operation
 */
w_status_t can_get_ms5611(float32_t *baro_pressure_pa, float32_t *baro_temp_deg, uint32_t *timestamp_ms) {
    if (!baro_ms5611_is_valid) {
        return W_NO_DATA;
    }
    if ((NULL == baro_pressure_pa) || (NULL == baro_temp_deg) || (NULL == timestamp_ms)) {
        return W_FAILURE;
    }
    taskENTER_CRITICAL();
    *baro_pressure_pa = baro_ms5611_buffer.baro_pressure_pa;
    *baro_temp_deg = baro_ms5611_buffer.baro_temp_deg;
    *timestamp_ms = baro_ms5611_timestamp;
    taskEXIT_CRITICAL();
    return W_SUCCESS;
}




