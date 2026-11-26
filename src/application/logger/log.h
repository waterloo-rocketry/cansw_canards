#ifndef LOG_H
#define LOG_H

#include "rocketlib.h"
#include <stdint.h>
#include <string.h>
// Include headers for structs used in log_data_container_t
#include "application/health_checks/health_checks.h"
#include "application/sensor_handler/sensor_handler.h"
#include "common/gnc/gnc_types.h"
#include "drivers/altimu-10/altimu-10.h"
#include "log_definitions.h"

/* Size of a single buffer (bytes) */
#define LOG_BUFFER_SIZE 32768
// #define LOG_BUFFER_SIZE 16384
/* Size of each message region in error buffers (bytes) */
#define MAX_ERROR_MSG_LENGTH 32
/* Max number of optional arguments to LOG_ERROR, each stored in a 4-byte slot */
#define MAX_LOG_ERROR__ARGS 5
/* Size of the argument region in each error message (bytes) */
#define LOG_ERROR_VARIADIC_ARGUMENTS_LENGTH (MAX_LOG_ERROR__ARGS * sizeof(uint32_t))
/* Size of each message region in text buffers (bytes) */
#define MAX_TEXT_MSG_LENGTH 128
/**
 * Size of each message region in data buffers (bytes).
 * If changing this value, make sure to update it in scripts/logparse.py too!
 */
#define MAX_DATA_MSG_LENGTH 32
/* Number of message regions in a single error buffer */
#define ERROR_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_ERROR_MSG_LENGTH)
STATIC_ASSERT(ERROR_MSGS_PER_BUFFER == 1024,
              "Number of error logs per buffer was not correctly calculated");
/* Number of message regions in a single text buffer */
#define TEXT_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_TEXT_MSG_LENGTH)
STATIC_ASSERT(TEXT_MSGS_PER_BUFFER == 256,
              "Number of text logs per buffer was not correctly calculated");
/* Number of message regions in a single data buffer */
#define DATA_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_DATA_MSG_LENGTH)
STATIC_ASSERT(DATA_MSGS_PER_BUFFER == 1024,
              "Number of data logs per buffer was not correctly calculated");

/* Each log each be able to pack evenly within the buffer size */
STATIC_ASSERT(LOG_BUFFER_SIZE % MAX_ERROR_MSG_LENGTH == 0,
              "Error log message region size does not pack evenly into buffer size");
STATIC_ASSERT(LOG_BUFFER_SIZE % MAX_TEXT_MSG_LENGTH == 0,
              "Text log message region size does not pack evenly into buffer size");
STATIC_ASSERT(LOG_BUFFER_SIZE % MAX_DATA_MSG_LENGTH == 0,
              "Data log message region size does not pack evenly into buffer size");

/* Number of error log buffers <-- PLACEHOLDER VALUE, NEEDS REVIEWING */
#define NUM_ERROR_LOG_BUFFERS 1
/* Number of text log buffers */
#define NUM_TEXT_LOG_BUFFERS 2
/* Number of data log buffers */
#define NUM_DATA_LOG_BUFFERS 2

/* Size of filename string */
#define FILENAME_STRING_SIZE (8 + 1 + 3 + 1)

/**
 * Version number to identify post-flight how to parse data log messages.
 * Increment this value when making an incompatible change to log_data_type_t or log_data()'s
 * formatting of messages.
 *
 * Deprecated values: none
 */
#define LOG_DATA_FORMAT_VERSION 3

/**
 * Magic number to encode into log_data_type_t values: "DL" encoded as a little-endian 16-bit int.
 * If changing this value, make sure to update it in scripts/logparse.py too!
 */
#define LOG_DATA_MAGIC 0x4c44

/**
 * Place v in upper 16 bits and LOG_DATA_MAGIC in lower 16 bits.
 * If changing this macro, make sure to update it in scripts/logparse.py too!
 */
#define M(v) ((((v) & 0xffff) << 16) | LOG_DATA_MAGIC)

/**
 * All possible types of log messages emitted by log_data().
 * Make sure to update scripts/logparse.py too!
 *
 * Deprecated values: none
 */
typedef enum {
	/* Deprecated message type values for backwards compatibility
	LOG_TYPE_MOVELLA_READING = M(0x04),
	LOG_TYPE_ESTIMATOR_CTX = M(0x05),

	LOG_TYPE_POLOLU_READING = M(0x07),
	LOG_TYPE_POLOLU_RAW = M(0x08),
	LOG_TYPE_MOVELLA_PT4 = M(0x0A),
	*/

	// Message type values in use
	LOG_TYPE_HEADER = 0x44414548, // "HEAD" encoded as a little-endian 32-bit int
	LOG_TYPE_TEST = M(0x01),

	LOG_TYPE_NAVIGATOR_PT1 = M(0x02),
	LOG_TYPE_NAVIGATOR_PT2 = M(0x03),

	LOG_TYPE_CONTROLLER = M(0x04),

	LOG_TYPE_BOARD_IMU = M(0x05),

	LOG_TYPE_BOARD_MAG_BARO = M(0x06),

	LOG_TYPE_MOVELLA_PT1 = M(0x07),
	LOG_TYPE_MOVELLA_PT2 = M(0x08),
	LOG_TYPE_MOVELLA_PT3 = M(0x09),

	LOG_TYPE_AD_BREAKOUT = M(0x0b),

	LOG_TYPE_SERVO_MOTOR = M(0x0c)

	// Insert new types above this line in the format:
	// LOG_TYPE_XXX = M(unique_small_integer),
} log_data_type_t;

typedef enum {
	LOG_LVL_FATAL, // Errors (non-recoverable)
	LOG_LVL_WARN, // Warnings (recoverable issues)
	LOG_LVL_INFO, // Info (data from sensors, etc)
	LOG_LVL_DEBUG // Only for debugging on the ground
} log_level_t;

// Packed vector3d_f32_t for logging only
typedef union {
	float array[SIZE_VECTOR_3D];

	struct __attribute__((packed)) {
		float x;
		float y;
		float z;
	};
} vector3d_f32_packed_t;

#undef M

/**
 * The container for data to be included in messages from log_data().
 * Make sure to update format descriptions in scripts/logparse.py too!
 */
// Add structs for each type defined in log_data_type_t
// Please include `__attribute__((packed))` in struct declarations
typedef union __attribute__((packed)) {
	// LOG_TYPE_HEADER:
	struct __attribute__((packed)) {
		uint32_t version;
		uint32_t index;
	} header;

	// LOG_TYPE_TEST:
	struct __attribute__((packed)) {
		float test_val;
	} test;

	// LOG_TYPE_NAVIGATOR:
	struct __attribute__((packed)) {
		// quaternion_f32_t attitude;
		float orient_w;
		float orient_x;
		float orient_y;
		float orient_z;
		float altitude; // m
		float variance_norm;
	} navigator_pt1;

	struct __attribute__((packed)) {
		vector3d_f32_packed_t velocity; // m/s
		vector3d_f32_packed_t angular_velocity; // rad/sec
	} navigator_pt2;

	// LOG_TYPE_CONTROLLER:
	struct __attribute__((packed)) {
		float command; // rad
		float canard_coeff;
		float body_coeff;
		float roll_angle_target; // rad
		float roll_rate_target; // rad/s
	} controller;

	// LOG_TYPE_BOARD_IMU:
	struct __attribute__((packed)) {
		vector3d_f32_packed_t accelerometer; // m/s^2
		vector3d_f32_packed_t gyroscope; // rad/s
	} board_imu;

	// LOG_TYPE_BOARD_MAG_BARO:
	struct __attribute__((packed)) {
		vector3d_f32_packed_t magnetometer; // Gauss
		float barometer; // Pa
		float thermometer; // C
	} board_mag_baro;

	// LOG_TYPE_MOVELLA
	// note: dont use the all_imus_input_t struct here because packing isn't recursive
	struct __attribute__((packed)) {
		vector3d_f32_packed_t accelerometer; // m/s^2
		vector3d_f32_packed_t gyroscope; // rad/s
	} movella_pt1;

	struct __attribute__((packed)) {
		vector3d_f32_packed_t magnetometer; // Gauss
		float barometer; // Pa
	} movella_pt2;

	struct __attribute__((packed)) {
		float orient_w;
		float orient_x;
		float orient_y;
		float orient_z;
	} movella_pt3;

	// LOG_TYPE_AD_ACCEL:
	struct __attribute__((packed)) {
		vector3d_f32_packed_t accelerometer; // m/s^2
		float gyroscope; // rad/s
	} ad_breakout;

	// LOG_TYPE_SERVO_MOTOR:
	struct __attribute__((packed)) {
		float motor_angle; // deg
		float motor_current; // A
		float motor_temperature; // C
	} servo_motor;

} log_data_container_t;

// Log error site buffer contains information about previous buffer write
typedef struct {
	uint16_t last_logged_version;
	uint8_t last_logged_buffer;
	uint16_t last_logged_msg_num;
} log_error_site_t;

// MAX_DATA_MSG_LENGTH includes type and timestamp (8 bytes)
STATIC_ASSERT(sizeof(log_data_container_t) <= MAX_DATA_MSG_LENGTH - 8,
              "log_data_container_t must fit within MAX_DATA_MSG_LENGTH");

/**
 * @brief Create log buffers and mutexes necessary for logger operation.
 * @pre must be run after scheduler start, and after sd_card module is init
 */
w_status_t log_init(void);

/**
 * Use LOG_ERROR macro instead.
 */
w_status_t log_error(log_error_site_t *log_error_site, uint32_t timeout, log_file_id_t file_ID,
                     uint16_t line_number, log_error_id_t error_code,
                     const uint32_t args[MAX_LOG_ERROR__ARGS]);
/**
 * @brief Log a message in text form to the text log file.
 *
 * @param timeout Maximum amount of time to block waiting to log, in ms
 * @param source A string identifying the source of the log message
 * @param format The message to log, optionally specifying printf-like formatting for optional
 * variable arguments
 * @param ... Optional values to print according to format
 * @return Status indicating success or failure
 */
w_status_t log_text(uint32_t timeout, log_level_t level, const char *source, const char *format,
                    ...);

/**
 * @brief Log a message in binary form to the data log file.
 *
 * @param timeout Maximum amount of time to block waiting to log, in ms
 * @param type The type of data log message to write
 * @param data Pointer to raw data to write via memcpy
 * @return Status indicating success or failure
 */
w_status_t log_data(uint32_t timeout, log_data_type_t type, const log_data_container_t *data);

void log_task(void *argument);

/**
 * @brief Get and report the logger status for the health check system
 *
 * Gets the logger health status and logs relevant information.
 * Follows the module_get_status naming convention used by health_checks.
 *
 * @return CAN board status bitfield
 */
health_status_t logger_get_status(void);

/* Start of goofy log error variadic argument macro shenanigans*/

// LOG_ERROR argument packing helpers (internal, used by LOG_ERROR)
static inline uint32_t log_error_arg_u32(uint32_t v) {
	return v;
}

static inline uint32_t log_error_arg_f32(float f) {
	uint32_t u;
	memcpy(&u, &f, sizeof(u));
	return u;
}

// Convert one argument to its 4-byte slot value
#define LOG_ERROR_ARG(x)                                                                           \
	_Generic((x),                                                                                  \
	    float: log_error_arg_f32,                                                                  \
	    double: log_error_arg_f32,                                                                 \
	    default: log_error_arg_u32)(x)

// Pad the argument list with zeros so there are always MAX_LOG_ERROR__ARGS slots
#define LOG_ERROR_PACK(zero_parameter_case_slot, a, b, c, d, e, ...)                               \
	{LOG_ERROR_ARG(a), LOG_ERROR_ARG(b), LOG_ERROR_ARG(c), LOG_ERROR_ARG(d), LOG_ERROR_ARG(e)}
/**
 * @brief Log a message in an encoded form to the error log file.
 * 		  Requires #DEFINE LOG_FILE_ID at the top of the file
 * 		  Accepts up to MAX_LOG_ERROR__ARGS (5) arguments of up to 4 bytes each
 * 		  (ints, uints, chars, bools, floats, doubles are casted to a float).
 *
 * @param timeout Maximum amount of time to block waiting to log, in ms
 * @param error_id Error id which will be converted to a stirng
 * @param ... Optional values to print according to format in the error string
 * @return Status indicating success or failure
 */
#define LOG_ERROR(timeout, error_id, ...)                                                          \
	({                                                                                             \
		static log_error_site_t this_site = {0};                                                   \
		STATIC_ASSERT(sizeof((double[]){0, ##__VA_ARGS__}) <=                                      \
		                  (1 + MAX_LOG_ERROR__ARGS) * sizeof(double),                              \
		              "LOG_ERROR takes at most 5 arguments");                                      \
		const uint32_t log_error_args_[MAX_LOG_ERROR__ARGS] =                                      \
			LOG_ERROR_PACK(0, ##__VA_ARGS__, 0, 0, 0, 0, 0, 0);                                    \
		log_error(&this_site, timeout, LOG_FILE_ID, __LINE__, error_id, log_error_args_);          \
	})

#endif
