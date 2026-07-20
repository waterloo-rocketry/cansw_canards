#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"
#include <stdint.h>
// Include headers for structs used in log_data_container_t
#include "application/health_checks/health_checks.h"
#include "common/math/math.h"

/* Size of a single buffer (bytes) */
#define LOG_BUFFER_SIZE 32768
/* Size of each message region in text buffers (bytes) */
#define MAX_TEXT_MSG_LENGTH 128
/**
 * Size of each message region in data buffers (bytes).
 * If changing this value, make sure to update it in scripts/logparse.py too!
 */
#define MAX_DATA_MSG_LENGTH 32
/* Number of message regions in a single text buffer */
#define TEXT_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_TEXT_MSG_LENGTH)
/* Number of message regions in a single data buffer */
#define DATA_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_DATA_MSG_LENGTH)

#if LOG_BUFFER_SIZE % MAX_TEXT_MSG_LENGTH != 0
#warning "Text log message region size does not pack evenly into buffer size"
#endif
#if LOG_BUFFER_SIZE % MAX_DATA_MSG_LENGTH != 0
#warning "Data log message region size does not pack evenly into buffer size"
#endif

/* Number of text log buffers */
#define NUM_TEXT_LOG_BUFFERS 2
/* Number of data log buffers */
#define NUM_DATA_LOG_BUFFERS 2

/**
 * Version number to identify post-flight how to parse data log messages.
 * Increment this value when making an incompatible change to log_data_type_t or log_data()'s
 * formatting of messages.
 *
 * Deprecated values: none
 */
#define LOG_DATA_FORMAT_VERSION 2

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
	*/

	// Message type values in use
	LOG_TYPE_HEADER = 0x44414548, // "HEAD" encoded as a little-endian 32-bit int
	LOG_TYPE_TEST = M(0x01),

	LOG_TYPE_NAVIGATOR_PT1 = M(0x02),
	LOG_TYPE_NAVIGATOR_PT2 = M(0x03),

	LOG_TYPE_CONTROLLER = M(0x04),

	LOG_TYPE_BOARD_IMU = M(0x05),

	LOG_TYPE_BOARD_BAROMETER = M(0x06),

	LOG_TYPE_BOARD_MAG = M(0x07),

	LOG_TYPE_MOVELLA_PT1 = M(0x08),
	LOG_TYPE_MOVELLA_PT2 = M(0x09),
	LOG_TYPE_MOVELLA_PT3 = M(0x0A),
	LOG_TYPE_MOVELLA_PT4 = M(0x0B),

	LOG_TYPE_AD_ACCEL = M(0x0C),
	LOG_TYPE_AD_GYRO = M(0x0D),

	LOG_TYPE_SERVO_MOTOR = M(0x0E)

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
		// the 3 vars in roll_state_t
		float command; // deg
		float roll_target; // deg
		float canard_coeff;
	} controller;

	// LOG_TYPE_BOARD_IMU:
	struct __attribute__((packed)) {
		vector3d_f32_packed_t accelerometer; // m/s^2
		vector3d_f32_packed_t gyroscope; // rad/s
	} board_imu;

	// LOG_TYPE_BOARD_BAROMETER:
	struct __attribute__((packed)) {
		float barometer; // Pa
		float thermometer; // C
	} board_barometer;

	// LOG_TYPE_BOARD_MAG:
	struct __attribute__((packed)) {
		vector3d_f32_packed_t accelerometer; // m/s^2
		vector3d_f32_packed_t magnetometer; // Gauss
	} board_mag;

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
	} ad_accel;

	// LOG_TYPE_AD_GYRO:
	struct __attribute__((packed)) {
		float gyroscope; // rad/s
	} ad_gyro;

	// LOG_TYPE_SERVO_MOTOR:
	struct __attribute__((packed)) {
		float motor_angle; // deg
		float motor_current; // mA
		float motor_temperature; // C
	} servo_motor;

} log_data_container_t;

// MAX_DATA_MSG_LENGTH includes type and timestamp (8 bytes)
STATIC_ASSERT(sizeof(log_data_container_t) <= MAX_DATA_MSG_LENGTH - 8,
			  "log_data_container_t must fit within MAX_DATA_MSG_LENGTH");

/**
 * A collection of status variables describing the current health of the logger module.
 */
typedef struct {
	bool is_init;
	uint32_t dropped_txt_msgs;
	uint32_t dropped_data_msgs;
	uint32_t trunc_msgs;
	uint32_t full_buffer_moments;
	uint32_t log_write_timeouts;
	uint32_t invalid_region_moments;
	uint32_t crit_errs;
	uint32_t no_full_buf_moments;
	uint32_t buffer_flush_fails;
	uint32_t unsafe_buffer_flushes;
} logger_health_t;

/**
 * @brief Create log buffers and mutexes necessary for logger operation.
 * @pre must be run after scheduler start, and after sd_card module is init
 */
w_status_t log_init(void);

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

#endif
