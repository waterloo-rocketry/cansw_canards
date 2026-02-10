#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"
#include <stdint.h>
// Include headers for structs used in log_data_container_t
#include "application/controller/controller.h" // For controller_input_t, controller_output_t
#include "application/estimator/estimator.h" // For estimator_all_imus_input_t
#include "application/imu_handler/imu_handler.h"

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
#define LOG_DATA_FORMAT_VERSION 1

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
	LOG_TYPE_CANARD_CMD = M(0x02),
	LOG_TYPE_CONTROLLER_INPUT = M(0x03),

	LOG_TYPE_ENCODER = M(0x06),

	LOG_TYPE_MOVELLA_READING_PT1 = M(0x10),
	LOG_TYPE_MOVELLA_READING_PT2 = M(0x11),
	LOG_TYPE_MOVELLA_READING_PT3 = M(0x12),

	LOG_TYPE_ESTIMATOR_CTX_PT1 = M(0x13),
	LOG_TYPE_ESTIMATOR_CTX_PT2 = M(0x14),
	LOG_TYPE_ESTIMATOR_CTX_PT3 = M(0x15),

	LOG_TYPE_POLOLU_READING_PT1 = M(0x16),
	LOG_TYPE_POLOLU_READING_PT2 = M(0x17),
	LOG_TYPE_POLOLU_READING_PT3 = M(0x18),

	LOG_TYPE_POLOLU_RAW_PT1 = M(0x19),
	LOG_TYPE_POLOLU_RAW_PT2 = M(0x1A)

	// Insert new types above this line in the format:
	// LOG_TYPE_XXX = M(unique_small_integer),
} log_data_type_t;

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

	// LOG_TYPE_CANARD_CMD:
	struct __attribute__((packed)) {
		float cmd_angle;
		float ref_signal;
	} controller;

	// LOG_TYPE_CONTROLLER_INPUT:
	struct __attribute__((packed)) {
		// the 3 vars in roll_state_t
		float roll_angle;
		float roll_rate;
		float canard_angle;
		// Scheduling variables (flight condition)
		float pressure_dynamic;
		float canard_coeff;
	} controller_input_t;

	// LOG_TYPE_MOVELLA_READING or LOG_TYPE_POLOLU_READING:
	// note: dont use the all_imus_input_t struct here because packing isn't recursive
	struct __attribute__((packed)) {
		vector3d_f32_packed_t accelerometer; // m/s^2

	} imu_reading_pt1;

	struct __attribute__((packed)) {
		vector3d_f32_packed_t gyroscope; // rad/sec

	} imu_reading_pt2;

	struct __attribute__((packed)) {
		vector3d_f32_packed_t magnetometer; // mgauss (pololu) or arbitrary units (movella)
		float barometer; // Pa
		uint32_t timestamp_imu;
		bool is_dead;

	} imu_reading_pt3;

	// LOG_TYPE_ESTIMATOR_CTX:
	struct __attribute__((packed)) {
		// quaternion_f32_t attitude;
		float w;
		float x;
		float y;
		float z;

		float altitude;

	} estimator_ctx_pt1;

	struct __attribute__((packed)) {
		vector3d_f32_packed_t rates;
		float CL;
		float delta;

	} estimator_ctx_pt2;

	struct __attribute__((packed)) {
		vector3d_f32_packed_t velocity;
		float t;

	} estimator_ctx_pt3;

	// LOG_TYPE_ENCODER:
	struct __attribute__((packed)) {
		// radians. this is the value estimator module uses
		float angle_rad;
		bool is_dead;
	} encoder;

	// LOG_TYPE_POLOLU_RAW:
	struct __attribute__((packed)) {
		altimu_raw_imu_data_t raw_acc; // raw accelerometer data
		altimu_raw_imu_data_t raw_gyro; // raw gyroscope data

	} raw_pololu_data_pt1;

	struct __attribute__((packed)) {
		altimu_raw_imu_data_t raw_mag; // raw magnetometer data
		altimu_raw_baro_data_t raw_baro; // raw barometer data

	} raw_pololu_data_pt2;
} log_data_container_t;

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
w_status_t log_text(uint32_t timeout, const char *source, const char *format, ...);

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
uint32_t logger_get_status(void);

#endif
