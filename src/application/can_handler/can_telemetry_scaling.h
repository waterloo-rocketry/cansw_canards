#ifndef CAN_TELEMETRY_SCALING_H
#define CAN_TELEMETRY_SCALING_H

#include <stdint.h>

// A float sensor reading is packed into a fixed-width integer CAN field, but a
// float can also be NaN or +/-Inf. These are signalled in-band by reserving the
// top SENTINEL_COUNT codes of every integer type's range. The reserved value for
// a given type is: sentinel_value = type_max - SENTINEL_*.

#define SENTINEL_POS_INF 0U // type_max - 1
#define SENTINEL_NEG_INF 1U // type_max - 2

typedef enum {
	TYPE_INT16,
	TYPE_UINT16,
	TYPE_INT24,
	TYPE_INT32,
	TYPE_UINT32,
	TYPE_COUNT
} can_types_t;

typedef enum {
	SCALE_NAV_ORIENTATION,
	SCALE_NAV_ANGULAR_VELOCITY,
	SCALE_NAV_VELOCITY,
	SCALE_NAV_ALTITUDE,
	SCALE_NAV_VARIANCE_NORM,

	SCALE_CTRL_CMD,
	SCALE_CTRL_COEF_OF_ROLL_CTRL,
	SCALE_CTRL_ROLL_TARGET,

	SCALE_MTI_ACCEL,
	SCALE_MTI_GYRO,
	SCALE_MTI_PRESSURE,
	SCALE_MTI_MAG,

	SCALE_MTI_EST_QUATERNION,
	SCALE_MTI_EST_ANGULAR_VELOCITY,
	SCALE_MTI_EST_VELOCITY,

	SCALE_SERVO_ANGLE,
	SCALE_SERVO_CURRENT,
	SCALE_SERVO_TEMP,

	SCALE_COUNT
} can_scaling_types_t;

typedef struct {
	can_types_t type;
	uint16_t scale;
} can_scale_data_t;

static const can_scale_data_t can_scale_factor[SCALE_COUNT] = {
	[SCALE_NAV_ORIENTATION] = {.type = TYPE_INT16, .scale = 10000},
	// TODO: check with Tristan whether INT24 is necessary here. Using INT32 for now
	// because can_store_signed() stores INT24 as a full 4-byte int32 anyway (the 3-byte
	// packing isn't implemented), so INT24 would overflow a real 24-bit field
	[SCALE_NAV_ANGULAR_VELOCITY] = {.type = TYPE_INT24, .scale = 1000},
	[SCALE_NAV_VELOCITY] = {.type = TYPE_INT24, .scale = 1000},
	[SCALE_NAV_ALTITUDE] = {.type = TYPE_UINT16, .scale = 1},
	[SCALE_NAV_VARIANCE_NORM] = {.type = TYPE_UINT16, .scale = 1},

	[SCALE_CTRL_CMD] = {.type = TYPE_INT16, .scale = 1000},
	[SCALE_CTRL_COEF_OF_ROLL_CTRL] = {.type = TYPE_INT16, .scale = 100},
	[SCALE_CTRL_ROLL_TARGET] = {.type = TYPE_INT16, .scale = 100},

	[SCALE_MTI_ACCEL] = {.type = TYPE_INT16, .scale = 100},
	[SCALE_MTI_GYRO] = {.type = TYPE_INT16, .scale = 100},
	[SCALE_MTI_PRESSURE] = {.type = TYPE_UINT32, .scale = 1},
	[SCALE_MTI_MAG] = {.type = TYPE_INT16, .scale = 1000},

	[SCALE_MTI_EST_QUATERNION] = {.type = TYPE_INT16, .scale = 10000},
	[SCALE_MTI_EST_ANGULAR_VELOCITY] = {.type = TYPE_INT16, .scale = 10},
	[SCALE_MTI_EST_VELOCITY] = {.type = TYPE_INT16, .scale = 10},

	[SCALE_SERVO_ANGLE] = {.type = TYPE_INT16, .scale = 1000},
	[SCALE_SERVO_CURRENT] = {.type = TYPE_UINT16, .scale = 1},
	[SCALE_SERVO_TEMP] = {.type = TYPE_INT16, .scale = 1},
};

#endif
