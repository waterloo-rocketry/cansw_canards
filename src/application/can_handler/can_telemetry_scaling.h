#ifndef CAN_TELEMETRY_SCALING_H
#define CAN_TELEMETRY_SCALING_H

#include <stdint.h>

typedef enum {
	TYPE_INT8,
	TYPE_UINT8,
	TYPE_INT16,
	TYPE_UINT16,
	TYPE_INT24,
	TYPE_UINT24,
	TYPE_INT32,
	TYPE_UINT32,
	TYPE_COUNT
} can_types_t;


// A float sensor reading is packed into a fixed-width integer CAN field, but a
// float can also be NaN or +/-Inf. These are signalled in-band by reserving the
// top SENTINEL_COUNT codes of every integer type's range. The reserved value for
// a given type is: sentinel_value = type_max - SENTINEL_*.
//
// ENCODE (this firmware, can_handler.c): non-finite inputs store the matching
// sentinel; finite values are clamped to [type_min, type_max - SENTINEL_COUNT] so
// they can never alias a reserved code.
//
// DECODE: for a received raw integer R
// of a given type with maximum type_max,
//     R == type_max - 0  -> NaN
//     R == type_max - 1  -> +Inf
//     R == type_max - 2  -> -Inf
//     otherwise          -> finite value = R / scale

#define SENTINEL_NAN 0U // type_max - 0
#define SENTINEL_POS_INF 1U // type_max - 1
#define SENTINEL_NEG_INF 2U // type_max - 2
#define SENTINEL_COUNT 3U // number of reserved codes at the top of the range

typedef enum {
	SCALE_NAV_Q,
	SCALE_NAV_W,
	SCALE_NAV_V,
	SCALE_NAV_RX,
	SCALE_NAV_P,

	SCALE_CTRL_U,
	SCALE_CTRL_CL,
	SCALE_CTRL_PH,

	SCALE_IMU_A,
	SCALE_IMU_W,

	SCALE_BARO_P,
	SCALE_BARO_T,

	SCALE_COMP_M,

	SCALE_MTI_A,
	SCALE_MTI_W,
	SCALE_MTI_P,
	SCALE_MTI_M,

	SCALE_MTI_EST_E,
	SCALE_MTI_EST_W,
	SCALE_MTI_EST_V,
	SCALE_MTI_EST_RX,

	SCALE_AD_A,
	SCALE_AD_W,

	SCALE_SERVO_D,
	SCALE_SERVO_I,
	SCALE_SERVO_T,

	SCALE_COUNT
} can_scaling_types_t;

typedef struct {
	can_types_t type;
	uint16_t scale;
} can_scale_data_t;

#define SCALE_MAP_INIT                                                                             \
	{                                                                                              \
		[SCALE_NAV_Q] = {.type = TYPE_INT16, .scale = 10000},                                      \
		[SCALE_NAV_W] = {.type = TYPE_INT24, .scale = 1000},                                       \
		[SCALE_NAV_V] = {.type = TYPE_INT24, .scale = 1000},                                       \
		[SCALE_NAV_RX] = {.type = TYPE_UINT16, .scale = 1},                                        \
		[SCALE_NAV_P] = {.type = TYPE_UINT16, .scale = 1},                                         \
                                                                                                   \
		[SCALE_CTRL_U] = {.type = TYPE_INT16, .scale = 1000},                                      \
		[SCALE_CTRL_CL] = {.type = TYPE_INT16, .scale = 100},                                      \
		[SCALE_CTRL_PH] = {.type = TYPE_INT16, .scale = 1},                                        \
                                                                                                   \
		[SCALE_IMU_A] = {.type = TYPE_INT16, .scale = 1},                                          \
		[SCALE_IMU_W] = {.type = TYPE_INT16, .scale = 1},                                          \
                                                                                                   \
		[SCALE_BARO_P] = {.type = TYPE_INT24, .scale = 1},                                         \
		[SCALE_BARO_T] = {.type = TYPE_INT24, .scale = 1},                                         \
		[SCALE_COMP_M] = {.type = TYPE_INT16, .scale = 1},                                         \
                                                                                                   \
		[SCALE_MTI_A] = {.type = TYPE_INT16, .scale = 100},                                        \
		[SCALE_MTI_W] = {.type = TYPE_INT16, .scale = 100},                                        \
		[SCALE_MTI_P] = {.type = TYPE_UINT32, .scale = 1},                                         \
		[SCALE_MTI_M] = {.type = TYPE_INT16, .scale = 1000},                                       \
                                                                                                   \
		[SCALE_MTI_EST_E] = {.type = TYPE_INT16, .scale = 10000},                                  \
		[SCALE_MTI_EST_W] = {.type = TYPE_INT16, .scale = 10},                                     \
		[SCALE_MTI_EST_V] = {.type = TYPE_INT16, .scale = 10},                                     \
		[SCALE_MTI_EST_RX] = {.type = TYPE_UINT16, .scale = 1},                                    \
                                                                                                   \
		[SCALE_AD_A] = {.type = TYPE_INT16, .scale = 1},                                           \
		[SCALE_AD_W] = {.type = TYPE_INT24, .scale = 1},                                           \
                                                                                                   \
		[SCALE_SERVO_D] = {.type = TYPE_INT16, .scale = 1000},                                     \
		[SCALE_SERVO_I] = {.type = TYPE_UINT16, .scale = 1},                                       \
		[SCALE_SERVO_T] = {.type = TYPE_INT16, .scale = 1},                                        \
	}

#endif
