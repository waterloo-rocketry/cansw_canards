#ifndef CAN_TELEMETRY_SCALING_H
#define CAN_TELEMETRY_SCALING_H

#include <stdint.h>

typedef enum
{
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
    int64_t scale;
} can_scale_data_t;

const can_scale_data_t scale_map[SCALE_COUNT] = {
    [SCALE_NAV_Q] =         {.type = TYPE_INT16,    .scale = 10000},
    [SCALE_NAV_W] =         {.type = TYPE_INT16,    .scale = 10},
    [SCALE_NAV_V] =         {.type = TYPE_INT16,    .scale = 10},
    [SCALE_NAV_RX] =        {.type = TYPE_UINT16,   .scale = 1},
    [SCALE_NAV_P] =         {.type = TYPE_UINT16,   .scale = 1},

    [SCALE_CTRL_U] =        {.type = TYPE_INT16,    .scale = 1000},
    [SCALE_CTRL_CL] =       {.type = TYPE_INT16,    .scale = 100},
    [SCALE_CTRL_PH] =       {.type = TYPE_INT16,    .scale = 1},

    [SCALE_IMU_A] =         {.type = TYPE_INT16,    .scale = 1},
    [SCALE_IMU_W] =         {.type = TYPE_INT16,    .scale = 1},

    [SCALE_BARO_P] =        {.type = TYPE_INT24,    .scale = 1},
    [SCALE_BARO_T] =        {.type = TYPE_INT24,    .scale = 1},

    [SCALE_COMP_M] =        {.type = TYPE_INT16,    .scale = 1},

    [SCALE_MTI_A] =         {.type = TYPE_INT16,    .scale = 100},
    [SCALE_MTI_W] =         {.type = TYPE_INT16,    .scale = 100},
    [SCALE_MTI_P] =         {.type = TYPE_UINT32,   .scale = 1},
    [SCALE_MTI_M] =         {.type = TYPE_INT24,    .scale = 1000},

    [SCALE_MTI_EST_E] =     {.type = TYPE_INT16,    .scale = 10000},
    [SCALE_MTI_EST_W] =     {.type = TYPE_INT16,    .scale = 10},
    [SCALE_MTI_EST_V] =     {.type = TYPE_INT16,    .scale = 10},
    [SCALE_MTI_EST_RX] =    {.type = TYPE_UINT16,   .scale = 1},

    [SCALE_AD_A] =          {.type = TYPE_INT24,    .scale = 1},
    [SCALE_AD_W] =          {.type = TYPE_INT24,    .scale = 1},

    [SCALE_SERVO_D] =       {.type = TYPE_INT16,    .scale = 1000},
    [SCALE_SERVO_I] =       {.type = TYPE_UINT8,    .scale = 1},
    [SCALE_SERVO_T] =       {.type = TYPE_INT8,     .scale = 1},
};

#endif
