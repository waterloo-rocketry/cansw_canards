#ifndef HBL3850_DRIVER_H
#define HBL3850_DRIVER_H

#include "FreeRTOS.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "application/health_checks/health_checks.h"

typedef struct {
    float32_t position_deg;
    float32_t speed_dps;
    uint32_t timestamp_ms;
} hbl3850_encoder_t;

typedef struct {
    float32_t voltage_v;
    float32_t current_a;
    uint8_t circuit_err_flags;
    uint32_t circuit_timestamp_ms;
    int8_t temperature_c;
    uint8_t temp_err_flags;
    uint32_t temp_timestamp_ms;
} hbl3850_errors_t;

w_status_t hbl3850_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms);
w_status_t hbl3850_send_position_cmd(float32_t angle_deg);
w_status_t hbl3850_get_encoder(hbl3850_encoder_t *out);
// HAL_FDCAN_RxFifo1Callback is declared by HAL
health_status_t hbl3850_get_status(void);
w_status_t hbl3850_get_errors(hbl3850_errors_t *out);
#endif // HBL3850_DRIVER_H