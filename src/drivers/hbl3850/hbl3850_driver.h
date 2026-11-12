#ifndef HBL3850_DRIVER_H
#define HBL3850_DRIVER_H

#include "FreeRTOS.h"
#include "application/health_checks/health_checks.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// CircuitStatus error flags
typedef enum {
	HBL3850_CIRCUIT_ERR_NONE = 0,
	HBL3850_CIRCUIT_ERR_OVERVOLTAGE = 1,
	HBL3850_CIRCUIT_ERR_UNDERVOLTAGE = 2,
	HBL3850_CIRCUIT_ERR_OVERCURRENT = 4,
	HBL3850_CIRCUIT_ERR_OVERCURRENT_PROTECTION = 8
} hbl3850_circuit_err_t;

// Temperature error flags
typedef enum {
	HBL3850_TEMP_ERR_NONE = 0,
	HBL3850_TEMP_ERR_OVERHEATING = 1,
	HBL3850_TEMP_ERR_OVERCOOLING = 2
} hbl3850_temp_err_t;

/**
 * @brief Decoded encoder values from Motor encoder
 */
typedef struct {
	float32_t position_deg;
	float32_t speed_dps;
	uint32_t timestamp_ms;
} hbl3850_encoder_t;

/**
 * @brief (Decoded) health values from NodeStatus, CircuitStatus and Temperature. Also includes
 * circuit and temperature errors;
 */
typedef struct {
	bool node_status_ok;
	uint32_t nodestatus_timestamp_ms;
	float32_t voltage_v;
	float32_t current_a;
	uint8_t circuit_err_flags;
	uint32_t circuit_timestamp_ms;
	int8_t temperature_c;
	uint8_t temp_err_flags;
	uint32_t temp_timestamp_ms;
} hbl3850_health_t;

/**
 * @brief Initialize hbl3850 driver. Start FDCAN peripheral
 * @param[in] hfdcan Pointer to FDCAN handle struct for FDCAN peripheral
 * @param[in] can_init_timeout_ms Amount of time to wait to receive NodeStatus
 * @return W_SUCCESS on success and W_FAILURE on error. If FDCAN handle is null W_INVALID_PARAM.
 */
w_status_t hbl3850_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms);

/**
 * @brief Send position command to motor. Clips position command to +/-60 degrees and converts it to
 * a unitless ArrayCommand value (between -1 and 1).
 * @param[in] angle_deg Position command in degrees
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t hbl3850_send_position_cmd(float32_t angle_deg);

/**
 * @brief Copies motor encoder values stored in queue to an hbl3850_encoder_t struct variable
 * @param[out] out Pointer to hbl3850_encoder_t struct variable
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t hbl3850_get_encoder(hbl3850_encoder_t *out);
// HAL_FDCAN_RxFifo1Callback is declared by HAL

/**
 * @brief Get and report the HBL3850 status for the health check system. Built from the latest
 * NodeStatus, CircuitStatus and Temperature values.
 * @return CAN board status bitfield
 */
health_status_t hbl3850_get_status(void);

/**
 * @brief Copy the latest decoded health values (NodeStatus, CircuitStatus, Temperature) into out
 * @param[out] out Pointer to hbl3850_health_t struct variable
 * @return W_SUCCESS on success, W_FAILURE if out is NULL or no health data received yet
 */
w_status_t hbl3850_get_health(hbl3850_health_t *out);
#endif // HBL3850_DRIVER_H
