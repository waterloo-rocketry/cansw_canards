#ifndef HBL3850_DRIVER_H
#define HBL3850_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"

#include "application/health_checks/health_checks.h"

// Motor IDs
typedef enum {
	HBL3850_MOTOR_1 = 0,
	HBL3850_MOTOR_2,
	HBL3850_MOTOR_3,
	HBL3850_MOTOR_4,
	HBL3850_MOTOR_COUNT
} hbl3850_motor_id_t;

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
 * circuit and temperature errors
 */
typedef struct {
	uint8_t node_health;
	uint8_t node_mode;
	uint32_t nodestatus_timestamp_ms;
	float32_t voltage_v;
	float32_t current_a;
	uint8_t circuit_err_flags;
	uint32_t circuit_timestamp_ms;
	float32_t temperature_c;
	uint8_t temp_err_flags;
	uint32_t temp_timestamp_ms;
} hbl3850_health_t;

/**
 * @brief Reassembly buffer for a multi-frame Actuator.Status transfer. Only the RX callback uses
 * it.
 */
typedef struct {
	uint8_t payload[8]; // the 8 Status payload bytes, filled in across both frames
	uint8_t len; // bytes received so far (0 means no transfer in progress)
	uint16_t crc; // transfer CRC from bytes 0-1 of frame 1
	uint8_t transfer_id; // from the tail byte, must match in frame 2
	bool expect_toggle; // toggle bit expected in the next frame
} hbl3850_rx_transfer_t;

/**
 * @brief Struct to represent individual motors. Instances would be created for each motor used.
 */
typedef struct {
	uint8_t node_id;
	uint8_t actuator_id;
	QueueHandle_t encoder_queue;
	hbl3850_health_t health;
	hbl3850_rx_transfer_t status_rx;
	volatile bool heard;
} hbl3850_motor_t;

/**
 * @brief Initialize hbl3850 driver. Start FDCAN peripheral
 * @param[in] motor Which motor (HBL3850_MOTOR_1..HBL3850_MOTOR_4)
 * @param[in] node_id Motor's DroneCAN node ID (1..127), set on the motor with the GUI Tool
 * @param[in] actuator_id Motor's actuator ID, used in ArrayCommand
 * @param[in] hfdcan Pointer to FDCAN handle struct for FDCAN peripheral
 * @param[in] can_init_timeout_ms Amount of time to wait to receive NodeStatus
 * @return W_SUCCESS on success, W_FAILURE on error, W_INVALID_PARAM if motor is invalid, node_id
 * is outside 1..127, node_id is already used by another motor, or the FDCAN handle is NULL
 */
w_status_t hbl3850_driver_init(hbl3850_motor_id_t motor, uint8_t node_id, uint8_t actuator_id,
                               FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms);

/**
 * @brief Send position command to motor. Clips position command to +/-60 degrees and converts it to
 * a unitless ArrayCommand value (between -1 and 1).
 * @param[in] motor Which motor (HBL3850_MOTOR_1..HBL3850_MOTOR_4)
 * @param[in] angle_deg Position command in degrees
 * @return W_SUCCESS on success, W_FAILURE on error, W_INVALID_PARAM if motor is invalid or
 * angle_deg is NaN
 */
w_status_t hbl3850_send_position_cmd(hbl3850_motor_id_t motor, float32_t angle_deg);

/**
 * @brief Copies motor encoder values stored in queue to an hbl3850_encoder_t struct variable
 * @param[in] motor Which motor (HBL3850_MOTOR_1..HBL3850_MOTOR_4)
 * @param[out] encoder Pointer to hbl3850_encoder_t struct variable
 * @return W_SUCCESS on success, W_FAILURE on error, W_INVALID_PARAM if motor is invalid
 */
w_status_t hbl3850_get_encoder(hbl3850_motor_id_t motor, hbl3850_encoder_t *encoder);

// HAL_FDCAN_RxFifo1Callback is declared by HAL

/**
 * @brief Get and report the HBL3850 status for the health check system. Built from the latest
 * NodeStatus, CircuitStatus and Temperature values.
 * @return CAN board status bitfield
 */
health_status_t hbl3850_get_status(void);

/**
 * @brief Copy the latest decoded health values (NodeStatus, CircuitStatus, Temperature) into
 * health_stats
 * @param[in] motor Which motor (HBL3850_MOTOR_1..HBL3850_MOTOR_4)
 * @param[out] health_stats Pointer to hbl3850_health_t struct variable
 * @return W_SUCCESS on success, W_FAILURE if health_stats is NULL or no health data received yet,
 * W_INVALID_PARAM if motor is invalid
 */
w_status_t hbl3850_get_health(hbl3850_motor_id_t motor, hbl3850_health_t *health_stats);

#endif // HBL3850_DRIVER_H
