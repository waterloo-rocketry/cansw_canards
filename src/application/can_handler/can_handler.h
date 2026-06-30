#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <stdint.h>

#include "canlib.h"
#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"

#include "application/can_handler/can_telemetry_scaling.h"
#include "application/health_checks/health_checks.h"

#define UINT24_MAX (1U << 24) - 1U
#define INT24_MIN -(1 << 23)
#define INT24_MAX (1 << 23) - 1

/**
 * @brief Structure to track CAN handler stats, errors and status
 */
typedef struct {
	bool initialized; /**< Initialization status flag */
	uint32_t dropped_rx_counter; /**< Number of dropped RX messages from rx isr */
	uint32_t dropped_tx_counter; /**< Number of dropped TX messages from tx queue */
	uint32_t tx_failures; /**< Number of transmission failures */
	uint32_t rx_callback_errors; /**< Number of RX callback execution errors */
	uint32_t rx_timeouts; /**< Number of RX queue timeouts */
	uint32_t tx_timeouts; /**< Number of TX queue timeouts */
	uint32_t messages_sent; /**< Number of messages successfully sent */
	uint32_t messages_received; /**< Number of messages successfully received */
	uint32_t encode_overflow; /**< Encoded values above range (-> type_min) */
	uint32_t encode_underflow; /**< Encoded values below range (-> type_max) */
	uint32_t encode_pos_inf; /**< +Inf float inputs encoded (-> type_min) */
	uint32_t encode_neg_inf; /**< -Inf float inputs encoded (-> type_max) */
	uint32_t encode_nan; /**< NaN float inputs dropped */
} can_handler_status_t;

// Signature for rx callback functions
typedef w_status_t (*can_callback_t)(const can_msg_t *);

/**
 * @brief Initializer to setup queues and canlib
 * @return Status of the operation
 */
w_status_t can_handler_init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Used to send a can message
 * @param message Pointer to the message to write
 * @return Status of the operation
 */
w_status_t can_handler_transmit(const can_msg_t *message);

/**
 * @brief Binds a callback which will be triggered when we recieve any messages of a particular type
 * @param msg_type The canlib message type to register the callback for
 * @param callback Pointer to the callback function to use
 * @return Status of the operation
 */
w_status_t can_handler_register_callback(can_msg_type_t msg_type, can_callback_t callback);

/**
 * @brief When busqueue_rx recieves a message, this task calls the corresponding callback
 */
void can_handler_task_rx(void *argument);

/**
 * @brief When busqueue_tx recieves a message, this task sends it to the can bus
 */
void can_handler_task_tx(void *argument);

/**
 * @brief Encodes a float telemetry value into an integer representation according to predefined
 * scaling rules.
 *
 * Out-of-range readings map to the target type's bounds with an inverted sentinel
 * scheme (no wraparound), using the full integer range (see can_telemetry_scaling.h
 * for the encode/decode contract):
 * +Inf or above-range -> type_min,
 * -Inf or below-range -> type_max.
 * NaN has no representation and is dropped: nothing is written to out and W_MATH_ERROR
 * is returned.
 *
 * @param sensor The predefined scaling rule to apply (defined in can_telemetry_scaling.h)
 * @param input The raw telemetry integer value to encode
 * @param out Pointer to the output variable where the encoded value will be stored
 *
 * @return w_status_t indicating success or type of failure
 */
w_status_t can_encode_scaled_float(can_scaling_types_t sensor, float input, void *out);

/**
 * @brief Encodes an integer telemetry value into an integer representation according to predefined
 * scaling rules.
 *
 * @param sensor The predefined scaling rule to apply (defined in can_telemetry_scaling.h)
 * @param input The raw telemetry value to encode
 * @param out Pointer to the output variable where the encoded value will be stored
 *
 * @return w_status_t indicating success or type of failure
 */
w_status_t can_encode_scaled_int(can_scaling_types_t sensor, int64_t input, void *out);

/**
 * @brief Report CAN handler module health status
 *
 * Retrieves and reports CAN error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t can_handler_get_status(void);

#endif
