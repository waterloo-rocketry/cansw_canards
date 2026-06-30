#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <stdint.h>

#include "canlib.h"
#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"

#include "application/can_handler/can_telemetry_scaling.h"
#include "application/health_checks/health_checks.h"

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
 * @brief CAN rx callback — software-filters incoming messages and queues
 *        those with a registered handler. Pass to stm32h7_can_init().
 * @param message Pointer to the received CAN message
 */
void can_handler_rx_message(const can_msg_t *message);

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
