#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "canlib.h"
#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

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
 * @brief Handles a fatal system error by sending a CAN message.
 *
 * This function attempts to send a CAN message indicating the error and then
 * enters a safe, non-recoverable state (infinite loop with interrupts disabled).
 * It is designed to be called in critical failure scenarios where normal error
 * logging (e.g., to SD card) or task execution may not be possible.
 *
 * It uses the canlib library to send a DEBUG_RAW message with a coarse timestamp
 * and the first few characters of the error message.
 *
 * @param errorMsg A descriptive string for the error (only the first ~6 chars will be sent).
 */
void proc_handle_fatal_error(const char *errorMsg);

/**
 * @brief Report CAN handler module health status
 *
 * Retrieves and reports CAN error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
uint32_t can_handler_get_status(void);

#endif