/**
 * @file uart.h
 * @brief UART driver supporting message-based reception using IDLE line detection
 * @details Implements double-buffering to handle back-to-back messages efficiently
 */

#ifndef UART_H
#define UART_H

#include "FreeRTOS.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief Maximum message length for UART reception in bytes */
#define UART_MAX_LEN 256u

/** @brief Number of buffers in circular buffer arrangement */
#define UART_NUM_RX_BUFFERS 2 /* Theoretically expandable to N buffers */

/**
 * @brief Available UART channels in the system
 */
typedef enum {
	UART_MOVELLA, // Movella IMU
	UART_DEBUG_SERIAL, // debugger serial
	UART_CHANNEL_COUNT // Number of UART channels
} uart_channel_t;

/**
 * @brief Message descriptor for received UART data
 * @details Uses pointer to static buffer to avoid copying large messages
 */
typedef struct {
	uint8_t *data; /**< Pointer to data in static buffer pool */
	uint32_t len; /**< Message length in bytes */
	bool busy; /**< Buffer busy flag for double buffering */
} uart_msg_t;

/**
 * @brief Initialize UART for interrupt-driven reception
 * @param channel UART channel to initialize
 * @param huart HAL UART handle from CubeMX
 * @param timeout_ms Operation timeout in milliseconds (0 for default)
 * @return Status code indicating success or failure
 * @retval W_SUCCESS Initialization completed successfully
 * @retval W_INVALID_PARAM Invalid channel or NULL huart
 * @retval W_FAILURE Failed to create queue or register callbacks
 * @retval W_IO_ERROR Failed to start initial reception
 */
w_status_t uart_init(uart_channel_t channel, UART_HandleTypeDef *huart, uint32_t timeout_ms);

/**
 * @brief Read message from UART with timeout
 * @param channel UART channel to read from
 * @param buffer Buffer to store received data
 * @param length Pointer to store message length
 * @param timeout_ms Maximum time to wait for message
 * @return Status code indicating success or failure
 * @retval W_SUCCESS Message read successfully
 * @retval W_INVALID_PARAM Invalid parameters
 * @retval W_IO_TIMEOUT No message received within timeout
 * @note Message length will be truncated to UART_MAX_LEN if overflow occurs
 */
w_status_t uart_read(uart_channel_t channel, uint8_t *buffer, uint16_t *length,
					 uint32_t timeout_ms);

/**
 * @brief Write message to UART with timeout
 * @param channel UART channel to write from
 * @param buffer Buffer to store data
 * @param length uint to store message length
 * @param timeout_ms Maximum time to wait for message
 * @return Status code indicating success or failure
 * @retval W_SUCCESS Message written successfully
 * @retval W_INVALID_PARAM Invalid parameters
 * @retval W_IO_TIMEOUT Could not acquire the mutex or semaphore in the given time or
 * HAL_UART_Transmit_IT timeout
 * @retval W_IO_ERROR Call to HAL_UART_Transmit_IT failed
 * @note Message length will be truncated to UART_MAX_LEN if overflow occurs
 */
w_status_t uart_write(uart_channel_t channel, uint8_t *buffer, uint16_t length,
					  uint32_t timeout_ms);

/**
 * @brief Gets the current status of all UART channels
 * @return CAN board status bitfield
 * @details Logs the initialization status, operation statistics and error counters for all UART
 * channels in the system
 */
uint32_t uart_get_status(void);

#endif // UART_H