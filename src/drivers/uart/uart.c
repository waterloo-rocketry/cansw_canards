/**
 * @file uart.c
 * @brief Implementation of UART driver with IDLE line detection
 */

#include "drivers/uart/uart.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <string.h>
/* Static buffer pool for all channels */
static uint8_t s_buffer_pool[UART_CHANNEL_COUNT][UART_MAX_LEN * UART_NUM_RX_BUFFERS];

/**
 * @brief Internal handle structure for UART channel state
 */
typedef struct {
    UART_HandleTypeDef *huart; /**< HAL UART handle */
    uint32_t timeout_ms; /**< Operation timeout */
    uart_msg_t rx_msgs[UART_NUM_RX_BUFFERS]; /* Array of N message buffers */
    uint8_t curr_buffer_num; /**< Index in circular buffer array */
    QueueHandle_t msg_queue; /**< Queue for message pointers */

    SemaphoreHandle_t
        transfer_complete; // Communicate transfer complete between uart_write and the ISR
    SemaphoreHandle_t write_mutex; // Allows tasks to line up to use uart_write

} uart_handle_t;

/** @brief Array of UART channel handles */
static uart_handle_t s_uart_handles[UART_CHANNEL_COUNT] = {0};

/**
 * @brief Error statistics structure
 */
typedef struct {
    bool initialized; /**< Whether UART is initialized */
    uint32_t overflows; /**< Count of message size overflows */
    uint32_t timeouts; /**< Count of operation timeouts */
    uint32_t hw_errors; /**< Count of hardware errors */
    uint32_t messages_received; /**< Count of messages successfully received */
    uint32_t messages_sent; /**< Count of messages successfully sent */
} uart_stats_t;

/** @brief Error statistics for each channel */
static uart_stats_t s_uart_stats[UART_CHANNEL_COUNT] = {0};

/**
 * @brief Initialize UART channel
 * @param channel UART channel to initialize
 * @param huart HAL UART handle
 * @param timeout_ms Operation timeout in milliseconds
 * @return Status of the initialization
 */
w_status_t uart_init(uart_channel_t channel, UART_HandleTypeDef *huart, uint32_t timeout_ms) {
    if ((channel >= UART_CHANNEL_COUNT) || (NULL == huart)) {
        return W_INVALID_PARAM;
    }

    /* Get handle for this channel and clear it to known state */
    uart_handle_t *handle = &s_uart_handles[channel];
    memset(handle, 0, sizeof(*handle));
    handle->huart = huart;
    handle->timeout_ms = timeout_ms;

    // Init semaphores/mutexes
    handle->write_mutex = xSemaphoreCreateMutex();
    handle->transfer_complete = xSemaphoreCreateBinary();
    if ((NULL == handle->transfer_complete) || (NULL == handle->write_mutex)) {
        vSemaphoreDelete(handle->write_mutex);
        vSemaphoreDelete(handle->transfer_complete);
        log_text(10, "uart", "initmtxfail %d", channel);
        return W_FAILURE;
    }

    /* Initialize N message buffers in circular buffer arrangement */
    for (int i = 0; i < UART_NUM_RX_BUFFERS; i++) {
        handle->rx_msgs[i].data = &s_buffer_pool[channel][i * UART_MAX_LEN];
        handle->rx_msgs[i].len = 0;
        handle->rx_msgs[i].busy = false;
    }

    // Create queue for message pointers
    handle->msg_queue = xQueueCreate(1, sizeof(uart_msg_t *));
    if (NULL == handle->msg_queue) {
        vSemaphoreDelete(handle->write_mutex);
        vSemaphoreDelete(handle->transfer_complete);
        log_text(10, "uart", "initqfail %d", channel);
        return W_FAILURE;
    }

    // Register callbacks with appropriate types
    HAL_StatusTypeDef hal_status;
    hal_status = HAL_UART_RegisterRxEventCallback(huart, HAL_UARTEx_RxEventCallback);
    if (hal_status != HAL_OK) {
        vQueueDelete(handle->msg_queue);
        vSemaphoreDelete(handle->write_mutex);
        vSemaphoreDelete(handle->transfer_complete);
        log_text(10, "uart", "initcbfail %d", channel);
        return W_FAILURE;
    }

    hal_status = HAL_UART_RegisterCallback(
        huart, HAL_UART_ERROR_CB_ID, (pUART_CallbackTypeDef)HAL_UART_ErrorCallback
    );
    if (hal_status != HAL_OK) {
        vQueueDelete(handle->msg_queue);
        vSemaphoreDelete(handle->write_mutex);
        vSemaphoreDelete(handle->transfer_complete);
        log_text(10, "uart", "initfail %d", channel);
        return W_FAILURE;
    }

    // Register the transmit-complete ISR for this UART channel
    hal_status =
        HAL_UART_RegisterCallback(huart, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART_TxCpltCallback);
    if (hal_status != HAL_OK) {
        vQueueDelete(handle->msg_queue);
        vSemaphoreDelete(handle->write_mutex);
        vSemaphoreDelete(handle->transfer_complete);
        log_text(10, "uart", "initisr %d", channel);
        return W_FAILURE;
    }

    // Start first reception
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, handle->rx_msgs[0].data, UART_MAX_LEN) != HAL_OK) {
        vQueueDelete(handle->msg_queue);
        vSemaphoreDelete(handle->write_mutex);
        vSemaphoreDelete(handle->transfer_complete);
        log_text(10, "uart", "initrx %d", channel);
        return W_IO_ERROR;
    }

    // Mark as initialized
    s_uart_stats[channel].initialized = true;

    return W_SUCCESS;
}
/**
 * @brief Write to the specified UART channel
 * @details // One task can write to a channel at once, and concurrent calls will block for 'timeout
 * @param channel UART channel to write to
 * @param data Buffer to store the data to send
 * @param length uint to store the length of the sending message
 * @param timeout_ms Timeout in milliseconds
 * @return Status of the write operation
 */

w_status_t
uart_write(uart_channel_t channel, uint8_t *buffer, uint16_t length, uint32_t timeout_ms) {
    w_status_t status = W_SUCCESS;
    if ((channel >= UART_CHANNEL_COUNT) || (NULL == s_uart_handles[channel].huart) ||
        (buffer == NULL) || (length == 0)) {
        status = W_INVALID_PARAM; // Invalid parameter(s)
        return status;
    } else if (pdTRUE != xSemaphoreTake(s_uart_handles[channel].write_mutex, timeout_ms)) {
        return W_IO_TIMEOUT; // Could not acquire the mutex in the given time
    }
    HAL_StatusTypeDef transmit_status =
        HAL_UART_Transmit_DMA(s_uart_handles[channel].huart, buffer, length);

    if (HAL_OK != transmit_status) {
        xSemaphoreGive(s_uart_handles[channel].write_mutex); // Release mutex on failure
        if (HAL_ERROR == transmit_status) {
            return W_IO_ERROR;
        } else if (HAL_BUSY == transmit_status) {
            s_uart_stats[channel].timeouts++;
            return W_IO_TIMEOUT;
        }
    }

    // Wait for transfer completion
    if (pdTRUE == xSemaphoreTake(s_uart_handles[channel].transfer_complete, timeout_ms)) {
        // transfer completed successfully, release mutex and return
        if (pdTRUE != xSemaphoreGive(s_uart_handles[channel].write_mutex)) {
            status = W_IO_TIMEOUT;
        } else {
            s_uart_stats[channel].messages_sent++;
        }
        return status;
    } else {
        s_uart_stats[channel].timeouts++;
        status = W_IO_TIMEOUT;
        return status;
    }
    return status;
}

/**
 * @brief Read latest complete message from specified UART channel
 * @details Blocks until message arrives or timeout occurs
 * @param channel UART channel to read from
 * @param buffer Buffer to store the received message
 * @param length Pointer to store the length of the received message
 * @param timeout_ms Timeout in milliseconds
 * @return Status of the read operation
 */
w_status_t
uart_read(uart_channel_t channel, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms) {
    /* Validate all parameters before proceeding */
    if ((channel >= UART_CHANNEL_COUNT) || (NULL == buffer) || (NULL == length)) {
        return W_INVALID_PARAM;
    }

    uart_handle_t *handle = &s_uart_handles[channel];
    uart_msg_t *msg;

    // Wait for message pointer from queue
    if (xQueueReceive(handle->msg_queue, &msg, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_uart_stats[channel].timeouts++;
        *length = 0;
        return W_IO_TIMEOUT;
    }

    // Check for message overflow
    if (msg->len > UART_MAX_LEN) {
        s_uart_stats[channel].overflows++;
        msg->len = UART_MAX_LEN; // Truncate to avoid buffer overflow
    }

    memcpy(buffer, msg->data, msg->len);
    *length = (uint16_t)msg->len;
    msg->busy = false; // Buffer can be reused
    s_uart_stats[channel].messages_received++;
    return W_SUCCESS;
}

/**
 * @brief UART reception complete callback
 * @details Called from ISR when message is received or IDLE line detected
 * @param huart HAL UART handle that triggered the callback
 * @param size Number of bytes received
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    uart_channel_t ch;
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Find channel for this UART
    for (ch = 0; ch < UART_CHANNEL_COUNT; ch++) {
        if (s_uart_handles[ch].huart == huart) {
            uart_handle_t *handle = &s_uart_handles[ch];
            uint8_t curr_buffer = handle->curr_buffer_num;
            uart_msg_t *msg = &handle->rx_msgs[curr_buffer];

            // Store message length
            msg->len = size;
            msg->busy = true;

            /* Advance to next buffer in circular arrangement */
            uint8_t next_buffer = (curr_buffer + 1) % UART_NUM_RX_BUFFERS;
            if (!handle->rx_msgs[next_buffer].busy) {
                // Queue pointer to completed message
                xQueueOverwriteFromISR(handle->msg_queue, &msg, &higher_priority_task_woken);
                handle->curr_buffer_num = next_buffer;
            }

            // Start new reception
            uart_msg_t *next_msg = &handle->rx_msgs[handle->curr_buffer_num];
            next_msg->len = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(huart, next_msg->data, UART_MAX_LEN);

            portYIELD_FROM_ISR(higher_priority_task_woken);
            break;
        }
    }
}

/**
 * @brief UART error callback
 * @details Called from ISR when UART hardware error occurs
 * @param huart HAL UART handle that triggered the error
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uart_channel_t ch;
    BaseType_t higher_priority_task_woken = pdFALSE;

    for (ch = 0; ch < UART_CHANNEL_COUNT; ch++) {
        if (s_uart_handles[ch].huart == huart) {
            s_uart_stats[ch].hw_errors++;

            // Reset current buffer and restart reception
            uart_handle_t *handle = &s_uart_handles[ch];
            uart_msg_t *curr_msg = &handle->rx_msgs[handle->curr_buffer_num];
            curr_msg->len = 0;
            curr_msg->busy = false;
            // Attempt to restart reception
            if (HAL_UARTEx_ReceiveToIdle_DMA(huart, curr_msg->data, UART_MAX_LEN) != HAL_OK) {
                // Critical error, unsure how to recover ISR context
            }
            portYIELD_FROM_ISR(higher_priority_task_woken);
            break;
        }
    }
}

/**
 * @brief ISR triggered when the `huart` channel has completed a transmission
 * @details Gives back transmission complete semaphore when triggered
 * @param huart HAL UART handle that triggered the callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    uart_channel_t ch; // Initialize ch
    BaseType_t higher_priority_task_woken = pdFALSE; // Moved initialization here

    // Find the UART channel associated with the handle
    for (ch = 0; ch < UART_CHANNEL_COUNT; ch++) {
        if (s_uart_handles[ch].huart == huart) {
            // Give the semaphore to signal transfer completion
            xSemaphoreGiveFromISR(
                s_uart_handles[ch].transfer_complete, &higher_priority_task_woken
            );
            portYIELD_FROM_ISR(higher_priority_task_woken);
            break; // Exit loop once channel is found
        }
    }
}

/**
 * @brief Gets and logs the current status of all UART channels
 * @return Status code indicating success or failure
 */
uint32_t uart_get_status(void) {
    uint32_t status_bitfield = 0;

    // Iterate through all UART channels
    for (uart_channel_t channel = 0; channel < UART_CHANNEL_COUNT; channel++) {
        const char *channel_name = "";
        switch (channel) {
            case UART_MOVELLA:
                channel_name = "MOVELLA";
                break;
            case UART_DEBUG_SERIAL:
                channel_name = "DEBUG_SERIAL";
                break;
            default:
                channel_name = "UNKNOWN";
                break;
        }

        uart_stats_t *stats = &s_uart_stats[channel];

        // Log initialization status
        log_text(
            0,
            "uart",
            "%s: %s timeouts %lu hw_err %lu overflows %lu",
            channel_name,
            stats->initialized ? "INIT" : "NOT INIT",
            stats->timeouts,
            stats->hw_errors,
            stats->overflows
        );
    }

    return status_bitfield;
}
