#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "drivers/timer/timer.h"

// Include necessary headers for fatal error handler
#include "stm32h7xx_hal.h" // For __disable_irq, __NOP
#include "third_party/canlib/can.h" // For can_msg_t, can_send
#include "third_party/canlib/message/msg_general.h" // For build_debug_raw_msg
#include "third_party/canlib/message_types.h" // For MSG_DEBUG_RAW, PRIO_HIGH, etc.
#include <stdint.h>
#include <string.h>

// TODO: calculate better. for now make excessively large and check dropped tx counter
#define BUS_QUEUE_LENGTH 32

static QueueHandle_t bus_queue_rx = NULL;
static QueueHandle_t bus_queue_tx = NULL;
static uint32_t dropped_rx_counter = 0;
static can_handler_status_t can_error_stats = {0};

static can_callback_t callback_map[MSG_ID_ENUM_MAX] = {NULL};

static w_status_t can_reset_callback(const can_msg_t *msg) {
    if (check_board_need_reset(msg)) {
        NVIC_SystemReset();
        return W_FAILURE; // Should never reach here
    }
    return W_SUCCESS;
}

static w_status_t can_led_on_callback(const can_msg_t *msg) {
    (void)msg;
    w_status_t status = W_SUCCESS;
    status |= gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_LOW, 5);
    status |= gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_LOW, 5);
    status |= gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_LOW, 5);

    if (status != W_SUCCESS) {
        log_text(1, "CANCallback", "ERROR: LED ON callback failed to set GPIO.");
    }
    return status;
}

static w_status_t can_led_off_callback(const can_msg_t *msg) {
    (void)msg;
    w_status_t status = W_SUCCESS;
    status |= gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 5);
    status |= gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_HIGH, 5);
    status |= gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 5);

    if (status != W_SUCCESS) {
        log_text(1, "CANCallback", "ERROR: LED OFF callback failed to set GPIO.");
    }
    return status;
}

static void can_handle_rx_isr(const can_msg_t *message, uint32_t timestamp) {
    (void)timestamp;
    // software filter: only queue messages with registered callbacks
    can_msg_type_t msg_type = get_message_type(message);
    // drop any message types without a registered handler
    if (callback_map[msg_type] == NULL) {
        return; // drop unregistered IDs immediately
    }
    // enqueue message for RX task; track if higher-priority task should run
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (pdPASS != xQueueSendFromISR(bus_queue_rx, message, &higher_priority_task_woken)) {
        can_error_stats.rx_callback_errors++; // cant log err in isr
    }
    // request context switch if needed
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

w_status_t can_handler_init(FDCAN_HandleTypeDef *hfdcan) {
    if (NULL == hfdcan) {
        return W_INVALID_PARAM;
    }

    bus_queue_rx = xQueueCreate(BUS_QUEUE_LENGTH, sizeof(can_msg_t));
    bus_queue_tx = xQueueCreate(BUS_QUEUE_LENGTH, sizeof(can_msg_t));

    if ((NULL == bus_queue_tx) || (NULL == bus_queue_rx)) {
        log_text(1, "can", "initfailq");
        return W_FAILURE;
    }

    if (!can_init_stm(hfdcan, can_handle_rx_isr)) {
        log_text(1, "CANHandler", "ERROR: can_init_stm failed.");
        return W_FAILURE;
    }

    if ((W_SUCCESS != can_handler_register_callback(MSG_RESET_CMD, can_reset_callback)) ||
        (W_SUCCESS != can_handler_register_callback(MSG_LEDS_ON, can_led_on_callback)) ||
        (W_SUCCESS != can_handler_register_callback(MSG_LEDS_OFF, can_led_off_callback))) {
        log_text(1, "CANHandler", "ERROR: Failed to register mandatory CAN callbacks.");
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t can_handler_register_callback(can_msg_type_t msg_type, can_callback_t callback) {
    callback_map[msg_type] = callback;
    return W_SUCCESS;
}

w_status_t can_handler_transmit(const can_msg_t *message) {
    if (pdPASS != xQueueSend(bus_queue_tx, message, 0)) {
        log_text(1, "CANHandler", "ERROR: Failed to queue message for TX. Queue full?");
        can_error_stats.dropped_tx_counter++; // Track dropped TX messages
        return W_FAILURE;
    }
    return W_SUCCESS;
}

void can_handler_task_rx(void *argument) {
    (void)argument;
    // throttle RX timeout warnings to once per second
    static TickType_t last_rx_warn_tick = 0;
    for (;;) {
        can_msg_t rx_msg;
        if (pdPASS == xQueueReceive(bus_queue_rx, &rx_msg, 100)) {
            // dispatch to registered callback
            can_msg_type_t msg_type = get_message_type(&rx_msg);
            if (callback_map[msg_type] != NULL) {
                if (callback_map[msg_type](&rx_msg) != W_SUCCESS) {
                    log_text(1, "CANHandlerRX", "WARN: Callback failed for msg type %d.", msg_type);
                    can_error_stats.rx_callback_errors++; // Track callback execution errors
                }
            }
        } else {
            // timed out waiting; log once per second
            TickType_t now = xTaskGetTickCount();
            if ((now - last_rx_warn_tick) >= pdMS_TO_TICKS(1000)) {
                log_text(1, "CANHandlerRX", "WARN: Timed out waiting for RX message.");
                can_error_stats.rx_timeouts++; // Track RX timeouts
                last_rx_warn_tick = now; // update last warning time
            }
        }
    }
}

void can_handler_task_tx(void *argument) {
    (void)argument;
    // throttle TX timeout warnings to once per second
    static TickType_t last_tx_warn_tick = 0;
    for (;;) {
        can_msg_t tx_msg;

        if (xQueueReceive(bus_queue_tx, &tx_msg, pdMS_TO_TICKS(5)) == pdPASS) {
            // send to CAN bus; log errors
            if (!can_send(&tx_msg)) {
                can_error_stats.tx_failures++;
                log_text(3, "CAN tx", "CAN send failed!");
            }
            // hardware limitation stm32 backtoback tx fifo queue has 2 msgs..
            // but trying to do 2 in a row didnt work so just delay between every tx
            // also 1ms delay didnt work so 2ms???
            vTaskDelay(2);
        } else {
            // expect we send at least 1 message every 1.5sec
            TickType_t now = xTaskGetTickCount();
            if ((now - last_tx_warn_tick) >= pdMS_TO_TICKS(1500)) {
                log_text(1, "CANHandlerTX", "no tx msg in queue");
                last_tx_warn_tick = now;
            }
        }
    }
}

// --- Fatal Error Handler Implementation ---

// Note: All IDs (Board Type, Message Type, Instance ID)
//       are used directly from canlib/message_types.h

void proc_handle_fatal_error(const char *errorMsg) {
    // safe state - loop here forever and send CAN err msg repeatedly
    while (1) {
        __disable_irq();

        // let CAN still work
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

        can_msg_t msg;
        uint8_t data[6] = {0}; // Data for the debug message (max 6 bytes)

        // Copy error message to the data buffer
        if (errorMsg != NULL) {
            strncpy((char *)data, errorMsg, sizeof(data));
            // Ensure null termination
            data[sizeof(data) - 1] = '\0';
        }

        // Use canlib's helper function to build the debug message
        // Set priority to high and timestamp to 0 (since we can't reliably get timestamp in error
        // state)
        if (build_debug_raw_msg(PRIO_HIGH, 0, data, &msg)) {
            // Only try to send if message build succeeded
            can_send(&msg);
        }

        // scream a few times then attempt to reset.
        // delay for ~1sec without using systick-based delays (no hal_delay)
        volatile int dummy;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 50000000; j++) {
                dummy++;
            }
        }

        dummy++;

        // resetting is always a safe state even in midflight, as flightphase starts IDLE
        NVIC_SystemReset();
    }
}

// --- End Fatal Error Handler ---

uint32_t can_handler_get_status(void) {
    uint32_t status_bitfield = 0;

    // Log all error statistics
    log_text(
        0,
        "CAN",
        "dropped_rx=%lu, dropped_tx=%lu, tx_failures=%lu, ",
        dropped_rx_counter,
        can_error_stats.dropped_tx_counter,
        can_error_stats.tx_failures
    );
    log_text(
        0,
        "CAN",
        "rx_callback_errors=%lu, rx_timeouts=%lu, tx_timeouts=%lu",
        can_error_stats.rx_callback_errors,
        can_error_stats.rx_timeouts,
        can_error_stats.tx_timeouts
    );

    return status_bitfield;
}
