#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "third_party/canlib/message/msg_general.h" /* For build_debug_raw_msg */
#include "third_party/canlib/message_types.h" /* For MSG_DEBUG_RAW, PRIO_HIGH, etc. */

#include "application/can_handler/can_handler.h"
#include "application/can_handler/can_telemetry_scaling.h"
#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/gpio/gpio.h"
#include "drivers/timer/timer.h"

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
} can_handler_status_t;

// TODO: calculate better. for now make excessively large and check dropped tx counter
#define BUS_QUEUE_LENGTH 32

static QueueHandle_t bus_queue_rx = NULL;
static QueueHandle_t bus_queue_tx = NULL;
static uint32_t dropped_rx_counter = 0;
static can_handler_status_t can_error_stats = {0};

static can_callback_t callback_map[MSG_ID_ENUM_MAX] = {NULL};

static w_status_t can_reset_callback(const can_msg_t *msg) {
	bool need_reset = false;

	if (check_board_need_reset(msg, &need_reset) != W_SUCCESS) {
		log_text(1, LOG_LVL_WARN, "CANCallback", "Failed to read board reset");
		return W_FAILURE;
	}

	if (need_reset) {
		log_text(1, LOG_LVL_INFO, "can_handler", "System reset initiated");
		NVIC_SystemReset();
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
		log_text(1, LOG_LVL_WARN, "CANCallback", "LED ON callback failed to set GPIO.");
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
		log_text(1, LOG_LVL_WARN, "CANCallback", "LED OFF callback failed to set GPIO.");
	}
	return status;
}

void can_handler_rx_message(const can_msg_t *message) {
	// software filter: only queue messages with registered callbacks
	can_msg_type_t msg_type = get_message_type(message);
	// drop any message types without a registered handler
	if (NULL == callback_map[msg_type]) {
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

static void can_get_signed_limits(can_types_t type, int32_t *min_out, int32_t *max_out) {
	switch (type) {
		case TYPE_INT16:
			*min_out = INT16_MIN;
			*max_out = INT16_MAX;
			break;
		case TYPE_INT24:
			*min_out = INT24_MIN;
			*max_out = INT24_MAX;
			break;
		case TYPE_INT32:
			*min_out = INT32_MIN;
			*max_out = INT32_MAX;
			break;
		default:
			*min_out = 0;
			*max_out = 0;
			break;
	}
}

static void can_get_unsigned_max(can_types_t type, uint32_t *max_out) {
	switch (type) {
		case TYPE_UINT16:
			*max_out = UINT16_MAX;
			break;
		case TYPE_UINT32:
			*max_out = UINT32_MAX;
			break;
		default:
			*max_out = 0;
			break;
	}
}

static bool can_type_is_unsigned(can_types_t type) {
	return ((type == TYPE_UINT16) || (type == TYPE_UINT32));
}

static w_status_t can_store_unsigned(can_types_t type, uint32_t value, void *out) {
	if (NULL == out) {
		return W_INVALID_PARAM;
	}

	switch (type) {
		case TYPE_UINT16: {
			uint16_t encoded = (uint16_t)value;
			memcpy(out, &encoded, sizeof(encoded));
			return W_SUCCESS;
		}
		case TYPE_UINT32: {
			uint32_t encoded = (uint32_t)value;
			memcpy(out, &encoded, sizeof(encoded));
			return W_SUCCESS;
		}
		default:
			return W_INVALID_PARAM;
	}
}

static w_status_t can_store_signed(can_types_t type, int32_t value, void *out) {
	if (NULL == out) {
		return W_INVALID_PARAM;
	}

	switch (type) {
		case TYPE_INT16: {
			int16_t encoded = (int16_t)value;
			memcpy(out, &encoded, sizeof(encoded));
			return W_SUCCESS;
		}
		case TYPE_INT24: // using int32
		case TYPE_INT32: {
			int32_t encoded = (int32_t)value;
			memcpy(out, &encoded, sizeof(encoded));
			return W_SUCCESS;
		}
		default:
			return W_INVALID_PARAM;
	}
}

// Store the sentinel code for a non-finite float into the target type.
// offset is one of SENTINEL_POS_INF / SENTINEL_NEG_INF and selects
// a code at the top of the type's range (type_max - offset).
static w_status_t can_store_sentinel(can_types_t type, bool is_unsigned, uint32_t offset,
									 void *out) {
	if (is_unsigned) {
		uint32_t maxv = 0U;
		can_get_unsigned_max(type, &maxv);
		return can_store_unsigned(type, (maxv - offset), out);
	} else {
		int32_t minv = 0;
		int32_t maxv = 0;
		can_get_signed_limits(type, &minv, &maxv);
		return can_store_signed(type, (maxv - (int32_t)offset), out);
	}
}

w_status_t can_handler_init(FDCAN_HandleTypeDef *hfdcan) {
	if (NULL == hfdcan) {
		return W_INVALID_PARAM;
	}

	bus_queue_rx = xQueueCreate(BUS_QUEUE_LENGTH, sizeof(can_msg_t));
	bus_queue_tx = xQueueCreate(BUS_QUEUE_LENGTH, sizeof(can_msg_t));

	if ((NULL == bus_queue_tx) || (NULL == bus_queue_rx)) {
		log_text(1, LOG_LVL_FATAL, "can", "initfailq");
		return W_FAILURE;
	}

	if (!stm32h7_can_init(hfdcan, &can_handler_rx_message)) {
		log_text(1, LOG_LVL_FATAL, "CANHandler", "can_init_stm failed.");
		return W_FAILURE;
	}

	if ((W_SUCCESS != can_handler_register_callback(MSG_RESET_CMD, can_reset_callback)) ||
		(W_SUCCESS != can_handler_register_callback(MSG_LEDS_ON, can_led_on_callback)) ||
		(W_SUCCESS != can_handler_register_callback(MSG_LEDS_OFF, can_led_off_callback))) {
		log_text(1, LOG_LVL_FATAL, "CANHandler", "Failed to register mandatory CAN callbacks.");
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
		log_text(1, LOG_LVL_WARN, "CANHandler", "Failed to queue message for TX. Queue full?");
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
					log_text(1,
							 LOG_LVL_WARN,
							 "CANHandlerRX",
							 "Callback failed for msg type %d.",
							 msg_type);
					can_error_stats.rx_callback_errors++; // Track callback execution errors
				}
			}
		} else {
			// timed out waiting; log once per second
			TickType_t now = xTaskGetTickCount();
			if ((now - last_rx_warn_tick) >= pdMS_TO_TICKS(1000)) {
				log_text(1, LOG_LVL_WARN, "CANHandlerRX", "Timed out waiting for RX message.");
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
			if (!stm32h7_can_send(&tx_msg)) {
				can_error_stats.tx_failures++;
				log_text(3, LOG_LVL_WARN, "CAN tx", "CAN send failed!");
			}
			// hardware limitation stm32 backtoback tx fifo queue has 2 msgs..
			// but trying to do 2 in a row didnt work so just delay between every tx
			// also 1ms delay didnt work so 2ms???
			vTaskDelay(pdMS_TO_TICKS(2));
		} else {
			// expect we send at least 1 message every 1.5sec
			TickType_t now = xTaskGetTickCount();
			if ((now - last_tx_warn_tick) >= pdMS_TO_TICKS(1500)) {
				log_text(1, LOG_LVL_WARN, "CANHandlerTX", "no tx msg in queue");
				last_tx_warn_tick = now;
			}
		}
	}
}

w_status_t can_encode_scaled_float(can_scaling_types_t sensor, float32_t input, void *out) {
	if ((sensor >= SCALE_COUNT) || (NULL == out)) {
		return W_INVALID_PARAM;
	}

	can_types_t target_type = can_scale_factor[sensor].type;
	bool is_unsigned = can_type_is_unsigned(target_type);

	if (isnan(input)) {
		uint32_t offset = SENTINEL_NAN;
		w_status_t store_status = can_store_sentinel(target_type, is_unsigned, offset, out);
		return store_status;
	}

	// handle +/-Inf with sentinel codes (not reserved) at the top of the target type
	if (!isfinite(input)) {
		uint32_t offset = signbit(input) ? SENTINEL_NEG_INF : SENTINEL_POS_INF;

		w_status_t store_status = can_store_sentinel(target_type, is_unsigned, offset, out);
		return store_status;
	}

	float32_t scaled = input * (float32_t)can_scale_factor[sensor].scale;

	if (is_unsigned) {
		uint32_t maxv = 0U;
		can_get_unsigned_max(target_type, &maxv);

		if ((scaled < 0.0f) || (scaled > (float32_t)maxv)) {
			return W_OVERFLOW; // TODO: better handling of scaled overflow
		}

		return can_store_unsigned(target_type, (uint32_t)scaled, out);

	} else {
		int32_t minv = 0;
		int32_t maxv = 0;
		can_get_signed_limits(target_type, &minv, &maxv);

		if ((scaled < (float32_t)minv) || (scaled > (float32_t)maxv)) {
			return W_OVERFLOW;
		}

		return can_store_signed(target_type, (int32_t)scaled, out);
	}

	return W_SUCCESS;
}

w_status_t can_encode_scaled_int(can_scaling_types_t sensor, int64_t input, void *out) {
	if ((sensor >= SCALE_COUNT) || (NULL == out)) {
		return W_INVALID_PARAM;
	}

	can_types_t target_type = can_scale_factor[sensor].type;
	bool is_unsigned = can_type_is_unsigned(target_type);

	int64_t scaled = input * can_scale_factor[sensor].scale;

	// Scale and bounds check according to target type
	if (is_unsigned) {
		uint32_t maxv = 0U;
		can_get_unsigned_max(target_type, &maxv);

		if ((scaled < 0) || (scaled > (int64_t)maxv)) {
			return W_OVERFLOW; // TODO: better handling of scaled overflow
		}

		return can_store_unsigned(target_type, (uint32_t)scaled, out);

	} else {
		int32_t minv = 0;
		int32_t maxv = 0;
		can_get_signed_limits(target_type, &minv, &maxv);

		if ((scaled < (int64_t)minv) || (scaled > (int64_t)maxv)) {
			return W_OVERFLOW; // TODO: better handling of scaled overflow
		}

		return can_store_signed(target_type, (int32_t)scaled, out);
	}
	return W_SUCCESS;
}

health_status_t can_handler_get_status(void) {
	uint32_t status_bitfield = 0; // TODO: add meanful errors for health checks

	// Log all error statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "CAN",
			 "dropped_rx=%lu, dropped_tx=%lu, tx_failures=%lu, ",
			 dropped_rx_counter,
			 can_error_stats.dropped_tx_counter,
			 can_error_stats.tx_failures);
	log_text(0,
			 LOG_LVL_INFO,
			 "CAN",
			 "rx_callback_errors=%lu, rx_timeouts=%lu, tx_timeouts=%lu",
			 can_error_stats.rx_callback_errors,
			 can_error_stats.rx_timeouts,
			 can_error_stats.tx_timeouts);

	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_CAN_HANDLER, .error_bitfield = status_bitfield};

	return status;
}
