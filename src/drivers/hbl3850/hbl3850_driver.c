#ifdef MOTOR_HBL3850
// -DMOTOR_HBL3850 would need to be added to platformio.ini to use HBL3850 motors.

#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "hbl3850_driver.h"
#include "queue.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"

#include "application/health_checks/health_checks.h"

// Fallback for the sim's HAL mock; the real HAL defines this
#ifndef FDCAN_FILTER_MASK
#define FDCAN_FILTER_MASK ((uint32_t)0x00000002U)
#endif

// Fallback for the test/sim FreeRTOS mock, which only defines the task-level critical section
#ifndef taskENTER_CRITICAL_FROM_ISR
#define taskENTER_CRITICAL_FROM_ISR() ((UBaseType_t)0)
#define taskEXIT_CRITICAL_FROM_ISR(x) ((void)(x))
#endif

#define OUR_NODE_ID 10U // DroneCAN node ID of this board

#define CMD_PRIORITY 8U
#define COMMAND_TYPE_UNITLESS 0U
#define ARRAY_COMMAND_TYPE 1010U
#define ACTUATOR_STATUS_TYPE 1011U
#define NODE_STATUS_TYPE 341U
#define TEMPERATURE_STATUS_TYPE 1110U
#define CIRCUIT_STATUS_TYPE 1091U

#define ACTUATOR_STATUS_SIGNATURE 0x5e9bba44faf1ea04ULL
#define TAIL_START_OF_TRANSFER 0x80U
#define TAIL_END_OF_TRANSFER 0x40U
#define TAIL_TOGGLE 0x20U
#define HBL3850_RAD_TO_DEG (180.0F / 3.14159265F)

/**
 * @brief Reassembly buffer for a multi-frame Actuator.Status transfer. Only the RX callback uses
 * it.
 */
typedef struct {
	uint8_t payload[8]; // the 8 Status payload bytes, filled in across both frames
	uint8_t len; // bytes received so far (0 means no transfer in progress)
	uint16_t crc; // CRC from bytes 0–1 of frame 1
	uint8_t transfer_id; // from the tail byte, must match in frame 2
	bool expect_toggle; // toggle bit expected in the next frame
} hbl3850_rx_transfer_t;

/**
 * @brief Per-motor configuration and state. One entry per slot in s_motors.
 */
typedef struct {
	uint8_t node_id;
	uint8_t actuator_id;
	QueueHandle_t encoder_queue;
	hbl3850_health_t health;
	bool configured;
	hbl3850_rx_transfer_t status_rx;
	volatile bool heard;
} hbl3850_motor_t;

static const uint16_t LOG_WAIT_MS = 0;
static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static uint8_t g_cmd_transfer_id = 0;
// ArrayCommand CAN ID: priority (bits 28-24) | message type (bits 23-8) | our node ID (bits 6-0)
static const uint32_t can_id =
	((uint32_t)CMD_PRIORITY << 24) | ((uint32_t)ARRAY_COMMAND_TYPE << 8) | OUR_NODE_ID;
static hbl3850_motor_t s_motors[HBL3850_MOTOR_COUNT];

w_status_t hbl3850_motor_init(hbl3850_motor_id_t motor, uint8_t node_id, uint8_t actuator_id) {
	if (motor >= HBL3850_MOTOR_COUNT) {
		return W_INVALID_PARAM;
	}
	if (node_id == OUR_NODE_ID) {
		return W_INVALID_PARAM;
	}
	for (hbl3850_motor_id_t m = 0; m < HBL3850_MOTOR_COUNT; ++m) {
		if (s_motors[m].configured && (m != motor && ((s_motors[m].node_id == node_id) ||
		                                              (s_motors[m].actuator_id == actuator_id)))) {
			return W_INVALID_PARAM;
		}
	}
	if (node_id == 0 || node_id > 127) {
		return W_INVALID_PARAM;
	}
	s_motors[motor].node_id = node_id;
	s_motors[motor].actuator_id = actuator_id;
	s_motors[motor].configured = true;
	return W_SUCCESS;
}

/**
 * @brief Convert a float to IEEE 754 half precision bits
 * @param[in] value Value to convert
 * @return float16 bit pattern
 */
static uint16_t float_to_half(float32_t value) {
	(void)value;
	return 0; // TODO: implement IEEE 754 half precision
}

/**
 * @brief Convert IEEE 754 half precision bits to a float
 * @param[in] value float16 bit pattern
 * @return Converted value
 */
static float32_t half_to_float(uint16_t value) {
	(void)value;
	return 0.0F; // TODO: implement IEEE 754 half precision
}

/**
 * @brief Add one byte to a running CRC-16-CCITT (polynomial 0x1021)
 * @param[in] crc  Running CRC value
 * @param[in] byte Byte to add
 * @return Updated CRC
 */
static uint16_t crc16_add_byte(uint16_t crc, uint8_t byte) {
	crc ^= (uint16_t)byte << 8;
	for (int i = 0; i < 8; ++i) {
		if (crc >> 15) {
			crc = (crc << 1) ^ 0x1021;
		} else {
			crc = crc << 1;
		}
	}
	return crc;
}

/**
 * @brief Compute the DroneCAN transfer CRC of a reassembled multi-frame payload
 * @param[in] signature Data type signature of the message
 * @param[in] payload   Reassembled payload bytes
 * @param[in] len       Number of payload bytes
 * @return Transfer CRC
 */
static uint16_t dronecan_transfer_crc(uint64_t signature, const uint8_t *payload, size_t len) {
	// CRC's initial value is 0xFFFF and signature is fed in first.
	uint16_t crc = 0xffff;
	for (int i = 0; i < 8; ++i) {
		crc = crc16_add_byte(crc, (uint8_t)(signature >> (i * 8)));
	}
	for (size_t i = 0; i < len; ++i) {
		crc = crc16_add_byte(crc, (uint8_t)(payload[i]));
	}
	return crc;
}

/**
 * @brief Stop the FDCAN peripheral
 */
static void stop_can(void) {
	HAL_FDCAN_Stop(g_hfdcan);
}

/**
 * @brief Check whether NodeStatus has been received from every configured motor
 * @return true if all configured motors have been heard, false otherwise
 */
static bool all_motors_heard(void) {
	for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
		if (!s_motors[motor].heard && s_motors[motor].configured) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Find a configured motor in s_motors by its DroneCAN node ID
 * @param[in] node_id DroneCAN node ID of the sender
 * @return Index into s_motors, or -1 if no configured motor has that node ID
 */
static int find_motor_by_node_id(uint8_t node_id) {
	for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
		if (s_motors[motor].node_id == node_id && s_motors[motor].configured) {
			return motor;
		}
	}
	return -1;
}

w_status_t hbl3850_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms) {
	if (NULL == hfdcan) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Invalid pointers");
		return W_INVALID_PARAM;
	}

	g_hfdcan = hfdcan;
	bool configured_motors = false;
	for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
		if (s_motors[motor].configured) {
			configured_motors = true;
		}
	}
	if (!configured_motors) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No motors are configured");
		return W_FAILURE;
	}

	// Create a one-slot encoder queue for each configured motor and reset its RX state
	for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
		if (s_motors[motor].configured) {
			if (s_motors[motor].encoder_queue == NULL) {
				s_motors[motor].encoder_queue = xQueueCreate(1, sizeof(hbl3850_encoder_t));
			}
			if (s_motors[motor].encoder_queue == NULL) {
				log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
				         "Lack of memory to create encoder queue");
				return W_FAILURE;
			}
			s_motors[motor].heard = false;
			s_motors[motor].status_rx.len = 0;
		}
	}

	// Accept all extended IDs into RX FIFO1
	FDCAN_FilterTypeDef filter = {0};
	filter.IdType = FDCAN_EXTENDED_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	filter.FilterID1 = 0;
	filter.FilterID2 = 0;

	if (HAL_FDCAN_ConfigFilter(g_hfdcan, &filter) != HAL_OK) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN filter config failed");
		return W_FAILURE;
	}

	if (HAL_FDCAN_Start(g_hfdcan) != HAL_OK) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN start failed");
		stop_can();
		return W_FAILURE;
	}

	if (HAL_FDCAN_ActivateNotification(g_hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN activate notification failed");
		stop_can();
		return W_FAILURE;
	}
	// Wait for NodeStatus from every configured motor
	uint32_t start_ms = 0;
	if (timer_get_ms(&start_ms) != W_SUCCESS) {
		stop_can();
		return W_FAILURE;
	}
	uint32_t now_ms = start_ms;
	while (!(all_motors_heard()) && (now_ms - start_ms <= can_init_timeout_ms)) {
		vTaskDelay(pdMS_TO_TICKS(500));
		if (timer_get_ms(&now_ms) != W_SUCCESS) {
			stop_can();
			return W_FAILURE;
		}
	}
	if (!all_motors_heard()) {
		for (hbl3850_motor_id_t motor = 0; motor < HBL3850_MOTOR_COUNT; ++motor) {
			if (s_motors[motor].configured && !s_motors[motor].heard) {
				log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No NodeStatus from node %u",
				         s_motors[motor].node_id);
			}
		}
		stop_can();
		return W_FAILURE;
	}
	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "hbl3850", "Init successful");
	return W_SUCCESS;
}

/**
 * @brief Transmit 29-bit ID via FDCAN
 *
 * @param[in] ext_id  29-bit extended CAN ID
 * @param[in] data    Pointer to payload data
 * @param[in] len     Payload length
 * @return W_SUCCESS on success, W_FAILURE on error
 */
static w_status_t hbl3850_can_transmit_ext(uint32_t ext_id, const uint8_t *data, uint8_t len) {
	if ((NULL == g_hfdcan) || (NULL == data) || (len > 8)) {
		return W_FAILURE;
	}

	// Reinit the CAN module if a bus off state was detected
	FDCAN_ProtocolStatusTypeDef protocolStatus = {};
	HAL_FDCAN_GetProtocolStatus(g_hfdcan, &protocolStatus);
	if (protocolStatus.BusOff) {
		CLEAR_BIT(g_hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}

	FDCAN_TxHeaderTypeDef tx_header = {0};
	tx_header.Identifier = ext_id;
	tx_header.IdType = FDCAN_EXTENDED_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;
	tx_header.DataLength = len;

	if (HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &tx_header, data) != HAL_OK) {
		return W_FAILURE;
	}
	return W_SUCCESS;
}

w_status_t hbl3850_send_position_cmd(hbl3850_motor_id_t motor, float32_t angle_deg) {
	if (motor >= HBL3850_MOTOR_COUNT || isnan(angle_deg)) {
		return W_INVALID_PARAM;
	}
	if (!s_motors[motor].configured) {
		return W_INVALID_PARAM;
	}
	// ArrayCommand payload: actuator ID, command type, float16 value (low byte first), tail
	uint8_t data[5];
	data[0] = s_motors[motor].actuator_id;
	data[1] = COMMAND_TYPE_UNITLESS;
	// The motor maps the unitless range -1..1 to -60..+60 degrees
	float32_t converted_angle = angle_deg / 60.0F;
	if (converted_angle > 1.0F) {
		converted_angle = 1.0F;
	}
	if (converted_angle < -1.0F) {
		converted_angle = -1.0F;
	}
	uint16_t angle_bits = float_to_half(converted_angle);

	data[2] = (uint8_t)(angle_bits & 0xff);
	data[3] = angle_bits >> 8;
	// Single-frame transfer, so start and end are both set
	data[4] = TAIL_START_OF_TRANSFER | TAIL_END_OF_TRANSFER | g_cmd_transfer_id;
	g_cmd_transfer_id = (g_cmd_transfer_id + 1) & 0x1f; // transfer ID is 5 bits, wraps at 32

	return hbl3850_can_transmit_ext(can_id, data, sizeof(data));
}

w_status_t hbl3850_get_encoder(hbl3850_motor_id_t motor, hbl3850_encoder_t *encoder) {
	if (motor >= HBL3850_MOTOR_COUNT) {
		return W_INVALID_PARAM;
	}
	if (NULL == encoder) {
		return W_FAILURE;
	}

	if (s_motors[motor].encoder_queue == NULL) {
		return W_FAILURE;
	}

	if (xQueuePeek(s_motors[motor].encoder_queue, encoder, 0) == pdPASS) {
		return W_SUCCESS;
	}
	return W_FAILURE; // no feedback received yet
}

/**
 * @brief FDCAN RX FIFO1 callback for HBL3850 DroneCAN messages
 *
 * Called from ISR context for every frame in RX FIFO1. Reassembles multi-frame
 * Actuator.Status transfers into the encoder queue, and decodes NodeStatus,
 * CircuitStatus and Temperature into the motor's health struct.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	// Check if the frame belongs to the right FDCAN peripheral
	if (hfdcan != g_hfdcan) {
		return;
	}

	if (0 == (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {
		return;
	}

	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) {
		return;
	}

	// DroneCAN frames always use 29-bit IDs and end with a tail byte
	if ((rx_header.IdType != FDCAN_EXTENDED_ID) || (rx_header.DataLength == 0)) {
		return;
	}

	// DroneCAN 29-bit ID: type ID = bits 23-8, service = bit 7, source node = bits 6-0
	uint32_t id = rx_header.Identifier;
	uint32_t typeID = (id << 8) >> 16;
	uint32_t service = (id << 24) >> 31;
	uint32_t source = (id << 25) >> 25;
	int motor_index = find_motor_by_node_id((uint8_t)source);

	// Ignore service frames and frames from unknown nodes
	if (service || motor_index == -1) {
		return;
	}

	uint32_t now_ms = 0;

	if (timer_get_ms(&now_ms) != W_SUCCESS) {
		return;
	}

	if (typeID == ACTUATOR_STATUS_TYPE) {
		// Tail byte: start (bit 7), end (bit 6), toggle (bit 5), transfer ID (bits 4-0)
		uint8_t tail = rx_data[rx_header.DataLength - 1];
		bool tail_start = (tail & TAIL_START_OF_TRANSFER) != 0;
		bool tail_end = (tail & TAIL_END_OF_TRANSFER) != 0;
		bool tail_toggle = (tail & TAIL_TOGGLE) != 0;
		uint8_t transfer_id = tail & 0x1f;
		hbl3850_rx_transfer_t *rx = &s_motors[motor_index].status_rx;

		if ((tail_start && !tail_end && !tail_toggle) && rx_header.DataLength == 8) {
			// Frame 1: transfer CRC (bytes 0-1), payload 0-4 (bytes 2-6), tail
			rx->crc = (uint16_t)(rx_data[0] | (rx_data[1] << 8));
			rx->payload[0] = rx_data[2];
			rx->payload[1] = rx_data[3];
			rx->payload[2] = rx_data[4];
			rx->payload[3] = rx_data[5];
			rx->payload[4] = rx_data[6];
			rx->transfer_id = transfer_id;
			rx->expect_toggle = true;
			rx->len = 5;
			return;
		} else if ((!tail_start && tail_end && tail_toggle) && (rx_header.DataLength == 4) &&
		           (rx->len == 5) && (transfer_id == rx->transfer_id)) {
			// Frame 2: payload 5-7 (bytes 0-2), tail
			rx->payload[5] = rx_data[0];
			rx->payload[6] = rx_data[1];
			rx->payload[7] = rx_data[2];
			uint16_t calc = dronecan_transfer_crc(ACTUATOR_STATUS_SIGNATURE, rx->payload, 8);
			bool crc_ok = (calc == rx->crc);
			rx->len = 0;
			if (!crc_ok) {
				return;
			} // corrupted or mismatched, drop it
			// Payload: actuator ID (0), position (1-2), force (3-4), speed (5-6), power (7)
			if (rx->payload[0] != s_motors[motor_index].actuator_id) {
				return;
			}
			float32_t pos = half_to_float(rx->payload[2] << 8 | rx->payload[1]);
			float32_t speed = half_to_float(rx->payload[6] << 8 | rx->payload[5]);
			hbl3850_encoder_t motor_status;
			// TODO: verify on bench that the motor reports radians (DSDL) and not degrees (MKS doc)
			motor_status.position_deg = pos * HBL3850_RAD_TO_DEG;
			motor_status.speed_dps = speed * HBL3850_RAD_TO_DEG;
			motor_status.timestamp_ms = now_ms;
			BaseType_t woken = pdFALSE;
			xQueueOverwriteFromISR(s_motors[motor_index].encoder_queue, &motor_status, &woken);
			portYIELD_FROM_ISR(woken);
		} else {
			// Out-of-sequence or malformed frame: drop any partial transfer
			rx->len = 0;
			return;
		}
	} else if (typeID == NODE_STATUS_TYPE) {
		if (rx_header.DataLength < 8) {
			return;
		}
		uint8_t health = (rx_data[4] >> 6) & 0x03;
		uint8_t mode = (rx_data[4] >> 3) & 0x07;
		UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
		s_motors[motor_index].health.node_health = health;
		s_motors[motor_index].health.node_mode = mode;
		s_motors[motor_index].health.nodestatus_timestamp_ms = now_ms;
		taskEXIT_CRITICAL_FROM_ISR(saved);
		// Set after health is written, so anything waiting on heard sees complete data
		s_motors[motor_index].heard = true;
	} else if (typeID == CIRCUIT_STATUS_TYPE) {
		if (rx_header.DataLength < 8) {
			return;
		}
		float32_t voltage = half_to_float(rx_data[2] | rx_data[3] << 8);
		float32_t current = half_to_float(rx_data[4] | rx_data[5] << 8);
		uint8_t error = rx_data[6];
		UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
		s_motors[motor_index].health.voltage_v = voltage;
		s_motors[motor_index].health.current_a = current;
		s_motors[motor_index].health.circuit_err_flags = error;
		s_motors[motor_index].health.circuit_timestamp_ms = now_ms;
		taskEXIT_CRITICAL_FROM_ISR(saved);
	} else if (typeID == TEMPERATURE_STATUS_TYPE) {
		if (rx_header.DataLength < 6) {
			return;
		}
		// DroneCAN sends temperature in Kelvin
		float32_t temperature = half_to_float(rx_data[2] | rx_data[3] << 8) - 273.15F;
		uint8_t error = rx_data[4];
		UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
		s_motors[motor_index].health.temperature_c = temperature;
		s_motors[motor_index].health.temp_err_flags = error;
		s_motors[motor_index].health.temp_timestamp_ms = now_ms;
		taskEXIT_CRITICAL_FROM_ISR(saved);
	}
}

// TODO: implement hbl3850_get_health(motor, health_stats)
//   - Return W_INVALID_PARAM if motor >= HBL3850_MOTOR_COUNT; W_FAILURE if health_stats is NULL,
//     the motor isn't configured, or it hasn't been heard yet.
//   - The RX callback (an ISR) writes s_motors[motor].health, so copy it inside a critical section:
//         taskENTER_CRITICAL();
//         *health_stats = s_motors[motor].health;
//         taskEXIT_CRITICAL();

// TODO: implement hbl3850_get_status(void)
//   - Build one health_status_t covering all configured motors (pattern: ak45_get_status()).
//   - Skip motors that aren't configured.
//   - Copy each motor's health inside taskENTER_CRITICAL()/taskEXIT_CRITICAL() before checking it.
//   - Checks: offline if now - nodestatus_timestamp_ms > 3000 ms, node_health != 0,
//     circuit_err_flags or temp_err_flags nonzero.
//   - Needs a module ID; rocketlib has no CANARDS_MODULE_ID_HBL3850 yet.

#endif
