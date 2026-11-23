#ifdef MOTOR_HBL3850
// -DMOTOR_HBL3850 would need to be added to platformio.ini to use HBL3850 motors.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// FreeRTOS.h must come before task.h
#include "FreeRTOS.h"
#include "task.h"

#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "hbl3850_driver.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"

#include "application/health_checks/health_checks.h"

// Fallback for the sim's HAL mock; the real HAL defines this
#ifndef FDCAN_FILTER_MASK
#define FDCAN_FILTER_MASK ((uint32_t)0x00000002U)
#endif

#define OUR_NODE_ID 10U // DroneCAN node ID of this board

#define CMD_PRIORITY 8U
#define COMMAND_TYPE_UNITLESS 0U
#define ARRAY_COMMAND_TYPE 1010U
#define ARRAY_COMMAND_LEN 5U // actuator ID, command type, float16 value, tail
#define ACTUATOR_STATUS_TYPE 1011U
#define NODE_STATUS_TYPE 341U
#define TEMPERATURE_STATUS_TYPE 1110U
#define CIRCUIT_STATUS_TYPE 1091U

#define ACTUATOR_STATUS_SIGNATURE 0x5e9bba44faf1ea04ULL
#define TAIL_START_OF_TRANSFER 0x80U
#define TAIL_END_OF_TRANSFER 0x40U
#define TAIL_TOGGLE 0x20U
#define TAIL_TRANSFER_ID_MASK 0x1fU // transfer ID is 5 bits, wraps at 32

#define CAN_CLASSIC_MAX_LEN 8U // max payload bytes in a classic CAN frame
#define DRONECAN_MAX_NODE_ID 127U

// Frame lengths in bytes, including the tail byte
#define NODE_STATUS_FRAME_LEN 8U
#define CIRCUIT_STATUS_FRAME_LEN 8U
#define TEMPERATURE_FRAME_LEN 6U
#define ACTUATOR_STATUS_FRAME1_LEN 8U // CRC (2) + payload 0-4 (5) + tail
#define ACTUATOR_STATUS_FRAME2_LEN 4U // payload 5-7 (3) + tail
#define ACTUATOR_STATUS_FRAME1_PAYLOAD 5U // payload bytes carried in frame 1
#define ACTUATOR_STATUS_PAYLOAD_LEN 8U // full reassembled payload

// NodeStatus byte 4: health (bits 7-6), mode (bits 5-3)
#define NODE_STATUS_HEALTH_SHIFT 6U
#define NODE_STATUS_HEALTH_MASK 0x03U
#define NODE_STATUS_MODE_SHIFT 3U
#define NODE_STATUS_MODE_MASK 0x07U

#define KELVIN_TO_CELSIUS_OFFSET 273.15F
#define HBL3850_MAX_ANGLE_DEG 60.0F // unitless +/-1 maps to +/-60 degrees
#define INIT_POLL_PERIOD_MS 500U
#define HBL3850_RAD_TO_DEG (180.0F / 3.14159265F)

#define FP32_MANT_BITS 23U
#define FP16_MANT_BITS 10U
#define BIAS_DELTA (127U - 15U)
#define MANT_DELTA (FP32_MANT_BITS - FP16_MANT_BITS)

#define FP16_SIGN_MASK 0x8000U
#define FP16_EXPONENT_MASK 0x7c00U
#define FP16_MANT_MASK 0x03ffU

#define FP32_SIGN_MASK 0x80000000U
#define FP32_EXPONENT_MASK 0x7f800000U
#define FP32_MANT_MASK 0x007fffffU

// Staleness thresholds for get_status. NodeStatus uses DroneCAN's standard OFFLINE_TIMEOUT_MS
// (3x its 1 s max publish period). CircuitStatus (10 Hz) and Temperature (1 Hz) reuse it, based on
// MKS default publish rates. TODO: confirm the rates on the bench
#define HBL3850_NODE_TIMEOUT_MS 3000U
#define HBL3850_CIRCUIT_TIMEOUT_MS 3000U
#define HBL3850_TEMP_TIMEOUT_MS 3000U

/**
 * @brief Reassembly buffer for a multi-frame Actuator.Status transfer. Only the RX callback uses
 * it.
 */
typedef struct {
	uint8_t payload[ACTUATOR_STATUS_PAYLOAD_LEN]; // the 8 Status payload bytes, filled in across
	                                              // both frames
	uint8_t len; // bytes received so far (0 means no transfer in progress)
	uint16_t crc; // CRC from bytes 0–1 of frame 1
	uint8_t transfer_id; // from the tail byte, must match in frame 2
	bool expect_toggle; // toggle bit expected in the next frame
} hbl3850_rx_transfer_t;

/**
 * @brief Raw motor health data from NodeStatus, CircuitStatus and Temperature. float16 values are
 * stored as raw bits (to limit operations in an ISR) and converted in hbl3850_get_health
 */
typedef struct {
	uint8_t node_health;
	uint8_t node_mode;
	uint32_t nodestatus_timestamp_ms;
	uint16_t voltage_bits; // float16, volts
	uint16_t current_bits; // float16, amps
	uint8_t circuit_err_flags;
	uint32_t circuit_timestamp_ms;
	uint16_t temperature_bits; // float16, Kelvin
	uint8_t temp_err_flags;
	uint32_t temp_timestamp_ms;
} hbl3850_health_raw_t;

/**
 * @brief Raw motor encoder data from Actuator.Status (to limit operations in an ISR)
 */
typedef struct {
	uint16_t pos_bits;
	uint16_t speed_bits;
	uint32_t timestamp_ms;
} hbl3850_encoder_raw_t;

/**
 * @brief Per-motor configuration and state. One entry per slot in s_motors.
 */
typedef struct {
	uint8_t node_id;
	uint8_t actuator_id;
	hbl3850_encoder_raw_t encoder_raw;
	bool encoder_received;
	hbl3850_health_raw_t health;
	bool configured;
	hbl3850_rx_transfer_t status_rx;
	volatile bool heard; // NodeStatus received at least once
	bool circuit_received; // CircuitStatus received at least once
	bool temperature_received; // Temperature received at least once
} hbl3850_motor_t;

/**
 * @brief Copy of one motor's state that get_status reads, taken in a single critical section
 */
typedef struct {
	bool configured;
	uint8_t node_id;
	hbl3850_health_raw_t health;
	bool node_received;
	bool circuit_received;
	bool temperature_received;
	uint32_t now_ms; // time of the snapshot, read in the same critical section
} hbl3850_motor_snapshot_t;

/**
 * @brief Status variables describing the health of the HBL3850 driver module
 */
typedef struct {
	bool is_init;
	uint32_t invalid_args; // NULL or out-of-range arguments
	uint32_t reinit_attempts; // driver_init called after a successful init
	uint32_t init_fdcan_filter_cfg_fails;
	uint32_t init_fdcan_start_fails;
	uint32_t init_fdcan_notification_fails;
	uint32_t init_fdcan_timeout; // a configured motor wasn't heard during init
	uint32_t timer_get_ms_fails;
	uint32_t tx_errors; // HAL_FDCAN_AddMessageToTxFifoQ failures
	uint32_t rx_errors; // HAL_FDCAN_GetRxMessage failures or malformed frames
	uint32_t status_crc_fails; // Actuator.Status transfer CRC mismatches
	uint32_t status_seq_drops; // out-of-sequence Status frames
	uint32_t fdcan_stop_fails;
} hbl3850_driver_health_t;

static const uint16_t LOG_WAIT_MS = 0;
static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static uint8_t g_cmd_transfer_id = 0;
// ArrayCommand CAN ID: priority (bits 28-24) | message type (bits 23-8) | our node ID (bits 6-0)
static const uint32_t can_id =
	((uint32_t)CMD_PRIORITY << 24) | ((uint32_t)ARRAY_COMMAND_TYPE << 8) | OUR_NODE_ID;
static hbl3850_motor_t s_motors[HBL3850_MOTOR_COUNT];
static hbl3850_driver_health_t s_driver_health = {0};

static uint32_t last_tx_errors = 0;
static uint32_t last_rx_errors = 0;
static uint32_t last_crc_fails = 0;

w_status_t hbl3850_motor_init(hbl3850_motor_id_t motor, uint8_t node_id, uint8_t actuator_id) {
	if (s_driver_health.is_init) {
		return W_FAILURE;
	}
	if (motor >= HBL3850_MOTOR_COUNT) {
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}
	if (node_id == OUR_NODE_ID) {
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}
	for (hbl3850_motor_id_t m_id = 0; m_id < HBL3850_MOTOR_COUNT; ++m_id) {
		if (s_motors[m_id].configured &&
		    ((m_id != motor) && ((s_motors[m_id].node_id == node_id) ||
		                         (s_motors[m_id].actuator_id == actuator_id)))) {
			++s_driver_health.invalid_args;
			return W_INVALID_PARAM;
		}
	}
	if ((node_id == 0) || (node_id > DRONECAN_MAX_NODE_ID)) {
		++s_driver_health.invalid_args;
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
	uint32_t fp32 = 0U;
	memcpy(&fp32, &value, sizeof(fp32));

	uint16_t sign = (uint16_t)((fp32 & FP32_SIGN_MASK) >> 16);
	uint32_t raw_exponent = (fp32 & FP32_EXPONENT_MASK) >> FP32_MANT_BITS;
	uint32_t mant = fp32 & FP32_MANT_MASK;

	// FP32 is Infinity or NaN (all 8 exponent bits are 1)
	if (raw_exponent == 0xffU) {
		if (mant != 0U) {
			// Preserve NaN (set quiet NaN bit 0x0200U + top mantissa bits)
			return (uint16_t)(sign | (0x1fU << FP16_MANT_BITS) | 0x0200U | (mant >> MANT_DELTA));
		}
		// Infinity
		return (uint16_t)(sign | (0x1fU << FP16_MANT_BITS));
	}

	// Re-bias exponent from FP32 (bias 127) to FP16 (bias 15) using signed int to detect underflow
	int32_t exponent = (int32_t)raw_exponent - (int32_t)BIAS_DELTA;

	// Overflow (larger than max normal FP16 exponent 30) -> clamp to Infinity
	if (exponent > 30) {
		return (uint16_t)(sign | (0x1fU << FP16_MANT_BITS));
	}

	// Underflow to signed zero or subnormal FP16
	if (exponent <= 0) {
		if (exponent < -10) {
			// Too small even for a 10-bit FP16 subnormal -> signed zero (+/- 0.0)
			return sign;
		}

		// Restore implicit leading 1 bit of FP32 mantissa and shift right into subnormal position
		mant |= 0x00800000U;
		uint32_t shift = (uint32_t)(14 - exponent);
		uint16_t sub_mant = (uint16_t)(mant >> shift);

		// Round to nearest using the highest discarded bit
		if (((mant >> (shift - 1U)) & 1U) != 0U) {
			sub_mant++;
		}
		return (uint16_t)(sign | sub_mant);
	}

	// Normal number (1 <= exponent <= 30)
	uint16_t fp16_exp = (uint16_t)((uint32_t)exponent << FP16_MANT_BITS);
	uint16_t fp16_mant = (uint16_t)(mant >> MANT_DELTA);

	// Round to nearest using bit 12 (the highest discarded mantissa bit)
	if ((mant & 0x00001000U) != 0U) {
		fp16_mant++;
	}

	// Add (not OR) so a rounding carry out of the mantissa bumps the exponent
	return (uint16_t)(sign | (uint16_t)(fp16_exp + fp16_mant));
}

/**
 * @brief Convert IEEE 754 half precision bits to a float
 * @param[in] value float16 bit pattern
 * @return Converted value
 */
static float32_t half_to_float(uint16_t value) {
	uint32_t sign = ((uint32_t)(value & FP16_SIGN_MASK)) << 16;
	uint32_t exponent = ((uint32_t)(value & FP16_EXPONENT_MASK)) >> FP16_MANT_BITS;
	uint32_t mant = (uint32_t)(value & FP16_MANT_MASK);

	uint32_t fp32 = 0U;

	if (exponent == 0U) {
		if (mant == 0U) {
			// Signed zero (+/- 0.0)
			fp32 = sign;
		} else {
			// Subnormal FP16: shift mantissa left until implicit leading 1 reaches bit 10
			exponent = BIAS_DELTA + 1U;
			while ((mant & 0x0400U) == 0U) {
				mant <<= 1;
				exponent--;
			}
			mant &= FP16_MANT_MASK;
			fp32 = sign | (exponent << FP32_MANT_BITS) | (mant << MANT_DELTA);
		}
	} else if (exponent == 31U) {
		// Infinity (mant == 0) or NaN (mant != 0)
		fp32 = sign | (0xffU << FP32_MANT_BITS) | (mant << MANT_DELTA);
	} else {
		// Normal number: re-bias exponent to FP32 and pad mantissa with 13 trailing zeros
		fp32 = sign | ((exponent + BIAS_DELTA) << FP32_MANT_BITS) | (mant << MANT_DELTA);
	}

	// Reinterpret raw IEEE 754 bits as float32_t without numeric conversion
	float32_t result = 0.0F;
	memcpy(&result, &fp32, sizeof(result));
	return result;
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
		if ((crc & 0x8000U) != 0U) {
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
	if (HAL_FDCAN_Stop(g_hfdcan) != HAL_OK) {
		++s_driver_health.fdcan_stop_fails;
	}
}

/**
 * @brief Handle one Actuator.Status frame. Reassembles the 2-frame transfer, checks its CRC and
 * stores the raw position and speed bits for the motor. Called from ISR context.
 * @param[in] motor_index Index into s_motors of the sending motor
 * @param[in] rx_data     Frame payload
 * @param[in] len         Frame payload length in bytes
 * @param[in] now_ms      Time the frame was received
 */
static void handle_actuator_status(int motor_index, const uint8_t *rx_data, uint32_t len,
                                   uint32_t now_ms) {
	// Tail byte: start (bit 7), end (bit 6), toggle (bit 5), transfer ID (bits 4-0)
	uint8_t tail = rx_data[len - 1];
	bool tail_start = ((tail & TAIL_START_OF_TRANSFER) != 0);
	bool tail_end = ((tail & TAIL_END_OF_TRANSFER) != 0);
	bool tail_toggle = ((tail & TAIL_TOGGLE) != 0);
	uint8_t transfer_id = tail & TAIL_TRANSFER_ID_MASK;
	hbl3850_rx_transfer_t *rx = &s_motors[motor_index].status_rx;

	if ((tail_start && !tail_end && !tail_toggle) && (len == ACTUATOR_STATUS_FRAME1_LEN)) {
		// Frame 1: transfer CRC (bytes 0-1), payload 0-4 (bytes 2-6), tail
		rx->crc = (uint16_t)(rx_data[0] | (rx_data[1] << 8));
		rx->payload[0] = rx_data[2];
		rx->payload[1] = rx_data[3];
		rx->payload[2] = rx_data[4];
		rx->payload[3] = rx_data[5];
		rx->payload[4] = rx_data[6];
		rx->transfer_id = transfer_id;
		rx->expect_toggle = true;
		rx->len = ACTUATOR_STATUS_FRAME1_PAYLOAD;
	} else if ((!tail_start && tail_end && tail_toggle) && (len == ACTUATOR_STATUS_FRAME2_LEN) &&
	           (rx->len == ACTUATOR_STATUS_FRAME1_PAYLOAD) && (transfer_id == rx->transfer_id)) {
		// TODO: record frame 1's receive time and reject frame 2 if it arrives after the DroneCAN
		// transfer timeout (2 s), so a stale frame 1 can't pair with a much later frame 2
		// Frame 2: payload 5-7 (bytes 0-2), tail
		rx->payload[5] = rx_data[0];
		rx->payload[6] = rx_data[1];
		rx->payload[7] = rx_data[2];
		uint16_t calc = dronecan_transfer_crc(ACTUATOR_STATUS_SIGNATURE, rx->payload,
		                                      ACTUATOR_STATUS_PAYLOAD_LEN);
		bool crc_ok = (calc == rx->crc);
		rx->len = 0;
		// Payload: actuator ID (0), position (1-2), force (3-4), speed (5-6), power (7)
		if (!crc_ok) {
			// Corrupted or mismatched frames, drop the reading
			++s_driver_health.status_crc_fails;
		} else if (rx->payload[0] != s_motors[motor_index].actuator_id) {
			// Reading is for a different actuator, drop it
			++s_driver_health.rx_errors;
		} else {
			// Store raw float16 bits; get_encoder converts them in task context
			s_motors[motor_index].encoder_raw.pos_bits =
				(uint16_t)((rx->payload[2] << 8) | rx->payload[1]);
			s_motors[motor_index].encoder_raw.speed_bits =
				(uint16_t)((rx->payload[6] << 8) | rx->payload[5]);
			s_motors[motor_index].encoder_raw.timestamp_ms = now_ms;
			s_motors[motor_index].encoder_received = true;
		}
	} else {
		// Out-of-sequence or malformed frame: drop any partial transfer
		rx->len = 0;
		++s_driver_health.status_seq_drops;
	}
}

/**
 * @brief Check that a frame is a complete single-frame transfer of the expected length
 * @param[in] rx_data      Frame payload
 * @param[in] len          Frame payload length in bytes
 * @param[in] expected_len Required payload length, including the tail byte
 * @return true if the length matches and the tail has start and end set and toggle clear
 */
static bool is_single_frame(const uint8_t *rx_data, uint32_t len, uint32_t expected_len) {
	if (len != expected_len) {
		return false;
	}
	uint8_t tail = rx_data[len - 1];
	return ((tail & TAIL_START_OF_TRANSFER) != 0) && ((tail & TAIL_END_OF_TRANSFER) != 0) &&
	       ((tail & TAIL_TOGGLE) == 0);
}

/**
 * @brief Handle one NodeStatus frame and update the motor's health. Called from ISR context.
 * @param[in] motor_index Index into s_motors of the sending motor
 * @param[in] rx_data     Frame payload
 * @param[in] len         Frame payload length in bytes
 * @param[in] now_ms      Time the frame was received
 */
static void handle_node_status(int motor_index, const uint8_t *rx_data, uint32_t len,
                               uint32_t now_ms) {
	if (!is_single_frame(rx_data, len, NODE_STATUS_FRAME_LEN)) {
		++s_driver_health.rx_errors;
		return;
	}
	s_motors[motor_index].health.node_health =
		(rx_data[4] >> NODE_STATUS_HEALTH_SHIFT) & NODE_STATUS_HEALTH_MASK;
	s_motors[motor_index].health.node_mode =
		(rx_data[4] >> NODE_STATUS_MODE_SHIFT) & NODE_STATUS_MODE_MASK;
	s_motors[motor_index].health.nodestatus_timestamp_ms = now_ms;
	// Set after health is written, so anything waiting on heard sees complete data
	s_motors[motor_index].heard = true;
}

/**
 * @brief Handle one CircuitStatus frame and store its raw values. Called from ISR context.
 * @param[in] motor_index Index into s_motors of the sending motor
 * @param[in] rx_data     Frame payload
 * @param[in] len         Frame payload length in bytes
 * @param[in] now_ms      Time the frame was received
 */
static void handle_circuit_status(int motor_index, const uint8_t *rx_data, uint32_t len,
                                  uint32_t now_ms) {
	if (!is_single_frame(rx_data, len, CIRCUIT_STATUS_FRAME_LEN)) {
		++s_driver_health.rx_errors;
		return;
	}
	// Store raw float16 bits; get_health converts them in task context
	s_motors[motor_index].health.voltage_bits = (uint16_t)(rx_data[2] | (rx_data[3] << 8));
	s_motors[motor_index].health.current_bits = (uint16_t)(rx_data[4] | (rx_data[5] << 8));
	s_motors[motor_index].health.circuit_err_flags = rx_data[6];
	s_motors[motor_index].health.circuit_timestamp_ms = now_ms;
	s_motors[motor_index].circuit_received = true;
}

/**
 * @brief Handle one Temperature frame and store its raw values. Called from ISR context.
 * @param[in] motor_index Index into s_motors of the sending motor
 * @param[in] rx_data     Frame payload
 * @param[in] len         Frame payload length in bytes
 * @param[in] now_ms      Time the frame was received
 */
static void handle_temperature(int motor_index, const uint8_t *rx_data, uint32_t len,
                               uint32_t now_ms) {
	if (!is_single_frame(rx_data, len, TEMPERATURE_FRAME_LEN)) {
		++s_driver_health.rx_errors;
		return;
	}
	// Store raw float16 bits; get_health converts them in task context
	s_motors[motor_index].health.temperature_bits = (uint16_t)(rx_data[2] | (rx_data[3] << 8));
	s_motors[motor_index].health.temp_err_flags = rx_data[4];
	s_motors[motor_index].health.temp_timestamp_ms = now_ms;
	s_motors[motor_index].temperature_received = true;
}

/**
 * @brief Check whether NodeStatus has been received from every configured motor
 * @return true if all configured motors have been heard, false otherwise
 */
static bool all_motors_heard(void) {
	for (hbl3850_motor_id_t motor_id = 0; motor_id < HBL3850_MOTOR_COUNT; ++motor_id) {
		if ((!s_motors[motor_id].heard) && s_motors[motor_id].configured) {
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
	for (hbl3850_motor_id_t motor_id = 0; motor_id < HBL3850_MOTOR_COUNT; ++motor_id) {
		if ((s_motors[motor_id].node_id == node_id) && s_motors[motor_id].configured) {
			return motor_id;
		}
	}
	return -1;
}

w_status_t hbl3850_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms) {
	if (s_driver_health.is_init) {
		++s_driver_health.reinit_attempts;
		return W_FAILURE;
	}

	if (NULL == hfdcan) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Invalid pointers");
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}

	g_hfdcan = hfdcan;
	bool configured_motors = false;
	for (hbl3850_motor_id_t motor_id = 0; motor_id < HBL3850_MOTOR_COUNT; ++motor_id) {
		if (s_motors[motor_id].configured) {
			configured_motors = true;
		}
	}
	if (!configured_motors) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No motors are configured");
		++s_driver_health.invalid_args;
		return W_FAILURE;
	}

	// Reset the RX state of each configured motor before receive interrupts start
	for (hbl3850_motor_id_t motor_id = 0; motor_id < HBL3850_MOTOR_COUNT; ++motor_id) {
		if (s_motors[motor_id].configured) {
			s_motors[motor_id].encoder_received = false;
			s_motors[motor_id].heard = false;
			s_motors[motor_id].circuit_received = false;
			s_motors[motor_id].temperature_received = false;
			s_motors[motor_id].status_rx.len = 0;
			// TODO: also clear health and encoder_raw, so a retried init (e.g. after a slot was
			// reassigned to a new node ID) can't return the previous motor's stale data
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
		++s_driver_health.init_fdcan_filter_cfg_fails;
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN filter config failed");
		return W_FAILURE;
	}

	if (HAL_FDCAN_Start(g_hfdcan) != HAL_OK) {
		++s_driver_health.init_fdcan_start_fails;
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN start failed");
		stop_can();
		return W_FAILURE;
	}

	if (HAL_FDCAN_ActivateNotification(g_hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		++s_driver_health.init_fdcan_notification_fails;
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "FDCAN activate notification failed");
		stop_can();
		return W_FAILURE;
	}
	// Wait for NodeStatus from every configured motor
	uint32_t start_ms = 0;
	if (timer_get_ms(&start_ms) != W_SUCCESS) {
		++s_driver_health.timer_get_ms_fails;
		stop_can();
		return W_FAILURE;
	}
	uint32_t now_ms = start_ms;
	while ((!all_motors_heard()) && ((now_ms - start_ms) <= can_init_timeout_ms)) {
		// TODO: sleep min(500, remaining) so a short can_init_timeout_ms isn't overshot by up to
		// 500 ms
		vTaskDelay(pdMS_TO_TICKS(INIT_POLL_PERIOD_MS));
		if (timer_get_ms(&now_ms) != W_SUCCESS) {
			++s_driver_health.timer_get_ms_fails;
			stop_can();
			return W_FAILURE;
		}
	}
	if (!all_motors_heard()) {
		for (hbl3850_motor_id_t motor_id = 0; motor_id < HBL3850_MOTOR_COUNT; ++motor_id) {
			if (s_motors[motor_id].configured && (!s_motors[motor_id].heard)) {
				log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No NodeStatus from node %u",
				         s_motors[motor_id].node_id);
			}
		}
		++s_driver_health.init_fdcan_timeout;
		stop_can();
		return W_FAILURE;
	}
	s_driver_health.is_init = true;
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
	if ((NULL == g_hfdcan) || (NULL == data) || (len > CAN_CLASSIC_MAX_LEN)) {
		++s_driver_health.invalid_args;
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
		++s_driver_health.tx_errors;
		return W_FAILURE;
	}
	return W_SUCCESS;
}

w_status_t hbl3850_send_position_cmd(hbl3850_motor_id_t motor, float32_t angle_deg) {
	if ((motor >= HBL3850_MOTOR_COUNT) || isnan(angle_deg)) {
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}
	if (!s_motors[motor].configured) {
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}
	// ArrayCommand payload: actuator ID, command type, float16 value (low byte first), tail.
	// The HAL copies TX data 4 bytes at a time, so the buffer is 8 bytes and only 5 are sent.
	uint8_t data[CAN_CLASSIC_MAX_LEN] = {0};
	data[0] = s_motors[motor].actuator_id;
	data[1] = COMMAND_TYPE_UNITLESS;
	// The motor maps the unitless range -1..1 to -60..+60 degrees
	float32_t converted_angle = angle_deg / HBL3850_MAX_ANGLE_DEG;
	// TODO: these clamp logs fire on every call while the FSM commands past +/-60 degrees (~100
	// Hz). Count clamps in s_driver_health and report from get_status instead.
	if (converted_angle > 1.0F) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Command angle is over 60 degrees");
		converted_angle = 1.0F;
	}
	if (converted_angle < -1.0F) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Command angle is lower than -60 degrees");
		converted_angle = -1.0F;
	}
	uint16_t angle_bits = float_to_half(converted_angle);

	data[2] = (uint8_t)(angle_bits & 0xff);
	data[3] = angle_bits >> 8;
	// Single-frame transfer, so start and end are both set
	data[4] = TAIL_START_OF_TRANSFER | TAIL_END_OF_TRANSFER | g_cmd_transfer_id;
	g_cmd_transfer_id = (g_cmd_transfer_id + 1) & TAIL_TRANSFER_ID_MASK;

	return hbl3850_can_transmit_ext(can_id, data, ARRAY_COMMAND_LEN);
}

w_status_t hbl3850_get_encoder(hbl3850_motor_id_t motor, hbl3850_encoder_t *encoder) {
	if (motor >= HBL3850_MOTOR_COUNT) {
		++s_driver_health.invalid_args;
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "motor does not exist.");
		return W_INVALID_PARAM;
	}
	if (NULL == encoder) {
		++s_driver_health.invalid_args;
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Encoder does not exist");
		return W_FAILURE;
	}
	if (!s_motors[motor].configured) {
		++s_driver_health.invalid_args;
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "motor is not configured");
		return W_INVALID_PARAM;
	}

	// Copy the latest reading and its flag together, since the RX ISR writes them
	taskENTER_CRITICAL();
	hbl3850_encoder_raw_t raw = s_motors[motor].encoder_raw;
	bool received = s_motors[motor].encoder_received;
	taskEXIT_CRITICAL();

	if (!received) {
		// TODO: this fires on every call until the first Status arrives (FSM polls at ~100 Hz).
		// Count it in s_driver_health and report from get_status instead.
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No feedback received yet");
		return W_FAILURE;
	}

	// Convert outside the critical section and the ISR to keep both short
	// TODO: verify on bench that the motor reports radians (DSDL) and not degrees (MKS doc)
	encoder->position_deg = half_to_float(raw.pos_bits) * HBL3850_RAD_TO_DEG;
	encoder->speed_dps = half_to_float(raw.speed_bits) * HBL3850_RAD_TO_DEG;
	encoder->timestamp_ms = raw.timestamp_ms;
	return W_SUCCESS;
}

/**
 * @brief FDCAN RX FIFO1 callback for HBL3850 DroneCAN messages
 *
 * Called from ISR context for every frame in RX FIFO1. Reassembles multi-frame
 * Actuator.Status transfers into the motor's latest encoder reading, and decodes NodeStatus,
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
	uint8_t rx_data[CAN_CLASSIC_MAX_LEN];

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) {
		++s_driver_health.rx_errors;
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
	if (service || (motor_index == -1)) {
		return;
	}

	uint32_t now_ms = 0;

	if (timer_get_ms(&now_ms) != W_SUCCESS) {
		++s_driver_health.timer_get_ms_fails;
		++s_driver_health.rx_errors;
		return;
	}

	uint32_t len = rx_header.DataLength;
	if (typeID == ACTUATOR_STATUS_TYPE) {
		handle_actuator_status(motor_index, rx_data, len, now_ms);
	} else if (typeID == NODE_STATUS_TYPE) {
		handle_node_status(motor_index, rx_data, len, now_ms);
	} else if (typeID == CIRCUIT_STATUS_TYPE) {
		handle_circuit_status(motor_index, rx_data, len, now_ms);
	} else if (typeID == TEMPERATURE_STATUS_TYPE) {
		handle_temperature(motor_index, rx_data, len, now_ms);
	}
}

/**
 * @brief Copy a motor's health state, validity flags and the current time together, since the RX
 * ISR writes them. Reading the clock in the same critical section means a timestamp refreshed by
 * the ISR can never be newer than now_ms. timer_get_ms only reads a timer register, so it's safe
 * here.
 * @param[in]  motor_id Index into s_motors
 * @param[out] snapshot Copy of the motor's state
 * @return W_SUCCESS, or W_FAILURE if the current time couldn't be read. The other fields are copied
 * either way, so the caller can still check snapshot->configured
 */
static w_status_t take_motor_snapshot(hbl3850_motor_id_t motor_id,
                                      hbl3850_motor_snapshot_t *snapshot) {
	w_status_t status = W_SUCCESS;
	// Single exit after taskEXIT_CRITICAL, so interrupts are always re-enabled
	taskENTER_CRITICAL();
	if (timer_get_ms(&snapshot->now_ms) != W_SUCCESS) {
		status = W_FAILURE;
	}
	snapshot->configured = s_motors[motor_id].configured;
	snapshot->node_id = s_motors[motor_id].node_id;
	snapshot->health = s_motors[motor_id].health;
	snapshot->node_received = s_motors[motor_id].heard;
	snapshot->circuit_received = s_motors[motor_id].circuit_received;
	snapshot->temperature_received = s_motors[motor_id].temperature_received;
	taskEXIT_CRITICAL();
	return status;
}

health_status_t hbl3850_get_status(void) {
	health_status_t status = {
		.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
		// TODO: add CANARDS_MODULE_ID_HBL3850 to rocketlib. AK45 is a placeholder; the two motor
		// drivers are never built together.
		.module_id = CANARDS_MODULE_ID_AK45,

		.error_bitfield = 0
	};

	bool any_configured = false;

	if (!s_driver_health.is_init) {
		status.error_bitfield |= (1U << CANARDS_MODULE_E_NOT_INIT_OFFSET);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Driver not initialized");
	}

	for (unsigned int motor_id = 0; motor_id < HBL3850_MOTOR_COUNT; ++motor_id) {
		hbl3850_motor_snapshot_t snap;
		w_status_t snap_status = take_motor_snapshot((hbl3850_motor_id_t)motor_id, &snap);

		// Only configured motors are checked (no continue/break, per safety-critical style)
		if (snap.configured) {
			any_configured = true;

			if (snap_status != W_SUCCESS) {
				++s_driver_health.timer_get_ms_fails;
				status.error_bitfield |= (1U << CANARDS_MODULE_E_INTERNAL_OFFSET);
				log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Failed to get time");
			} else {
				if (!snap.node_received) {
					status.error_bitfield |= (1U << CANARDS_MODULE_E_TIMEOUT_OFFSET);
					log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
					         "Motor %u (node %u): no NodeStatus", motor_id, snap.node_id);
				} else {
					if ((uint32_t)(snap.now_ms - snap.health.nodestatus_timestamp_ms) >
					    HBL3850_NODE_TIMEOUT_MS) {
						status.error_bitfield |= (1U << CANARDS_MODULE_E_TIMEOUT_OFFSET);
						log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
						         "Motor %u (node %u): offline", motor_id, snap.node_id);
					}

					if (snap.health.node_health != 0U) {
						status.error_bitfield |= (1U << CANARDS_MODULE_E_ERROR_STATE_OFFSET);
						log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
						         "Motor %u (node %u): NodeStatus health %u", motor_id, snap.node_id,
						         snap.health.node_health);
					}
				}

				if ((!snap.circuit_received) ||
				    ((uint32_t)(snap.now_ms - snap.health.circuit_timestamp_ms) >
				     HBL3850_CIRCUIT_TIMEOUT_MS)) {
					status.error_bitfield |= (1U << CANARDS_MODULE_E_NO_DATA_OFFSET);
					log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
					         "Motor %u (node %u): no recent CircuitStatus", motor_id, snap.node_id);
				} else if (snap.health.circuit_err_flags != HBL3850_CIRCUIT_ERR_NONE) {
					status.error_bitfield |= (1U << CANARDS_MODULE_E_DEVICE_FAULT_OFFSET);
					log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
					         "Motor %u (node %u): circuit error flags 0x%02x", motor_id,
					         snap.node_id, snap.health.circuit_err_flags);
				}

				if ((!snap.temperature_received) ||
				    ((uint32_t)(snap.now_ms - snap.health.temp_timestamp_ms) >
				     HBL3850_TEMP_TIMEOUT_MS)) {
					status.error_bitfield |= (1U << CANARDS_MODULE_E_NO_DATA_OFFSET);
					log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
					         "Motor %u (node %u): no recent Temperature", motor_id, snap.node_id);
				} else if (snap.health.temp_err_flags != HBL3850_TEMP_ERR_NONE) {
					status.error_bitfield |= (1U << CANARDS_MODULE_E_DEVICE_FAULT_OFFSET);
					log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850",
					         "Motor %u (node %u): temperature error flags 0x%02x", motor_id,
					         snap.node_id, snap.health.temp_err_flags);
				}
			}
		}
	}

	if (!any_configured) {
		status.error_bitfield |= (1U << CANARDS_MODULE_E_NOT_INIT_OFFSET);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "No motors configured");
	}

	// Driver-wide errors: report only if the counter went up since the last health check
	// TODO: read each counter once into a local and use that for the compare, log and baseline, so
	// an ISR increment between the log and the baseline update isn't lost

	if (s_driver_health.tx_errors != last_tx_errors) {
		status.error_bitfield |= (1U << CANARDS_MODULE_E_TX_FAILURE_OFFSET);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "TX errors: %lu", s_driver_health.tx_errors);
		last_tx_errors = s_driver_health.tx_errors;
	}
	if (s_driver_health.rx_errors != last_rx_errors) {
		status.error_bitfield |= (1U << CANARDS_MODULE_E_RX_FAILURE_OFFSET);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "RX errors: %lu", s_driver_health.rx_errors);
		last_rx_errors = s_driver_health.rx_errors;
	}
	if (s_driver_health.status_crc_fails != last_crc_fails) {
		status.error_bitfield |= (1U << CANARDS_MODULE_E_CRC_FAILED_OFFSET);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "hbl3850", "Status CRC fails: %lu",
		         s_driver_health.status_crc_fails);
		last_crc_fails = s_driver_health.status_crc_fails;
	}

	// Set severity last, after every bit has been added
	if (status.error_bitfield != 0U) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
	}

	return status;
}

w_status_t hbl3850_get_health(hbl3850_motor_id_t motor, hbl3850_health_t *health_stats) {
	if ((unsigned int)motor >= HBL3850_MOTOR_COUNT) {
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}

	if (!s_motors[motor].configured) {
		++s_driver_health.invalid_args;
		return W_INVALID_PARAM;
	}

	if (health_stats == NULL) {
		++s_driver_health.invalid_args;
		return W_FAILURE;
	}

	// Copy the raw data and its validity flags together, since the RX ISR writes them
	taskENTER_CRITICAL();
	hbl3850_health_raw_t raw = s_motors[motor].health;
	bool any_received = s_motors[motor].heard || s_motors[motor].circuit_received ||
	                    s_motors[motor].temperature_received;
	taskEXIT_CRITICAL();

	if (!any_received) {
		return W_FAILURE;
	}

	// Convert outside the critical section and the ISR to keep both short
	health_stats->node_health = raw.node_health;
	health_stats->node_mode = raw.node_mode;
	health_stats->nodestatus_timestamp_ms = raw.nodestatus_timestamp_ms;
	health_stats->voltage_v = half_to_float(raw.voltage_bits);
	health_stats->current_a = half_to_float(raw.current_bits);
	health_stats->circuit_err_flags = raw.circuit_err_flags;
	health_stats->circuit_timestamp_ms = raw.circuit_timestamp_ms;
	// DroneCAN sends temperature in Kelvin
	health_stats->temperature_c = half_to_float(raw.temperature_bits) - KELVIN_TO_CELSIUS_OFFSET;
	health_stats->temp_err_flags = raw.temp_err_flags;
	health_stats->temp_timestamp_ms = raw.temp_timestamp_ms;
	return W_SUCCESS;
}

#endif
