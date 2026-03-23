#include "drivers/motor_driver/motor_driver.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include <string.h>

#define LOGGER_WAIT_MS 10

// CAN command IDs
typedef enum {
	CAN_PACKET_SET_DUTY         = 0, // Duty cycle mode
	CAN_PACKET_SET_CURRENT      = 1, // Current loop mode
	CAN_PACKET_SET_CURRENT_BRAKE = 2, // Current brake mode
	CAN_PACKET_SET_RPM          = 3, // Velocity mode
	CAN_PACKET_SET_POS          = 4, // Position mode (likely most optimal)
	CAN_PACKET_SET_ORIGIN_HERE  = 5, // Set origin mode
	CAN_PACKET_SET_POS_SPD      = 6, // Position velocity loop mode
} can_packet_id_t;

// Feedback message mode ID
#define CAN_PACKET_FEEDBACK 0x10

// Scaling factors
#define MOTOR_POS_CMD_SCALE    10000.0f // Position: degrees * 10000
#define MOTOR_POS_FB_SCALE     0.1f    // Feedback position: raw * 0.1 = degrees
#define MOTOR_SPEED_FB_SCALE   10.0f    // speed feedback: raw * 10.0 = ERPM
#define MOTOR_CURRENT_FB_SCALE 0.01f   // current feedback: raw * 0.01 = Amps

static FDCAN_HandleTypeDef *motor_hfdcan = NULL;
static QueueHandle_t feedback_queue = NULL;
static uint32_t tx_errors = 0;
static bool is_init = false;

/**
 * @brief Transmit 29-bit ID via FDCAN
 *
 * @param[in] ext_id  29-bit extended CAN ID
 * @param[in] data    Pointer to payload data
 * @param[in] len     Payload length
 * @return W_SUCCESS on success, W_FAILURE on error
 */
static w_status_t motor_can_transmit_ext(uint32_t ext_id, const uint8_t *data, uint8_t len) {
	if (motor_hfdcan == NULL || data == NULL || len > 8) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * @brief Parse a raw 8-byte CAN feedback frame from the AK45 motor
 *
 * @param[in]  data   8-byte feedback payload
 * @param[out] fb     Decoded feedback struct
 */
static void motor_parse_feedback(const uint8_t *data, motor_feedback_t *fb) {
	int16_t raw_pos = ((data[0] << 8) | data[1]);
	fb->position_deg = (float)raw_pos * MOTOR_POS_FB_SCALE;

	int16_t raw_speed = ((data[2] << 8) | data[3]);
	fb->speed_erpm = (float)raw_speed * MOTOR_SPEED_FB_SCALE;

	int16_t raw_current = ((data[4] << 8) | data[5]);
	fb->current_a = (float)raw_current * MOTOR_CURRENT_FB_SCALE;

	fb->temperature_c = (float)data[6];
	fb->fault_code = (motor_fault_code_t)data[7];

	float ms = 0.0f
	if (timer_get_time_ms(&ms) == W_SUCCESS) {
		fb->timestamp_ms = (uint32_t)ms;
	}
}

w_status_t motor_driver_init(FDCAN_HandleTypeDef *hfdcan) {
	if (hfdcan == NULL) {
		return W_INVALID_PARAM;
	}

	motor_hfdcan = hfdcan;

	// feedback queue with length 1
	feedback_queue = xQueueCreate(1, sizeof(motor_feedback_t));
	if (feedback_queue == NULL){
		return W_FAILURE;
	}

	FDCAN_FilterTypeDef motor_filter = {0};
	motor_filter.IdType = FDCAN_EXTENDED_ID;
	motor_filter.FilterIndex = 0;
	motor_filter.FilterType = FDCAN_FILTER_MASK;
	motor_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	motor_filter.FilterID1 = MOTOR_DRIVER_ID | (uint32_t)(CAN_PACKET_FEEDBACK << 8);
	motor_filter.FilterID2 = 0x1FFFFFFF;

	if (HAL_FDCAN_ConfigFilter(motor_hfdcan, &motor_filter) != HAL_OK) {
		return W_FAILURE;
	}

	tx_errors = 0;
	is_init = true;

	return W_SUCCESS;
}

w_status_t motor_send_position_cmd(float angle_deg) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_POS << 8) | MOTOR_DRIVER_ID;

	int32_t pos_raw = (int32_t)(angle_deg * MOTOR_POS_CMD_SCALE);
	uint8_t data[4];
	data[0] = (uint8_t)((pos_raw >> 24) & 0xFF);
	data[1] = (uint8_t)((pos_raw >> 16) & 0xFF);
	data[2] = (uint8_t)((pos_raw >> 8) & 0xFF);
	data[3] = (uint8_t)(pos_raw & 0xFF);

	return motor_can_transmit_ext(ext_id, data, 4);
}

w_status_t motor_send_disable_cmd(void) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | MOTOR_DRIVER_ID;
	uint8_t data[4] = {0, 0, 0, 0};
	return motor_can_transmit_ext(ext_id, data, 4);
}

bool motor_is_fatal_fault(motor_fault_code_t fault) {
	switch (fault) {
		case MOTOR_FAULT_OVERVOLTAGE:
		case MOTOR_FAULT_ABS_OVERCURRENT:
		case MOTOR_FAULT_OVERTEMP_FET:
		case MOTOR_FAULT_OVERTEMP_MOTOR:
			return true;
		default:
			return false;
	}
}

w_status_t motor_get_latest_feedback(motor_feedback_t *fb) {
	if (fb == NULL) {
		return W_FAILURE;
	}
	
	return W_SUCCESS
}

uint32_t motor_get_tx_errors(void) {
	return tx_errors;
}

/**
 * @brief FDCAN RX FIFO1 callback for servo feedback messages
 *
 * Called from ISR context when a message matching the servo feedback filter
 * is received. Parses the feedback and puts it in the queue.
 */
static void motor_fdcan_rx_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	if (RxFifo1ITs == 0) {
		return;
	}
}