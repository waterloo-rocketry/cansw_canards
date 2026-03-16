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
	int16_t pos_raw = (data[0] << 8 | data[1]);
	fb->position = pos_raw * MOTOR_POS_FB_SCALE;
	int16_t speed_raw = (data[2] << 8 | data[3]);
	fb->speed = speed_raw * MOTOR_SPEED_FB_SCALE;
	int16_t current_raw = (data[4] << 8 | data[5]);
	fb->current = current_raw * MOTOR_CURRENT_FB_SCALE;

	fb->temperature = data[6];
	fb->fault_code = data[7];
}

w_status_t motor_driver_init(FDCAN_HandleTypeDef *hfdcan) {
	if (hfdcan == NULL) {
		return W_INVALID_PARAM;
	}

	motor_hfdcan = hfdcan;

	//todo: create queuue and fdcan filter

	return W_SUCCESS;
}

w_status_t motor_send_position_cmd(float angle_deg) {
	//todo: define ext id and data
	return motor_can_transmit_ext(ext_id, data, 4);
}

w_status_t motor_send_disable_cmd(void) {
	//todo: define ext id and data
	return motor_can_transmit_ext(ext_id, data, 4);
}

bool motor_is_fatal_fault(motor_fault_code_t fault) {
	switch (fault) {
		case MOTOR_FAULT_OVERVOLTAGE:
		case MOTOR_FAULT_OVERCURRENT:
		case MOTOR_FAULT_OVERTEMP_FET:
		case MOTOR_FAULT_OVERTEMP_MOTOR:
		case MOTOR_FAULT_ENCODER_ERROR;
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