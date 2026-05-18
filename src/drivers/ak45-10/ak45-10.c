#include "drivers/ak45-10/ak45-10.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include <string.h>

#define LOG_WAIT_MS 10

// CAN command IDs
typedef enum {
	CAN_PACKET_SET_DUTY = 0, // Duty cycle mode
	CAN_PACKET_SET_CURRENT = 1, // Current loop mode
	CAN_PACKET_SET_CURRENT_BRAKE = 2, // Current brake mode
	CAN_PACKET_SET_RPM = 3, // Velocity mode
	CAN_PACKET_SET_POS = 4, // Position mode (likely most optimal)
	CAN_PACKET_SET_ORIGIN_HERE = 5, // Set origin mode
	CAN_PACKET_SET_POS_SPD = 6 // Position velocity loop mode
} can_packet_id_t;

// Feedback message mode ID
#define CAN_PACKET_FEEDBACK 0x10

// Scaling factors
static const float AK45_POS_CMD_SCALE = 10000.0f; // Position: degrees * 10000
static const float AK45_POS_FB_SCALE = 0.1f; // Feedback position: raw * 0.1 = degrees
static const float AK45_SPEED_FB_SCALE = 10.0f; // speed feedback: raw * 10.0 = ERPM
static const float AK45_CURRENT_FB_SCALE = 0.01f; // current feedback: raw * 0.01 = Amps

static FDCAN_HandleTypeDef *ak45_hfdcan = NULL;
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
static w_status_t ak45_can_transmit_ext(uint32_t ext_id, const uint8_t *data, uint8_t len) {
	if ((NULL == ak45_hfdcan) || (NULL == data) || (len > 8)) {
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
static void ak45_parse_feedback(const uint8_t *data, ak45_feedback_t *fb) {
	int16_t raw_pos = ((data[0] << 8) | data[1]);
	int16_t raw_speed = ((data[2] << 8) | data[3]);
	int16_t raw_current = ((data[4] << 8) | data[5]);

	fb->position_deg = (float)raw_pos * AK45_POS_FB_SCALE;
	fb->speed_erpm = (float)raw_speed * AK45_SPEED_FB_SCALE;
	fb->current_a = (float)raw_current * AK45_CURRENT_FB_SCALE;

	fb->temperature_c = (float)data[6];
	fb->fault_code = (ak45_fault_code_t)data[7];
}

w_status_t ak45_init(FDCAN_HandleTypeDef *hfdcan) {
	if (NULL == hfdcan) {
		return W_INVALID_PARAM;
	}

	ak45_hfdcan = hfdcan;

	// todo: create queuue and fdcan filter

	return W_SUCCESS;
}

w_status_t ak45_send_position_cmd(float angle_deg) {
	// todo: define data
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_POS << 8) | AK45_10_ID;
	uint8_t data[4] = {0}; // placeholder data

	return ak45_can_transmit_ext(ext_id, data, 4);
}

w_status_t ak45_send_disable_cmd(void) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | AK45_10_ID;
	uint8_t data[4] = {0, 0, 0, 0};

	return ak45_can_transmit_ext(ext_id, data, 4);
}

w_status_t ak45_get_latest_feedback(ak45_feedback_t *fb) {
	if (NULL == fb) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

uint32_t ak45_get_tx_errors(void) {
	return tx_errors;
}

/**
 * @brief FDCAN RX FIFO1 callback for servo feedback messages
 *
 * Called from ISR context when a message matching the servo feedback filter
 * is received. Parses the feedback and puts it in the queue.
 */
static void ak45_fdcan_rx_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	if (0 == RxFifo1ITs) {
		return;
	}

	// Placeholder struct and data for feedback parsing function
	uint8_t rx_data[8] = {0};
	ak45_feedback_t fb = {0};
	ak45_parse_feedback(rx_data, &fb);

	// Queue feedback (overwrite old data)
	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueOverwriteFromISR(feedback_queue, &fb, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief Override weak HAL FDCAN RX FIFO1 callback
 *
 * Called by HAL when a new message arrives in RX FIFO1.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	ak45_fdcan_rx_callback(hfdcan, RxFifo1ITs);
}