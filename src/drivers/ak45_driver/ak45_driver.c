#include <string.h>

#include "FreeRTOS.h"
#include "fdcan.h"
#include "queue.h"

#include "application/logger/log.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "drivers/timer/timer.h"

// Motor CAN driver ID
static const uint16_t AK45_DRIVER_ID = 0x45;
static const uint16_t LOG_WAIT_MS = 1;

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
static const uint16_t CAN_REAL_TIME_FEEDBACK = 0x29;
static const uint16_t CAN_START_FRAME = 0x2C;

// Scaling factors
static const float32_t AK45_POS_CMD_DEG_TO_POS = 10000.0f; // Position: degrees * 10000
static const float32_t AK45_POS_FB_TO_DEG = 0.1f; // Feedback position: raw * 0.1 = degrees
// eRPM is electrically counted RPM
static const float32_t AK45_SPEED_FB_TO_ERPM = 10.0f; // speed feedback: raw * 10.0 = ERPM
static const float32_t AK45_CURRENT_FB_TO_A = 0.01f; // current feedback: raw * 0.01 = Amps

static FDCAN_HandleTypeDef *g_ak45_hfdcan = NULL;
static QueueHandle_t g_feedback_queue = NULL;
static volatile bool received_can_msg = false;

/**
 * Status variables describing the health of the AK45 driver module
 */
typedef struct {
	// TODO: implement calibration step and add the two checks
	bool is_init;
	bool hard_stop_calibrated; // false until hard stop calibration succeeds
	bool hard_stop_cal_failed; // set true on failed attempt, clear on success
	bool feedback_rx_failed; // set true on failed attempt, clear on success
	bool cmd_tx_failed; // set true on failed attempt, clear on success
	uint32_t rx_errors; // FDCAN RX-callback failures
	uint32_t tx_errors; // FDCAN transmit FIFO add failures
	uint32_t invalid_args; // NULL or out of range arguments to functions
	uint32_t out_of_memory; // failure to allocate memory for queue creation
	uint32_t not_initialized; // calls to get_feedback when driver is not initialized
	uint32_t feedback_queue_empty; // calls for latest feedback when queue empty
	uint32_t reinit_attempts; // count of init attempts after first successful init
	uint32_t fdcan_stop_fails; // count of failures to stop FDCAN bus
	uint32_t init_fdcan_timeout; // count of timeouts to receive response from the motor during init
	uint32_t init_fdcan_notification_fails; // Count of failures to activate FDCAN notification
											// during init
	uint32_t init_fdcan_start_fails; // count of failures to start FDCAN bus
	uint32_t init_fdcan_filter_cfg_fails; // count of failures to configure FDCAN filter during init
} ak45_health_t;

static ak45_health_t ak45_health = {0};

/**
 * @brief Transmit 29-bit ID via FDCAN
 *
 * @param[in] ext_id  29-bit extended CAN ID
 * @param[in] data    Pointer to payload data
 * @param[in] len     Payload length
 * @return W_SUCCESS on success, W_FAILURE on error
 */
static w_status_t ak45_can_transmit_ext(uint32_t ext_id, const uint8_t *data, uint8_t len) {
	if ((NULL == g_ak45_hfdcan) || (NULL == data) || (len > 8)) {
		ak45_health.invalid_args++;
		return W_FAILURE;
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

	if (HAL_FDCAN_AddMessageToTxFifoQ(g_ak45_hfdcan, &tx_header, data) != HAL_OK) {
		ak45_health.tx_errors++;
		ak45_health.cmd_tx_failed = true;
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
static w_status_t ak45_parse_feedback(const uint8_t *data, ak45_feedback_t *fb) {
	if ((NULL == data) || (NULL == fb)) {
		ak45_health.invalid_args++;
		return W_FAILURE;
	}

	int16_t raw_pos = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
	fb->position_deg = (float32_t)raw_pos * AK45_POS_FB_TO_DEG;

	int16_t raw_speed = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
	fb->speed_erpm = (float32_t)raw_speed * AK45_SPEED_FB_TO_ERPM;

	int16_t raw_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
	fb->current_a = (float32_t)raw_current * AK45_CURRENT_FB_TO_A;

	fb->temperature_c = (int8_t)data[6];
	fb->fault_code = (ak45_fault_code_t)data[7];

	uint32_t ms = 0;
	if (timer_get_ms(&ms) == W_SUCCESS) {
		fb->timestamp_ms = ms;
		return W_SUCCESS;
	}

	return W_FAILURE;
}

/* used to stop the general can bus*/
static void ak45_stop_can() {
	// turn off fdcan so can restart
	if (HAL_FDCAN_Stop(g_ak45_hfdcan) != HAL_OK) {
		ak45_health.fdcan_stop_fails++;
	}
}

w_status_t ak45_send_position_cmd(float32_t angle_deg) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_POS << 8) | AK45_DRIVER_ID;

	int32_t pos_raw = (int32_t)(angle_deg * AK45_POS_CMD_DEG_TO_POS);
	uint8_t data[4];
	data[0] = (uint8_t)((pos_raw >> 24) & 0xFF);
	data[1] = (uint8_t)((pos_raw >> 16) & 0xFF);
	data[2] = (uint8_t)((pos_raw >> 8) & 0xFF);
	data[3] = (uint8_t)(pos_raw & 0xFF);

	return ak45_can_transmit_ext(ext_id, data, FDCAN_DLC_BYTES_4);
}

w_status_t ak45_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms) {
	// check if the driver has inited
	if (ak45_health.is_init) {
		ak45_health.reinit_attempts++;
		return W_FAILURE;
	}

	if (NULL == hfdcan) {
		ak45_health.invalid_args++;
		return W_INVALID_PARAM;
	}

	g_ak45_hfdcan = hfdcan;

	// feedback queue with length 1
	g_feedback_queue = xQueueCreate(1, sizeof(ak45_feedback_t));
	if (g_feedback_queue == NULL) {
		ak45_health.out_of_memory++;
		return W_FAILURE;
	}

	FDCAN_FilterTypeDef motor_filter = {0};
	motor_filter.IdType = FDCAN_EXTENDED_ID;
	motor_filter.FilterIndex = 0;
	motor_filter.FilterType = FDCAN_FILTER_DUAL;
	motor_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	motor_filter.FilterID1 = (uint32_t)(CAN_REAL_TIME_FEEDBACK << 8) | AK45_DRIVER_ID;
	motor_filter.FilterID2 = (uint32_t)(CAN_START_FRAME << 8) | AK45_DRIVER_ID;

	if (HAL_FDCAN_ConfigFilter(g_ak45_hfdcan, &motor_filter) != HAL_OK) {
		ak45_health.init_fdcan_filter_cfg_fails++;
		return W_FAILURE;
	}

	if (HAL_FDCAN_Start(g_ak45_hfdcan) != HAL_OK) {
		ak45_health.init_fdcan_start_fails++;
		ak45_stop_can();
		return W_FAILURE;
	}

	// to check if there has been a new can msg since start
	received_can_msg = false;

	if (HAL_FDCAN_ActivateNotification(g_ak45_hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		ak45_health.init_fdcan_notification_fails++;
		ak45_stop_can();
		return W_FAILURE;
	}
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8) | AK45_DRIVER_ID;

	// make sure we recieved a can msg before we send one
	uint32_t start_can_init_time_ms = 0;
	if (timer_get_ms(&start_can_init_time_ms) != W_SUCCESS) {
		ak45_stop_can();
		return W_FAILURE;
	}

	uint32_t curr_time_ms = start_can_init_time_ms;

	// This will terminate either after recieving a CAN msg from the motor or worse case after the
	// described timeout
	while ((!received_can_msg) &&
		   ((curr_time_ms - start_can_init_time_ms) <= can_init_timeout_ms)) {
		vTaskDelay(500);

		if (timer_get_ms(&curr_time_ms) != W_SUCCESS) {
			ak45_stop_can();
			return W_FAILURE;
		}
	}

	// return error if no response from motor
	if (!received_can_msg) {
		ak45_health.init_fdcan_timeout++;
		ak45_stop_can();
		return W_FAILURE;
	}

	// set current position to 0
	uint8_t zero_data[1] = {0};
	if (ak45_can_transmit_ext(ext_id, zero_data, FDCAN_DLC_BYTES_1) != W_SUCCESS) {
		ak45_stop_can();
		return W_FAILURE;
	}

	ak45_health.tx_errors = 0;
	ak45_health.is_init = true;

	return W_SUCCESS;
}

w_status_t ak45_send_disable_cmd(void) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | AK45_DRIVER_ID;
	uint8_t data[4] = {0, 0, 0, 0};
	return ak45_can_transmit_ext(ext_id, data, FDCAN_DLC_BYTES_4);
}

w_status_t ak45_get_latest_feedback(ak45_feedback_t *fb) {
	if (NULL == fb) {
		ak45_health.invalid_args++;
		return W_FAILURE;
	}

	if (!ak45_health.is_init) {
		ak45_health.not_initialized++;
		return W_FAILURE;
	}

	if (xQueuePeek(g_feedback_queue, fb, 0) == pdPASS) {
		return W_SUCCESS;
	}

	ak45_health.feedback_queue_empty++;
	return W_FAILURE; // empty queue or no feedback yet
}

uint32_t ak45_get_tx_errors(void) {
	return ak45_health.tx_errors;
}

/**
 * @brief FDCAN RX FIFO1 callback for servo feedback messages
 *
 * Called from ISR context when a message matching the servo feedback filter
 * is received. Parses the feedback and puts it in the queue.
 */
static void ak45_fdcan_rx_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	// the motor should be sending periodic state messages at 300hz so even if the motor's CAN
	// starts the STM we willl still recieve a msg
	received_can_msg = true;

	// checks if the new message bit is not 0 (FDCAN_IT_RX_FIFO1_NEW_MESSAGE is the bit mask)
	if (0 == (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {
		return;
	}

	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) {
		ak45_health.rx_errors++;
		ak45_health.feedback_rx_failed = true;
		return;
	}

	uint32_t expected_id = ((uint32_t)CAN_REAL_TIME_FEEDBACK << 8) | AK45_DRIVER_ID;

	if ((rx_header.IdType != FDCAN_EXTENDED_ID) || (rx_header.Identifier != expected_id)) {
		ak45_health.rx_errors++;
		ak45_health.feedback_rx_failed = true;
		return;
	}

	ak45_feedback_t fb = {0};
	if (ak45_parse_feedback(rx_data, &fb) != W_SUCCESS) {
		ak45_health.feedback_rx_failed = true;
	}

	// overwrite stale data if newer data is available
	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueOverwriteFromISR(g_feedback_queue, &fb, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	// make sure from the right can bus
	if (g_ak45_hfdcan == hfdcan) {
		ak45_fdcan_rx_callback(hfdcan, RxFifo1ITs);
	}
}

health_status_t ak45_get_status(void) {
	health_status_t status = {.severity = HEALTH_OK, .module_id = MODULE_AK45, .error_bitfield = 0};

	if (!ak45_health.hard_stop_calibrated) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_NOT_CALIBRATED;
	}

	if (ak45_health.hard_stop_cal_failed) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_FAILED_CALIBRATION;
	}

	if (!ak45_health.cmd_tx_failed) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_TX_FAILURE;
	}

	if (!ak45_health.feedback_rx_failed) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << ERR_RX_FAILURE;
	}

	if (!ak45_health.is_init) {
		status.severity = HEALTH_ERROR; // not sure if this should be fatal or error
		status.error_bitfield |= 1 << ERR_NOT_INIT;
	}

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "is_init=%d, hard_stop_calibrated=%d, hard_stop_cal_failed=%d, rx_errors=%lu, "
			 "tx_errors=%lu",
			 ak45_health.is_init,
			 ak45_health.hard_stop_calibrated,
			 ak45_health.hard_stop_cal_failed,
			 ak45_health.rx_errors,
			 ak45_health.tx_errors);

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "out_of_memory=%lu, not_initialized=%lu, feedback_queue_empty=%lu, "
			 "reinit_attempts=%lu, invalid_args=%lu",
			 ak45_health.out_of_memory,
			 ak45_health.not_initialized,
			 ak45_health.feedback_queue_empty,
			 ak45_health.reinit_attempts,
			 ak45_health.invalid_args);

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "init_timeout=%lu, init_notif_fails=%lu, init_start_fails=%lu, init_filter_fails=%lu, "
			 "fdcan_stop_fails=%lu",
			 ak45_health.init_fdcan_timeout,
			 ak45_health.init_fdcan_notification_fails,
			 ak45_health.init_fdcan_start_fails,
			 ak45_health.init_fdcan_filter_cfg_fails,
			 ak45_health.fdcan_stop_fails);

	ak45_health.hard_stop_calibrated = false;
	ak45_health.hard_stop_cal_failed = false;
	ak45_health.cmd_tx_failed = false;
	ak45_health.feedback_rx_failed = false;
	ak45_health.is_init = false;

	return status;
}
