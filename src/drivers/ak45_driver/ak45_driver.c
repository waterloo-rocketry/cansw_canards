#include <inttypes.h>
#include <string.h>

#include "FreeRTOS.h"
#include "fdcan.h"
#include "queue.h"

#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "application/telemetry/telemetry.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "drivers/timer/timer.h"

// Motor CAN driver ID
static const uint16_t AK45_DRIVER_ID = 0x45;
static const uint16_t LOG_WAIT_MS = 0;

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

static const uint32_t AK45_UPDATE_PERIOD_MS = 5;

const ak45_calibration_config_t ak45_calibration_config = {
	.seek_target_deg = 70.0f,
	.backoff_deg = 20.0f,
	.settle_ms = 2000,

	.stall_current_a_min = 2.0f,

	.max_tap_delta_deg = 1.0f,
	.seek_timeout_ms = 40000,
};

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
	uint32_t timer_get_ms_fails; // count of failures to get ms timestamp during feedback parsing
	uint32_t telemetry_scale_fails; // count of failures to scale a telemetry value
	uint32_t telemetry_can_tx_fails; // count of failures to transmit telemetry over CAN
	uint32_t sd_log_data_fails; // count of failures to log motor data to SD
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

	// Reinit the CAN module if a bus off state was detected
	FDCAN_ProtocolStatusTypeDef protocolStatus = {0};
	HAL_FDCAN_GetProtocolStatus(g_ak45_hfdcan, &protocolStatus);
	if (protocolStatus.BusOff) {
		CLEAR_BIT(g_ak45_hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
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

	ak45_health.timer_get_ms_fails++;
	return W_FAILURE;
}

/* used to stop the general can bus*/
static void ak45_stop_can(void) {
	// turn off fdcan so can restart
	if (HAL_FDCAN_Stop(g_ak45_hfdcan) != HAL_OK) {
		ak45_health.fdcan_stop_fails++;
	}
}

/************************************** TELEM **************************************/
/**
 * @brief Send temperature and current telemetry through CAN from the AK45 motor
 * @return W_SUCCESS on success, W_FAILURE on error
 */
static w_status_t ak45_driver_temp_curr_telemetry(void) {
	ak45_feedback_t fb = {0};
	if (W_SUCCESS != ak45_get_latest_feedback(&fb)) {
		return W_FAILURE;
	}

	uint32_t timestamp_ms = 0;
	if (W_SUCCESS != timer_get_ms(&timestamp_ms)) {
		return W_FAILURE;
	}

	w_status_t status = W_SUCCESS;

	int16_t temperature_scaled_int16 = 0;

	if (can_encode_scaled_int(SCALE_SERVO_TEMP, fb.temperature_c, &temperature_scaled_int16) !=
		W_SUCCESS) {
		ak45_health.telemetry_scale_fails++;
		status |= W_FAILURE;

	} else {
		can_msg_t msg = {0};
		build_analog_sensor_16bit_msg(PRIO_LOW,
									  (uint16_t)timestamp_ms,
									  SENSOR_CANARD_SERVO_TEMP,
									  (uint16_t)(temperature_scaled_int16 + TELEMETRY_INT16_OFFSET),
									  &msg);

		if (can_handler_transmit(&msg) != W_SUCCESS) {
			ak45_health.telemetry_can_tx_fails++;
			status |= W_FAILURE;
		}
	}

	// TODO: change to use automatic telem scaling once merged
	int16_t current_scaled_int16 = 0;
	if (can_encode_scaled_float(SCALE_SERVO_CURRENT, fb.current_a, &current_scaled_int16) !=
		W_SUCCESS) {
		ak45_health.telemetry_scale_fails++;
		status |= W_FAILURE;

	} else {
		can_msg_t msg = {0};
		build_analog_sensor_16bit_msg(PRIO_LOW,
									  (uint16_t)timestamp_ms,
									  SENSOR_CANARD_SERVO_CURR,
									  (uint16_t)(current_scaled_int16 + TELEMETRY_INT16_OFFSET),
									  &msg);

		if (can_handler_transmit(&msg) != W_SUCCESS) {
			ak45_health.telemetry_can_tx_fails++;
			status |= W_FAILURE;
		}
	}

	return status;
}

/**
 * @brief Send temperature angle through CAN from the AK45 motor
 * @return W_SUCCESS on success, W_FAILURE on error
 */
static w_status_t ak45_driver_angle_telemetry(void) {
	ak45_feedback_t fb = {0};
	w_status_t status = W_SUCCESS;
	if (W_SUCCESS != ak45_get_latest_feedback(&fb)) {
		return W_FAILURE;
	}

	uint32_t timestamp_ms = 0;
	if (W_SUCCESS != timer_get_ms(&timestamp_ms)) {
		return W_FAILURE;
	}

	int32_t scaled_angle_int32 = 0;
	if (can_encode_scaled_float(SCALE_SERVO_ANGLE, fb.position_deg, &scaled_angle_int32) !=
		W_SUCCESS) {
		ak45_health.telemetry_scale_fails++;
		return W_FAILURE;
	}

	can_msg_t msg = {0};
	build_analog_sensor_32bit_msg(PRIO_LOW,
								  (uint16_t)timestamp_ms,
								  SENSOR_CANARD_SERVO_ANGLE,
								  (uint32_t)(scaled_angle_int32 + TELEMETRY_INT32_OFFSET),
								  &msg);

	if (can_handler_transmit(&msg) != W_SUCCESS) {
		ak45_health.telemetry_can_tx_fails++;
		status |= W_FAILURE;
	}

	return status;
}

static w_status_t ak45_sd_telemetry(void) {
	ak45_feedback_t fb = {0};
	if (W_SUCCESS != ak45_get_latest_feedback(&fb)) {
		return W_FAILURE;
	}

	// set up the log data
	log_data_container_t log_container = {0};

	// motor
	log_container.servo_motor.motor_angle = fb.position_deg;
	log_container.servo_motor.motor_current = fb.current_a;
	log_container.servo_motor.motor_temperature = (float32_t)fb.temperature_c;

	if (log_data(LOG_WAIT_MS, LOG_TYPE_SERVO_MOTOR, &log_container) != W_SUCCESS) {
		ak45_health.sd_log_data_fails++;
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/************************************** TELEM **************************************/

w_status_t ak45_send_position_cmd(float32_t angle_deg) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_POS << 8) | AK45_DRIVER_ID;

	int32_t pos_raw = (int32_t)(angle_deg * AK45_POS_CMD_DEG_TO_POS);
	uint8_t data[4];
	data[0] = (uint8_t)(((uint32_t)pos_raw >> 24) & 0xFF);
	data[1] = (uint8_t)(((uint32_t)pos_raw >> 16) & 0xFF);
	data[2] = (uint8_t)(((uint32_t)pos_raw >> 8) & 0xFF);
	data[3] = (uint8_t)((uint32_t)pos_raw & 0xFF);

	return ak45_can_transmit_ext(ext_id, data, FDCAN_DLC_BYTES_4);
}

w_status_t ak45_send_current_cmd(int32_t current_mA) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | AK45_DRIVER_ID;

	uint8_t data[4];
	data[0] = (uint8_t)(((uint32_t)current_mA >> 24) & 0xFF);
	data[1] = (uint8_t)(((uint32_t)current_mA >> 16) & 0xFF);
	data[2] = (uint8_t)(((uint32_t)current_mA >> 8) & 0xFF);
	data[3] = (uint8_t)((uint32_t)current_mA & 0xFF);

	return ak45_can_transmit_ext(ext_id, data, FDCAN_DLC_BYTES_4);
}

w_status_t ak45_send_pos_velo_cmd(float32_t angle_deg, uint16_t mag_speed_rpm,
								  int16_t accel_rpm_s2) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_POS_SPD << 8) | AK45_DRIVER_ID;

	uint32_t pos_raw = ((int32_t)(angle_deg * AK45_POS_CMD_DEG_TO_POS));
	uint8_t data[8];
	data[0] = (uint8_t)((pos_raw >> 24) & 0xFF);
	data[1] = (uint8_t)((pos_raw >> 16) & 0xFF);
	data[2] = (uint8_t)((pos_raw >> 8) & 0xFF);
	data[3] = (uint8_t)(pos_raw & 0xFF);

	data[4] = (uint8_t)(((uint16_t)mag_speed_rpm >> 8) & 0xFF);
	data[5] = (uint8_t)((uint16_t)mag_speed_rpm & 0xFF);

	data[6] = (uint8_t)((accel_rpm_s2 >> 8) & 0xFF);
	data[7] = (uint8_t)(accel_rpm_s2 & 0xFF);

	return ak45_can_transmit_ext(ext_id, data, FDCAN_DLC_BYTES_8);
}

w_status_t ak45_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms) {
	// check if the driver has inited
	if (ak45_health.is_init) {
		ak45_health.reinit_attempts++;
		return W_FAILURE;
	}

	if (NULL == hfdcan) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Invalid pointers");
		ak45_health.invalid_args++;
		return W_INVALID_PARAM;
	}

	g_ak45_hfdcan = hfdcan;

	// feedback queue with length 1
	g_feedback_queue = xQueueCreate(1, sizeof(ak45_feedback_t));
	if (g_feedback_queue == NULL) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Lack of memory to create feedback queue");
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
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "FDCAN filter config failed");
		ak45_health.init_fdcan_filter_cfg_fails++;
		return W_FAILURE;
	}

	if (HAL_FDCAN_Start(g_ak45_hfdcan) != HAL_OK) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "FDCAN start failed");
		ak45_health.init_fdcan_start_fails++;
		ak45_stop_can();
		return W_FAILURE;
	}

	// to check if there has been a new can msg since start
	received_can_msg = false;

	if (HAL_FDCAN_ActivateNotification(g_ak45_hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "FDCAN activate notification failed");
		ak45_health.init_fdcan_notification_fails++;
		ak45_stop_can();
		return W_FAILURE;
	}
	// make sure we recieved a can msg before we send one
	uint32_t start_can_init_time_ms = 0;
	if (timer_get_ms(&start_can_init_time_ms) != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to get time");
		ak45_health.timer_get_ms_fails++;
		ak45_stop_can();
		return W_FAILURE;
	}

	uint32_t curr_time_ms = start_can_init_time_ms;

	// This will terminate either after recieving a CAN msg from the motor or worse case after the
	// described timeout
	while ((!received_can_msg) &&
		   ((curr_time_ms - start_can_init_time_ms) <= can_init_timeout_ms)) {
		vTaskDelay(pdMS_TO_TICKS(500));

		if (timer_get_ms(&curr_time_ms) != W_SUCCESS) {
			log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to get time");
			ak45_health.timer_get_ms_fails++;
			ak45_stop_can();
			return W_FAILURE;
		}
	}

	// return error if no response from motor
	if (!received_can_msg) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Unable to connect to AK45 CAN");
		ak45_health.init_fdcan_timeout++;
		ak45_stop_can();
		return W_FAILURE;
	}

	// set current position to 0
	if (ak45_send_set_origin() != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "failed to reset to 0");
		ak45_stop_can();
		return W_FAILURE;
	}

	// Register telemetry callbacks
	// No telem sent during SLEEP or ERROR phases
	static const telemetry_source_config_t telemetry_sources[] = {
		{"Motor Angle", ak45_driver_angle_telemetry, STATE_IDLE, 1000 / 5},
		{"Motor Angle", ak45_driver_angle_telemetry, STATE_PAD_FILTER, 1000 / 20},
		{"Motor Angle", ak45_driver_angle_telemetry, STATE_PAD_NAV, 1000 / 20},
		{"Motor Angle", ak45_driver_angle_telemetry, STATE_BOOST, 1000 / 10},
		{"Motor Angle", ak45_driver_angle_telemetry, STATE_ACT_ALLOWED, 1000 / 10},

		{"Motor Temp", ak45_driver_temp_curr_telemetry, STATE_IDLE, 1000 / 5},
		{"Motor Temp", ak45_driver_temp_curr_telemetry, STATE_PAD_FILTER, 1000 / 5},
		{"Motor Temp", ak45_driver_temp_curr_telemetry, STATE_PAD_NAV, 1000 / 5},
		{"Motor Temp", ak45_driver_temp_curr_telemetry, STATE_BOOST, 1000 / 2},
		{"Motor Temp", ak45_driver_temp_curr_telemetry, STATE_ACT_ALLOWED, 1000 / 2},

		{"Motor SD", ak45_sd_telemetry, STATE_IDLE, 1000 / 1},
		{"Motor SD", ak45_sd_telemetry, STATE_PAD_FILTER, 1000 / 10},
		{"Motor SD", ak45_sd_telemetry, STATE_PAD_NAV, 1000 / MAX_LOGGING_RATE_HZ},
		{"Motor SD", ak45_sd_telemetry, STATE_BOOST, 1000 / MAX_LOGGING_RATE_HZ},
		{"Motor SD", ak45_sd_telemetry, STATE_ACT_ALLOWED, 1000 / MAX_LOGGING_RATE_HZ},
		{"Motor SD", ak45_sd_telemetry, STATE_RECOVERY, 1000 / 10},
		{"Motor SD", ak45_sd_telemetry, STATE_SLEEPY, 1000 / 1},

	};

	static const size_t telemetry_source_count =
		sizeof(telemetry_sources) / sizeof(telemetry_source_config_t);
	w_status_t telemetry_register_status = W_SUCCESS;

	// Register callbacks and check
	for (size_t i = 0; i < telemetry_source_count; i++) {
		telemetry_register_status |= telemetry_register(&telemetry_sources[i]);
	}

	if (W_SUCCESS != telemetry_register_status) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to register telemetry sources.");
		ak45_stop_can();
		return W_FAILURE;
	}

	ak45_health.tx_errors = 0;
	ak45_health.is_init = true;

	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "ak45", "Init successful");
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

w_status_t ak45_send_set_origin(void) {
	uint32_t ext_id = ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8) | AK45_DRIVER_ID;
	uint8_t zero_data[1] = {0};
	return ak45_can_transmit_ext(ext_id, zero_data, FDCAN_DLC_BYTES_1);
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
		ak45_health.rx_errors++;
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
	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_AK45,
							  .error_bitfield = 0};

	if (!ak45_health.hard_stop_calibrated) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_NOT_CALIBRATED_OFFSET);
	}

	if (ak45_health.hard_stop_cal_failed) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_FAILED_CALIBRATION_OFFSET);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed calibration.");
	}

	if (ak45_health.cmd_tx_failed) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_TX_FAILURE_OFFSET);
	}

	if (ak45_health.feedback_rx_failed) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_RX_FAILURE_OFFSET);
	}

	if (!ak45_health.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_NOT_INIT_OFFSET);
	}

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "is_init=%d, hard_stop_calibd=%d, hardstop_cal_fail=%d, rx_err=%lu, "
			 "tx_err=%lu",
			 ak45_health.is_init,
			 ak45_health.hard_stop_calibrated,
			 ak45_health.hard_stop_cal_failed,
			 ak45_health.rx_errors,
			 ak45_health.tx_errors);

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "OOM=%lu, not_init=%lu, fb_q_empty=%lu, "
			 "reinit_tries=%lu, invalid_arg=%lu, get_ms_fails=%lu",
			 ak45_health.out_of_memory,
			 ak45_health.not_initialized,
			 ak45_health.feedback_queue_empty,
			 ak45_health.reinit_attempts,
			 ak45_health.invalid_args,
			 ak45_health.timer_get_ms_fails);

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "timeout=%lu notif_fail=%lu start_fail=%lu cfg_fail=%lu "
			 "stop_fail=%lu",
			 ak45_health.init_fdcan_timeout,
			 ak45_health.init_fdcan_notification_fails,
			 ak45_health.init_fdcan_start_fails,
			 ak45_health.init_fdcan_filter_cfg_fails,
			 ak45_health.fdcan_stop_fails);

	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45",
			 "telem_scale_fail=%" PRIu32 " telem_can_tx_fail=%" PRIu32 ", log_data_fail=%" PRIu32,
			 ak45_health.telemetry_scale_fails,
			 ak45_health.telemetry_can_tx_fails,
			 ak45_health.sd_log_data_fails);

	ak45_health.cmd_tx_failed = false;
	ak45_health.feedback_rx_failed = false;

	return status;
}

static w_status_t ak45_detect_hard_stop(const ak45_calibration_config_t *config,
										bool check_pos_side, float32_t *hard_stop_angle_deg) {
	if ((NULL == config) || (NULL == hard_stop_angle_deg)) {
		ak45_health.invalid_args++;
		return W_FAILURE;
	}

	float32_t increment_deg = check_pos_side ? 0.1 : -0.1;

	uint32_t start_time_ms = 0;
	if (timer_get_ms(&start_time_ms) != W_SUCCESS) {
		ak45_health.timer_get_ms_fails++;
		return W_FAILURE;
	}

	uint32_t curr_time_ms = start_time_ms;

	ak45_feedback_t fb = {0};

	if (ak45_get_latest_feedback(&fb) != W_SUCCESS) {
		// bring back to zero
		ak45_send_position_cmd(0); // no success checks since this is an emergency cmd
		return W_FAILURE;
	}

	float32_t cur_angle_deg = fb.position_deg;

	while ((start_time_ms + config->seek_timeout_ms) >= curr_time_ms) {
		if (ak45_get_latest_feedback(&fb) != W_SUCCESS) {
			// bring back to zero
			ak45_send_position_cmd(0); // no success checks since this is an emergency cmd
			return W_FAILURE;
		}
		if ((fb.current_a) > (config->stall_current_a_min)) {
			break;
		}
		// ak45_send_pos_velo_cmd(max_angle, config->cal_speed_rpm, config->cal_accel_rpm_s2);
		ak45_send_position_cmd(cur_angle_deg);
		cur_angle_deg += increment_deg;
		if (fabsf(cur_angle_deg) > config->seek_target_deg) {
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(AK45_UPDATE_PERIOD_MS));

		if (timer_get_ms(&curr_time_ms) != W_SUCCESS) {
			ak45_health.timer_get_ms_fails++;
			ak45_send_position_cmd(0); // no success checks since this is an emergency cmd
			return W_FAILURE;
		}
	}

	// check if timed out
	if (((start_time_ms + config->seek_timeout_ms) < curr_time_ms) ||
		(fabsf(cur_angle_deg) > config->seek_target_deg)) {
		ak45_send_position_cmd(0); // no success checks since this is an emergency cmd
		return W_FAILURE;
	}

	*hard_stop_angle_deg = fb.position_deg;

	return W_SUCCESS;
}

// TODO: test version which 5 degrees on both side with
w_status_t ak45_hard_stop_calibrate(const ak45_calibration_config_t *config) {
	if (NULL == config) {
		ak45_health.invalid_args++;
		return W_FAILURE;
	}

	// reset to zero
	if (ak45_send_position_cmd(0) != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to return to zero.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// positive
	// tap 1
	float32_t pos_tap_1_deg = 0;
	if (ak45_detect_hard_stop(config, true, &pos_tap_1_deg) != W_SUCCESS) {
		// reset to zero and
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed positive tap 1 calibration.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// bring our self back a bit
	if (ak45_send_position_cmd(pos_tap_1_deg - config->backoff_deg) != W_SUCCESS) {
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to back off.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(config->settle_ms));

	// tap 2
	float32_t pos_tap_2_deg = 0;
	if (ak45_detect_hard_stop(config, true, &pos_tap_2_deg) != W_SUCCESS) {
		// reset to zero and
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed positive tap 2 calibration.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// make sure with range and set positive side
	if (fabsf(pos_tap_1_deg - pos_tap_2_deg) > config->max_tap_delta_deg) {
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed positive calibration.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	float32_t pos_hardstops_deg = (pos_tap_1_deg + pos_tap_2_deg) / 2;

	// find negative hardstops
	// reset to zero
	if (ak45_send_position_cmd(0) != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to return to zero.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(config->settle_ms));

	// negative
	// tap 1
	float32_t neg_tap_1_deg = 0;
	if (ak45_detect_hard_stop(config, false, &neg_tap_1_deg) != W_SUCCESS) {
		// reset to zero
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed negative tap 1 calibration.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// bring our self back a bit
	if (ak45_send_position_cmd(neg_tap_1_deg + config->backoff_deg) != W_SUCCESS) {
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to back off.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(config->settle_ms));

	// tap 2
	float32_t neg_tap_2_deg = 0;
	if (ak45_detect_hard_stop(config, false, &neg_tap_2_deg) != W_SUCCESS) {
		// reset to zero and
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed negative tap 2 calibration.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// make sure with range and set negative side
	if (fabsf(neg_tap_1_deg - neg_tap_2_deg) > config->max_tap_delta_deg) {
		ak45_send_position_cmd(0);
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed negative calibration.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// reset to zero
	if (ak45_send_position_cmd(0) != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to return to zero.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(config->settle_ms));

	// go to new zero
	float32_t neg_hardstops_deg = (neg_tap_1_deg + neg_tap_2_deg) / 2;

	if (ak45_send_position_cmd((pos_hardstops_deg + neg_hardstops_deg) / 2) != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to go to new zero.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	vTaskDelay(pdMS_TO_TICKS(config->settle_ms));

	if (ak45_send_set_origin() != W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45", "Failed to set new zero.");
		ak45_health.hard_stop_calibrated = false;
		ak45_health.hard_stop_cal_failed = true;
		return W_FAILURE;
	}

	// set to calibrated
	ak45_health.hard_stop_calibrated = true;
	ak45_health.hard_stop_cal_failed = false;
	return W_SUCCESS;
}
