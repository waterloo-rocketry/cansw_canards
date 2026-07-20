#include "application/hil/hil.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "usbd_cdc_if.h"

#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "common/math/math.h"

extern void unblock_fsm_hil();

// Packet framing bytes
#define HIL_HEADER_SIZE_BYTES (4U)
#define HIL_FOOTER_SIZE_BYTES (1U)

#define HIL_HEADER_CHAR_0 ('o')
#define HIL_HEADER_CHAR_1 ('r')
#define HIL_HEADER_CHAR_2 ('z')
#define HIL_HEADER_CHAR_3 ('!')

#define HIL_FOOTER_CHAR ('\n')

// state of hil parser - either waiting for new packet or current building a packet
typedef enum {
	HIL_PARSE_WAIT_FOR_HEADER = 0,
	HIL_PARSE_BUILDING_PACKET
} hil_parse_state_t;

// payload from simulink + footer (excluding header)
typedef struct __attribute__((packed)) {
	float32_t motor_encoder;

	float32_t board_accel_x;
	float32_t board_accel_y;
	float32_t board_accel_z;
	float32_t board_accel_status;

	float32_t board_gyro_x;
	float32_t board_gyro_y;
	float32_t board_gyro_z;
	float32_t board_gyro_status;

	float32_t board_mag_x;
	float32_t board_mag_y;
	float32_t board_mag_z;
	float32_t board_mag_status;

	float32_t board_baro;
	float32_t board_baro_status;

	float32_t mti_accel_x;
	float32_t mti_accel_y;
	float32_t mti_accel_z;
	float32_t mti_accel_status;

	float32_t mti_gyro_x;
	float32_t mti_gyro_y;
	float32_t mti_gyro_z;
	float32_t mti_gyro_status;

	float32_t mti_mag_x;
	float32_t mti_mag_y;
	float32_t mti_mag_z;
	float32_t mti_mag_status;

	float32_t mti_baro;
	float32_t mti_baro_status;

	float32_t ad_accel_x;
	float32_t ad_accel_y;
	float32_t ad_accel_z;
	float32_t ad_accel_status;

	float32_t ad_gyro_dummy_x;
	// simulink is stuck giving xyz for gyro even though only the x-axis actually exists
	float32_t ad_gyro_dummy_y;
	float32_t ad_gyro;
	float32_t ad_gyro_status;

	float32_t can_msg; //
	// bool pad_filter_can_msg;
	// bool ignitor_can_msg;
	// bool inj_valve_can_msg;

	uint8_t footer;
} hil_rx_payload_t;

// Full rx packet with header
typedef struct __attribute__((packed)) {
	uint8_t header[HIL_HEADER_SIZE_BYTES];
	hil_rx_payload_t payload;
} hil_rx_packet_t;

// Payload + footer to transmit
typedef struct __attribute__((packed)) {
	float64_t canard_cmd;
	float64_t att_w;
	float64_t att_x;
	float64_t att_y;
	float64_t att_z;
	float64_t rates_x;
	float64_t rates_y;
	float64_t rates_z;
	float64_t vel_x;
	float64_t vel_y;
	float64_t vel_z;
	float64_t altitude;
	float64_t cl;
	float64_t delta;
	// float64_t cov_norm;
	// float64_t where_it_is[2]; // {roll angle (rad), roll rate (rad/s)}
	// float64_t where_it_isnt[2]; // {roll angle (rad), roll rate (rad/s)}
	uint8_t footer[HIL_FOOTER_SIZE_BYTES];
} hil_tx_payload_t;

// Full tx packet with header
typedef struct __attribute__((packed)) {
	uint8_t header[HIL_HEADER_SIZE_BYTES];
	hil_tx_payload_t payload;
} hil_tx_packet_t;

// Size of packet received from simulink, excluding header but including footer
#define HIL_RX_BODY_SIZE_BYTES ((uint32_t)sizeof(hil_rx_payload_t))
// Size of packet sent to simulink, including header and footer
#define HIL_TX_PACKET_SIZE_BYTES ((uint32_t)sizeof(hil_tx_packet_t))

typedef struct {
	// true if a simulink packet has been parsed and is available to read
	volatile bool is_reading_avail;

	hil_parse_state_t parse_state;
	uint32_t header_match_idx; // number of consecutive matched header bytes
	uint32_t wip_rx_body_idx; // parser index into wip_rx_packet buffer, ignoring header

	uint8_t wip_rx_packet[HIL_RX_BODY_SIZE_BYTES];
	uint8_t ready_rx_packet[HIL_RX_BODY_SIZE_BYTES];

	// health stuff
	volatile uint32_t parse_overflow_count;
	volatile uint32_t parsed_packet_count;
} hil_ctx_t;

// static module context
static hil_ctx_t hil_ctx;

// HIL timestamp
volatile uint32_t hil_timestamp_tenth_ms = 0;

w_status_t hil_init(void) {
	// parser starts idle with no pending packet
	hil_ctx.is_reading_avail = false;
	hil_ctx.parse_state = HIL_PARSE_WAIT_FOR_HEADER;
	hil_ctx.header_match_idx = 0;
	hil_ctx.wip_rx_body_idx = 0;

	hil_ctx.parse_overflow_count = 0;
	hil_ctx.parsed_packet_count = 0;

	memset(hil_ctx.wip_rx_packet, 0, sizeof(hil_ctx.wip_rx_packet));
	memset(hil_ctx.ready_rx_packet, 0, sizeof(hil_ctx.ready_rx_packet));

	return W_SUCCESS;
}

void hil_parse_rx_bytes(uint8_t byte) {
	static const uint8_t header[HIL_HEADER_SIZE_BYTES] = {
		HIL_HEADER_CHAR_0,
		HIL_HEADER_CHAR_1,
		HIL_HEADER_CHAR_2,
		HIL_HEADER_CHAR_3,
	};

	if (HIL_PARSE_WAIT_FOR_HEADER == hil_ctx.parse_state) {
		if (byte == header[hil_ctx.header_match_idx]) {
			hil_ctx.header_match_idx++;
			if (hil_ctx.header_match_idx == HIL_HEADER_SIZE_BYTES) {
				hil_ctx.parse_state = HIL_PARSE_BUILDING_PACKET;
				hil_ctx.wip_rx_body_idx = 0;
				hil_ctx.header_match_idx = 0;
			}
		} else {
			hil_ctx.header_match_idx = 0;
		}
		return;
	}

	hil_ctx.wip_rx_packet[hil_ctx.wip_rx_body_idx++] = byte;

	// all bytes after header have been reeived, so the packet must be done
	if (hil_ctx.wip_rx_body_idx >= HIL_RX_BODY_SIZE_BYTES) {
		// publish completed simulink reading
		memcpy(hil_ctx.ready_rx_packet, hil_ctx.wip_rx_packet, HIL_RX_BODY_SIZE_BYTES);
		hil_ctx.is_reading_avail = true;
		hil_ctx.parsed_packet_count++;

		// reset parser state to wait for next packet
		hil_ctx.parse_state = HIL_PARSE_WAIT_FOR_HEADER;
		hil_ctx.header_match_idx = 0;
		hil_ctx.wip_rx_body_idx = 0;

		// check if any fake CAN msgs triggered. If so, send the flight phase event directly.
		// fake CAN bypasses CAN handler intentionally cuz that's tested in real CAN tests later
		hil_rx_payload_t *ready_packet = (hil_rx_payload_t *)&hil_ctx.ready_rx_packet[0];

		if (float_equal(ready_packet->can_msg, 3.0)) {
			flight_phase_send_event(EVENT_PAD_FILTER);
		}
		if (float_equal(ready_packet->can_msg, 2.0)) {
			flight_phase_send_event(EVENT_IGNITOR);
		}
		if (float_equal(ready_packet->can_msg, 1.0)) {
			flight_phase_send_event(EVENT_INJ_OPEN);
		}

		// also increment timestamp as we use HIL as the timebase for timer_get_ms()
		hil_timestamp_tenth_ms += 25; // hil gives packets at 400 hz (2.5 ms per packet)

		// also unblock fsm (since tim5 is not used as the timebase in HIL)
		unblock_fsm_hil();
	}
}

w_status_t hil_wait_for_simulink_data(all_sensors_data_t *out) {
	if (NULL == out) {
		return W_INVALID_PARAM;
	}

	// store local copy of the latest payload to have short critical section
	uint8_t ready_packet_bytes[sizeof(hil_rx_payload_t)] = {0};
	hil_rx_payload_t *ready_packet = (hil_rx_payload_t *)&ready_packet_bytes[0];

	// // return immediately if no completed packet is ready, dont wait
	// if (!hil_ctx.is_reading_avail) {
	// 	log_text(1, LOG_LVL_WARN, "hil", "no ready packet avail");
	// 	return W_FAILURE;
	// }

	// blocking wait for data
	uint32_t timeout_ms = 0; // 30 second timeout
	uint32_t start_time = xTaskGetTickCount();
	while (!hil_ctx.is_reading_avail &&
		   (xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms)) {
		vTaskDelay(pdMS_TO_TICKS(1)); // wait 1 ms before checking again
	}
	// critical section
	taskENTER_CRITICAL();
	memcpy(ready_packet_bytes, hil_ctx.ready_rx_packet, sizeof(ready_packet_bytes));
	taskEXIT_CRITICAL();

	out->board_meas.board_imu.accel.x = ready_packet->board_accel_x;
	out->board_meas.board_imu.accel.y = ready_packet->board_accel_y;
	out->board_meas.board_imu.accel.z = ready_packet->board_accel_z;
	out->board_meas.board_imu.gyro.x = ready_packet->board_gyro_x;
	out->board_meas.board_imu.gyro.y = ready_packet->board_gyro_y;
	out->board_meas.board_imu.gyro.z = ready_packet->board_gyro_z;
	out->board_meas.board_imu.is_new = ready_packet->board_accel_status;

	out->board_meas.board_baro.meas = ready_packet->board_baro;
	out->board_meas.board_baro.is_new = ready_packet->board_baro_status;

	out->board_meas.board_mag.meas.x = ready_packet->board_mag_x;
	out->board_meas.board_mag.meas.y = ready_packet->board_mag_y;
	out->board_meas.board_mag.meas.z = ready_packet->board_mag_z;
	out->board_meas.board_mag.is_new = ready_packet->board_mag_status;

	out->mti_meas.mti_accel.meas.x = ready_packet->mti_accel_x;
	out->mti_meas.mti_accel.meas.y = ready_packet->mti_accel_y;
	out->mti_meas.mti_accel.meas.z = ready_packet->mti_accel_z;
	out->mti_meas.mti_accel.is_new = ready_packet->mti_accel_status;

	out->mti_meas.mti_gyro.meas.x = ready_packet->mti_gyro_x;
	out->mti_meas.mti_gyro.meas.y = ready_packet->mti_gyro_y;
	out->mti_meas.mti_gyro.meas.z = ready_packet->mti_gyro_z;
	out->mti_meas.mti_gyro.is_new = ready_packet->mti_gyro_status;

	out->mti_meas.mti_baro.meas = ready_packet->mti_baro;
	out->mti_meas.mti_baro.is_new = ready_packet->mti_baro_status;

	out->mti_meas.mti_mag.meas.x = ready_packet->mti_mag_x;
	out->mti_meas.mti_mag.meas.y = ready_packet->mti_mag_y;
	out->mti_meas.mti_mag.meas.z = ready_packet->mti_mag_z;
	out->mti_meas.mti_mag.is_new = ready_packet->mti_mag_status;

	out->ad_meas.ad_accel.meas.x = ready_packet->ad_accel_x;
	out->ad_meas.ad_accel.meas.y = ready_packet->ad_accel_y;
	out->ad_meas.ad_accel.meas.z = ready_packet->ad_accel_z;
	out->ad_meas.ad_accel.is_new = ready_packet->ad_accel_status;

	out->ad_meas.ad_gyro.meas = ready_packet->ad_gyro;
	out->ad_meas.ad_gyro.is_new = false;

	out->motor_encoder_meas.meas = ready_packet->motor_encoder;
	out->motor_encoder_meas.is_new = true;

	log_text(1,
			 LOG_LVL_INFO,
			 "hil rx imu",
			 "acc %f, %f, %f gyro %f, %f, %f mag %f, %f, %f baro %f",
			 out->board_meas.board_imu.accel.x,
			 out->board_meas.board_imu.accel.y,
			 out->board_meas.board_imu.accel.z,
			 out->board_meas.board_imu.gyro.x,
			 out->board_meas.board_imu.gyro.y,
			 out->board_meas.board_imu.gyro.z,
			 out->board_meas.board_mag.meas.x,
			 out->board_meas.board_mag.meas.y,
			 out->board_meas.board_mag.meas.z,
			 out->board_meas.board_baro.meas);
	log_text(1,
			 LOG_LVL_INFO,
			 "hil rx mti",
			 "acc %f, %f, %f gyro %f, %f, %f mag %f, %f, %f baro %f",
			 out->mti_meas.mti_accel.meas.x,
			 out->mti_meas.mti_accel.meas.y,
			 out->mti_meas.mti_accel.meas.z,
			 out->mti_meas.mti_gyro.meas.x,
			 out->mti_meas.mti_gyro.meas.y,
			 out->mti_meas.mti_gyro.meas.z,
			 out->mti_meas.mti_mag.meas.x,
			 out->mti_meas.mti_mag.meas.y,
			 out->mti_meas.mti_mag.meas.z,
			 out->mti_meas.mti_baro.meas);
	log_text(1,
			 LOG_LVL_INFO,
			 "hil rx ad",
			 "acc %f, %f, %f gyro %f",
			 out->ad_meas.ad_accel.meas.x,
			 out->ad_meas.ad_accel.meas.y,
			 out->ad_meas.ad_accel.meas.z,
			 out->ad_meas.ad_gyro.meas);
	log_text(1, LOG_LVL_INFO, "hil rx encoder", "meas %f", out->motor_encoder_meas.meas);

	log_text(1, LOG_LVL_INFO, "hil", "rx footer: %c", ready_packet->footer);
	log_text(1, LOG_LVL_INFO, "hil", "rx canmsg: %f", ready_packet->can_msg);

	// reading is now consumed, so mark it as no longer available
	hil_ctx.is_reading_avail = false;

	return W_SUCCESS;
}

static hil_tx_packet_t tx_packet = {0};

w_status_t hil_send_simulink_cmd(navigator_input_t *p_nav_in, navigator_output_t *p_nav_out,
								 gnc_x_state_t *p_x_state, gnc_controller_ctx_t *p_controller_ctx,
								 controller_input_t *p_cntl_in, controller_output_t *p_cntl_out) {
	log_text(
		1, LOG_LVL_DEBUG, "HIL", "start send tx usb %lf", p_cntl_out->canard_command_angle_rad);
	tx_packet.header[0] = HIL_HEADER_CHAR_0;
	tx_packet.header[1] = HIL_HEADER_CHAR_1;
	tx_packet.header[2] = HIL_HEADER_CHAR_2;
	tx_packet.header[3] = HIL_HEADER_CHAR_3;

	tx_packet.payload.canard_cmd = p_cntl_out->canard_command_angle_rad;

	tx_packet.payload.att_w = p_x_state->q.w;
	tx_packet.payload.att_x = p_x_state->q.x;
	tx_packet.payload.att_y = p_x_state->q.y;
	tx_packet.payload.att_z = p_x_state->q.z;
	tx_packet.payload.rates_x = p_x_state->ang_rate.x;
	tx_packet.payload.rates_y = p_x_state->ang_rate.y;
	tx_packet.payload.rates_z = p_x_state->ang_rate.z;
	tx_packet.payload.vel_x = p_x_state->vel.x;
	tx_packet.payload.vel_y = p_x_state->vel.y;
	tx_packet.payload.vel_z = p_x_state->vel.z;
	tx_packet.payload.altitude = p_x_state->altitude;

	tx_packet.payload.cl = p_controller_ctx->coeffs[0];
	// tx_packet.payload.cov_norm = p_nav_out->cov_norm;
	// tx_packet.payload.where_it_is[0] = p_nav_out->roll_state[0];
	// tx_packet.payload.where_it_is[1] = p_nav_out->roll_state[1];
	// tx_packet.payload.where_it_isnt[0] = p_cntl_in->xR[0];
	// tx_packet.payload.where_it_isnt[1] = p_cntl_in->xR[1];

	tx_packet.payload.footer[0] = HIL_FOOTER_CHAR;

	uint8_t usb_status = CDC_Transmit_HS((uint8_t *)&tx_packet, (uint16_t)sizeof(tx_packet));

	if (USBD_OK != usb_status) {
		log_text(1, LOG_LVL_WARN, "hil", "CDC_Transmit_HS fail. usb_status: %d", usb_status);
		return W_FAILURE;
	} else {
		log_text(1, LOG_LVL_INFO, "hil", "sent tx usb %lf", tx_packet.payload.canard_cmd);
	}

	log_text(1,
			 LOG_LVL_DEBUG,
			 "HIL",
			 "att %f, %f, %f, %f rates %f, %f, %f vel %f, %f, %f alt %f",
			 tx_packet.payload.att_w,
			 tx_packet.payload.att_x,
			 tx_packet.payload.att_y,
			 tx_packet.payload.att_z,
			 tx_packet.payload.rates_x,
			 tx_packet.payload.rates_y,
			 tx_packet.payload.rates_z,
			 tx_packet.payload.vel_x,
			 tx_packet.payload.vel_y,
			 tx_packet.payload.vel_z,
			 tx_packet.payload.altitude);

	// vTaskDelay(10);
	// log_text(1, LOG_LVL_DEBUG, "HIL", "cov_norm %f where_it_is %f, %f where_it_isnt %f, %f",
	// tx_packet.payload.cov_norm, tx_packet.payload.where_it_is[0],
	// tx_packet.payload.where_it_is[1], tx_packet.payload.where_it_isnt[0],
	// tx_packet.payload.where_it_isnt[1]);

	return W_SUCCESS;
}
