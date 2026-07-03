#include "application/hil/hil.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "usbd_cdc_if.h"

#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"

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
	float32_t board_accel_x;
	float32_t board_accel_y;
	float32_t board_accel_z;

	float32_t board_gyro_x;
	float32_t board_gyro_y;
	float32_t board_gyro_z;
    
	float32_t board_mag_x;
	float32_t board_mag_y;
	float32_t board_mag_z;

	float32_t board_baro;
    
	float32_t mti_accel_x;
	float32_t mti_accel_y;
	float32_t mti_accel_z;

	float32_t mti_gyro_x;
	float32_t mti_gyro_y;
	float32_t mti_gyro_z;
    
	float32_t mti_mag_x;
	float32_t mti_mag_y;
	float32_t mti_mag_z;
    
    float32_t mti_baro;

	float32_t ad_accel_x;
	float32_t ad_accel_y;
	float32_t ad_accel_z;

	float32_t ad_gyro;
    // simulink is stuck giving xyz for gyro even though only the x-axis actually exists
	float32_t ad_gyro_dummy_y;
	float32_t ad_gyro_dummy_z;

	float32_t motor_encoder;

	bool ignitor_can_msg;
	bool inj_valve_can_msg;

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
		taskENTER_CRITICAL();
		memcpy(hil_ctx.ready_rx_packet, hil_ctx.wip_rx_packet, HIL_RX_BODY_SIZE_BYTES);
		hil_ctx.is_reading_avail = true;
		hil_ctx.parsed_packet_count++;
		taskEXIT_CRITICAL();

		// reset parser state to wait for next packet
		hil_ctx.parse_state = HIL_PARSE_WAIT_FOR_HEADER;
		hil_ctx.header_match_idx = 0;
		hil_ctx.wip_rx_body_idx = 0;

		// check if any fake CAN msgs triggered. If so, send the flight phase event directly.
		// fake CAN bypasses CAN handler intentionally cuz that's tested in real CAN tests later
		hil_rx_payload_t *ready_packet = (hil_rx_payload_t *)&hil_ctx.ready_rx_packet[0];
		if (ready_packet->ignitor_can_msg) {
			flight_phase_send_event(EVENT_IGNITOR);
			log_text(1, LOG_LVL_INFO, "hil", "ignitor can msg");
		}
		if (ready_packet->inj_valve_can_msg) {
			flight_phase_send_event(EVENT_INJ_OPEN);
			log_text(1, LOG_LVL_INFO, "hil", "inj valve can msg");
		}
	}
}

w_status_t hil_get_latest_sensor_data(all_sensors_data_t *out) {
	if (NULL == out) {
		return W_INVALID_PARAM;
	}

	// store local copy of the latest payload to have short critical section
	uint8_t ready_packet_bytes[sizeof(hil_rx_payload_t)] = {0};
	hil_rx_payload_t *ready_packet = (hil_rx_payload_t *)&ready_packet_bytes[0];

	// return immediately if no completed packet is ready, dont wait
	taskENTER_CRITICAL();
	if (!hil_ctx.is_reading_avail) {
		log_text(1, LOG_LVL_WARN, "hil", "no ready packet avail");
		taskEXIT_CRITICAL();
		return W_FAILURE;
	}
	memcpy(ready_packet_bytes, hil_ctx.ready_rx_packet, sizeof(ready_packet_bytes));
	taskEXIT_CRITICAL();

	out->board_meas.board_imu.accel.x = ready_packet->board_accel_x;
	out->board_meas.board_imu.accel.y = ready_packet->board_accel_y;
	out->board_meas.board_imu.accel.z = ready_packet->board_accel_z;
	out->board_meas.board_imu.gyro.x = ready_packet->board_gyro_x;
	out->board_meas.board_imu.gyro.y = ready_packet->board_gyro_y;
	out->board_meas.board_imu.gyro.z = ready_packet->board_gyro_z;
	out->board_meas.board_imu.is_new = true;

	out->board_meas.board_baro.meas = ready_packet->board_baro;
	out->board_meas.board_baro.is_new = true;

	out->board_meas.board_mag.meas.x = ready_packet->board_mag_x;
	out->board_meas.board_mag.meas.y = ready_packet->board_mag_y;
	out->board_meas.board_mag.meas.z = ready_packet->board_mag_z;
	out->board_meas.board_mag.is_new = true;

	out->mti_meas.mti_accel.meas.x = ready_packet->mti_accel_x;
	out->mti_meas.mti_accel.meas.y = ready_packet->mti_accel_y;
	out->mti_meas.mti_accel.meas.z = ready_packet->mti_accel_z;
	out->mti_meas.mti_accel.is_new = true;

	out->mti_meas.mti_gyro.meas.x = ready_packet->mti_gyro_x;
	out->mti_meas.mti_gyro.meas.y = ready_packet->mti_gyro_y;
	out->mti_meas.mti_gyro.meas.z = ready_packet->mti_gyro_z;
	out->mti_meas.mti_gyro.is_new = true;

	out->mti_meas.mti_baro.meas = ready_packet->mti_baro;
	out->mti_meas.mti_baro.is_new = true;

	out->mti_meas.mti_mag.meas.x = ready_packet->mti_mag_x;
	out->mti_meas.mti_mag.meas.y = ready_packet->mti_mag_y;
	out->mti_meas.mti_mag.meas.z = ready_packet->mti_mag_z;
	out->mti_meas.mti_mag.is_new = true;

	out->ad_meas.ad_accel.meas.x = ready_packet->ad_accel_x;
	out->ad_meas.ad_accel.meas.y = ready_packet->ad_accel_y;
	out->ad_meas.ad_accel.meas.z = ready_packet->ad_accel_z;
	out->ad_meas.ad_accel.is_new = true;

	out->ad_meas.ad_gyro.meas = ready_packet->ad_gyro;
	out->ad_meas.ad_gyro.is_new = true;

	out->motor_encoder_meas.meas = ready_packet->motor_encoder;
	out->motor_encoder_meas.is_new = true;

	log_text(1, LOG_LVL_INFO, "hil", "rx footer: %c", ready_packet->footer);
	return W_SUCCESS;
}

w_status_t hil_send_simulink_cmd(float64_t canard_cmd_rad) {
	hil_tx_packet_t tx_packet = {0};
	tx_packet.header[0] = HIL_HEADER_CHAR_0;
	tx_packet.header[1] = HIL_HEADER_CHAR_1;
	tx_packet.header[2] = HIL_HEADER_CHAR_2;
	tx_packet.header[3] = HIL_HEADER_CHAR_3;
	tx_packet.payload.canard_cmd = canard_cmd_rad;
	// TODO: stream more telem to simulink
	tx_packet.payload.footer[0] = HIL_FOOTER_CHAR;

	uint8_t usb_status = CDC_Transmit_HS((uint8_t *)&tx_packet, (uint16_t)sizeof(tx_packet));

	if (USBD_OK != usb_status) {
		log_text(1, LOG_LVL_WARN, "hil", "CDC_Transmit_HS fail. usb_status: %d", usb_status);
		return W_FAILURE;
	} else {
		log_text(1, LOG_LVL_INFO, "hil", "sent tx usb %lf", tx_packet.payload.canard_cmd);
	}

	return W_SUCCESS;
}
