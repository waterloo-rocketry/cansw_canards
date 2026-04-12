#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"

#include "fs.h"
#include "log.h"

static size_t active_write_buffer = 0;
static size_t active_read_buffer = 0;
static uint8_t sequence_number = 0;

volatile struct {
	size_t pointer;
	uint8_t buffer[PAGE_SIZE];
	bool ready_to_write;
} log_buffer[NUM_BUFFERS];

w_status_t temp_log_init(void) {
	for (int i = 0; i < NUM_BUFFERS; i++) {
		log_buffer[i].pointer = 0;
		log_buffer[i].ready_to_write = false;
	}

	return W_SUCCESS;
}

void log_handle_incoming_message(const can_msg_t *msg, uint32_t timestamp) {
	size_t pointer = log_buffer[active_write_buffer].pointer;

	if ((PAGE_SIZE - pointer) < (size_t)(9 + msg->data_len)) { // Not enough space in buffer
		// Fill remaining space with 0xff
		for (; pointer < PAGE_SIZE; pointer++) {
			log_buffer[active_write_buffer].buffer[pointer] = 0xff;
		}
		// Reset pointer to 0
		log_buffer[active_write_buffer].pointer = 0;
		// Label buffer ready for write
		log_buffer[active_write_buffer].ready_to_write = true;
		// Increment active buffer
		active_write_buffer =
			(active_write_buffer >= (NUM_BUFFERS - 1)) ? 0 : (active_write_buffer + 1);
		// Increment sequence number
		++sequence_number;
	}

	pointer = log_buffer[active_write_buffer].pointer;

	// Pointer should not point in page signature
	w_assert((pointer == 0) || (pointer >= SIGNATURE_SIZE));

	if (pointer < SIGNATURE_SIZE) {
		// Write page signature
		log_buffer[active_write_buffer].buffer[0] = 'L';
		log_buffer[active_write_buffer].buffer[1] = 'O';
		log_buffer[active_write_buffer].buffer[2] = 'G';
		log_buffer[active_write_buffer].buffer[3] = sequence_number & 0xff;
		log_buffer[active_write_buffer].pointer = SIGNATURE_SIZE;
	}

	pointer = log_buffer[active_write_buffer].pointer;

	// Write serialized CAN message
	log_buffer[active_write_buffer].buffer[pointer + 0] = msg->sid & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 1] = (msg->sid >> 8) & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 2] = (msg->sid >> 16) & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 3] = (msg->sid >> 24) & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 4] = timestamp & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 5] = (timestamp >> 8) & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 6] = (timestamp >> 16) & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 7] = (timestamp >> 24) & 0xff;
	log_buffer[active_write_buffer].buffer[pointer + 8] = msg->data_len;
	memcpy((uint8_t *)log_buffer[active_write_buffer].buffer + pointer + 9, msg->data, msg->data_len);
	log_buffer[active_write_buffer].pointer += 9 + msg->data_len;
}

bool red_led_on = false;

void log_heartbeat(void) {
	if (log_buffer[active_read_buffer].ready_to_write) {
		fs_write_page((const uint8_t *)log_buffer[active_read_buffer].buffer);
		log_buffer[active_read_buffer].ready_to_write = 0;
		active_read_buffer =
			(active_read_buffer >= (NUM_BUFFERS - 1)) ? 0 : (active_read_buffer + 1);
		if (red_led_on) {
			red_led_on = false;
		} else {
			red_led_on = true;
		}
	}
}