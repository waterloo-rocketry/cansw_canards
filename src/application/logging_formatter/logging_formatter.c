#include "application/logging_formatter/logging_formatter.h"
#include "application/logger/log.h"

typedef struct {
	bool is_init;
	bool invalid_param;
	bool timed_out;
	uint32_t read_count;
	uint32_t write_count;
	uint32_t read_error;
	uint32_t write_error;
	uint32_t not_writable;
	uint32_t error_state;
} logging_formatter_health_t;

typedef struct {
	char writing_buffer[LOG_BUFFER_SIZE];
	uint32_t byte_address;
	uint16_t file_counter;
	logging_formatter_data_type_t data_type;
	uint32_t segment_counter;
	uint16_t crc_footer;
} logging_formatter_buffer_t;

static logging_formatter_health_t logging_formatter_health = {0};
static logging_formatter_buffer_t formatter_buffer;

static SD_HandleTypeDef *logging_formatter_handle = NULL;

static const uint32_t header = 0x676bcfff;

static uint16_t CRC_footer = 0;

static const uint32_t header_footer_size = 80;

uint32_t logging_formatter_read_binary(uint8_t *buffer, uint32_t length, uint32_t start,
                                       int32_t end) {
	if ((NULL == buffer) || (0 == length)) {
		return -1;
	}

	uint32_t offset = 1;

	uint32_t counter = 0;

	for (uint32_t i = end; i > start; i--) {
		if (1 == buffer[i]) {
			counter += offset;
		}
		offset *= 2;
	}

	return counter;
}

// 16 bit calculations via cpu atm, but can use crc peripheral later on.
//  TODO: finish this
uint32_t logging_formatter_calc_crc(uint8_t *buffer) {
	if (NULL == buffer) {
		return -1;
	}

	uint16_t CRC_footer;

	return CRC_footer;
}

w_status_t logging_formatter_header_init(uint8_t *buffer) {
	uint32_t local_header = header;

	for (int i = 31; i >= 0; i--) {
		formatter_buffer.writing_buffer[i] = local_header & 1;
		local_header = local_header >> 1;
	}

	local_header = formatter_buffer.file_counter;

	for (int i = 44; i >= 32; i--) {
		formatter_buffer.writing_buffer[i] = local_header & 1;
		local_header = local_header >> 1;
	}

	// TODO: i think that this section should be moved to another file as it's these aren't constant
	// per reboot
	//       and the header init should only be called in init.
	/*
	local_header = formatter_buffer.data_type;

	for (int i = 47; i >= 45; i--) {
	    formatter_buffer.writing_buffer[i] = local_header & 1;
	    local_header = local_header >> 1;
	}

	local_header = formatter_buffer.segment_counter;

	for (int i = 79; i >= 48; i--) {
	    formatter_buffer.writing_buffer[i] = local_header & 1;
	    local_header = local_header >> 1;
	}

	local_header = formatter_buffer.crc_footer;

	for (int i = LOG_BUFFER_SIZE; i >= LOG_BUFFER_SIZE - 32; i--) {
	    formatter_buffer.writing_buffer[i] = local_header & 1;
	    local_header = local_header >> 1;
	}
	*/
	return W_SUCCESS;
}

w_status_t logging_formatting_init(SD_HandleTypeDef *p_sd_handle) {
	if (logging_formatter_health.is_init) {
		return W_SUCCESS;
	}

	if (NULL == p_sd_handle) {
		log_text(0, LOG_LVL_WARN, "RAW", "Given task handler does not exist.");
		return W_FAILURE;
	}

	logging_formatter_handle = p_sd_handle;

	uint8_t reading_buffer[BLOCKSIZE] = {0};
	uint8_t checking_buffer[BLOCKSIZE] = {0};

	uint32_t curr_address = LOG_BUFFER_SIZE;

	uint8_t zero_buffer[BLOCKSIZE];
	memset(&zero_buffer, 0, BLOCKSIZE);

	if (raw_logger_SD_read(reading_buffer, curr_address, 512, 200) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (0 == memcmp(&reading_buffer, &zero_buffer, 200)) {
		formatter_buffer.byte_address = curr_address;
		return W_SUCCESS;
	}

	uint32_t erase_size_blocks = 0;

	// to get the size of the SD card in the case that we're not solely writing with a 16GB card
	if (raw_logger_get_SD_specs(&curr_address, &erase_size_blocks, 200) != W_SUCCESS) {
		return W_FAILURE;
	}

	curr_address = (curr_address * BLOCKSIZE) / 2;
	uint32_t offset = (curr_address / 2);

	bool found_address = false;

	// binary search!
	while (!found_address || (offset > LOG_BUFFER_SIZE)) {
		if (raw_logger_SD_read(reading_buffer, curr_address, BLOCKSIZE, 200) != W_SUCCESS ||
		    raw_logger_SD_read(checking_buffer, (curr_address - LOG_BUFFER_SIZE), BLOCKSIZE, 200)) {
			return W_FAILURE;
		}

		if (0 == memcmp(&checking_buffer, &zero_buffer, 32)) {
			curr_address -= offset;
		} else if (0 != memcmp(&reading_buffer, &zero_buffer, 32)) {
			curr_address += offset;
		} else {
			formatter_buffer.byte_address = curr_address;
			found_address = true;
		}
		offset /= 2;
	}

	if (!found_address) {
		return W_FAILURE;
	}

	formatter_buffer.file_counter =
		logging_formatter_read_binary(reading_buffer, BLOCKSIZE, 32, 44) + 1;

	memset(formatter_buffer.writing_buffer, 0, LOG_BUFFER_SIZE);

	if (W_SUCCESS != logging_formatter_header_init(reading_buffer)) {
		return W_FAILURE;
	}

	logging_formatter_health.is_init = true;
	return W_SUCCESS;
}

// TODO: finish dis and also function for writing the segment counter, data type. can just use
// memcpy for the data imo
w_status_t logging_formatting_write(const uint8_t *buffer, uint32_t buffer_length,
                                    logging_formatter_data_type_t data_type) {
	if (!logging_formatter_health.is_init) {
		return W_FAILURE;
	}

	if ((buffer_length + header_footer_size) != LOG_BUFFER_SIZE) {
		return W_FAILURE;
	}

	if ((NULL == buffer) || (0 == buffer_length)) {
		return W_FAILURE;
	}

	formatter_buffer.data_type = data_type;

	logging_formatter_data_type_t address = formatter_buffer.byte_address;

	return W_SUCCESS;
}
