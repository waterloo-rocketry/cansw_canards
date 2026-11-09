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
    uint16_t file_identifier;
    uint32_t segment_counter;
} logging_formatter_buffer_t;

static logging_formatter_health_t logging_formatter_health = {0};
static logging_formatter_buffer_t formatter_write_buffer;

static SD_HandleTypeDef *logging_formatter_handle = NULL;

static const uint32_t header = 0xD7DDAFBF;

static const uint32_t header_footer_size = 80;

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

	if (raw_logger_SD_read(&reading_buffer, curr_address, 512, 200) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (0 == memcmp(&reading_buffer, &zero_buffer, 200)) {
		formatter_write_buffer.byte_address = curr_address;
		return W_SUCCESS;
	}

	uint32_t erase_size_blocks = 0;

    //to get the size of the SD card in the case that we're not solely writing with a 16GB card
	if (raw_logger_get_SD_specs(&curr_address, &erase_size_blocks, 200) != W_SUCCESS) {
		return W_FAILURE;
	}

	curr_address = (curr_address * BLOCKSIZE) / 2;

    bool found_address = false;

	while (!found_address || (curr_address >= LOG_BUFFER_SIZE)) {
		if (raw_logger_SD_read(&reading_buffer, curr_address, BLOCKSIZE, 200) != W_SUCCESS ||
		    raw_logger_SD_read(&checking_buffer, (curr_address - LOG_BUFFER_SIZE), BLOCKSIZE,
		                       200)) {
			return W_FAILURE;
		}

		if (0 == memcmp(&checking_buffer, &zero_buffer, 32)) {
			curr_address -= (curr_address / 2);
		} else if (0 != memcmp(&reading_buffer, &zero_buffer, 32)) {
			curr_address += (curr_address / 2);
		} else {
			formatter_write_buffer.byte_address = curr_address;
			found_address = true;
		}
	}

    memset(formatter_write_buffer.writing_buffer, 0, LOG_BUFFER_SIZE);

	logging_formatter_health.is_init = true;
	return W_SUCCESS;
}

w_status_t logging_formatting_write(const uint8_t *buffer, uint32_t buffer_length,
                                    logging_formatter_data_type_t data_type) {

    if (!logging_formatter_health.is_init) {
        return W_FAILURE;
    }

    if ((buffer_length + header_footer_size) != LOG_BUFFER_SIZE) {
        return W_FAILURE;
    }

}