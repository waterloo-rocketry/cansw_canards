#include "application/logging_formatter/logging_formatter.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include <inttypes.h>

typedef struct {
	bool is_init;
	bool invalid_param;
	bool timed_out;
	uint8_t types_created;
	uint32_t write_count;
	uint32_t write_error;
} logging_formatter_health_t;

typedef struct {
	uint8_t writing_buffer[LOG_BUFFER_SIZE];
	uint64_t chunk;
	uint16_t file_identifier;
	uint16_t crc_footer;
} logging_formatter_buffer_t;

static logging_formatter_health_t logging_formatter_health = {0};
static logging_formatter_buffer_t formatter_buffer;

static SD_HandleTypeDef *logging_formatter_handle = NULL;

static const uint8_t header[4] = {67, 107, 207, 255};

static uint16_t CRC_footer = 0;

static const uint32_t header_footer_size = 10;

static uint64_t chunk_num = 0;

w_status_t logging_formatter_create_file(formatter_file_ctx_t *file, logging_formatter_data_type_t type) {

	if (!logging_formatter_health.is_init) {
		return W_FAILURE;
	}

	if (NULL == file || 0 == (uint32_t)type) {
		return W_FAILURE;
	}

	if (logging_formatter_health.types_created & type) {
		return W_FAILURE;
	}

	logging_formatter_health.types_created |= type;
	
	file->chunk_count = 0;
	file->file_identifier = formatter_buffer.file_identifier;
	file->data_type = type;

	return W_SUCCESS;
}

// 16 bit calculations via cpu atm, but can use crc peripheral later on.
// TODO: finish this
static uint16_t crc16_table[256];

/*static w_status_t crc_table_init(void) {
	for (uint16_t i = 0; i < 256; i++) {
		uint16_t crc = i;
		for (int j = 0; j < 8; j++) {
			crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : (crc >> 1);
		}
		crc16_table[i] = crc;
	}

	return W_SUCCESS;
}*/

uint16_t logging_formatter_calc_crc(const uint8_t *buffer, size_t length) {
	if (NULL == buffer) {
		return 0;
	}
	uint16_t CRC_footer = 0xffff;
	for (size_t i = 0; i < length; i++) {
		uint8_t index = (CRC_footer ^ buffer[i]) & 0xff;
		CRC_footer = (CRC_footer >> 8) ^ crc16_table[index];
	}
	return CRC_footer;
}

w_status_t logging_formatting_init(SD_HandleTypeDef *p_sd_handle) {
	if (logging_formatter_health.is_init) {
		return W_SUCCESS;
	}
	if (NULL == p_sd_handle) {
		// log_text(0, LOG_LVL_WARN, "RAW", "Given task handler does not exist.");
		return W_FAILURE;
	}

	if (W_SUCCESS != raw_logger_SD_init(p_sd_handle)) {
		return W_FAILURE;
	}

	logging_formatter_handle = p_sd_handle;

	formatter_buffer.chunk = 0;
	formatter_buffer.file_identifier = 0;
	logging_formatter_health.types_created = 0;

	uint8_t init_buffer[BLOCKSIZE] = {0};

	// first 64 blocks are reserved
	chunk_num = 1;

	uint8_t zero_buffer[BLOCKSIZE];
	memset(&zero_buffer, 0, BLOCKSIZE);

	if (raw_logger_SD_read(init_buffer, chunk_num * LOG_BUFFER_SIZE, 512, 200) != W_SUCCESS) {
		return W_FAILURE;
	}

	while (HAL_SD_CARD_TRANSFER != HAL_SD_GetCardState(logging_formatter_handle)) {
		vTaskDelay(1);
	}

	bool found_address = false;

	if (0 == memcmp(&init_buffer, &zero_buffer, 200)) {
		formatter_buffer.chunk = chunk_num;
		found_address = true;
	}

	uint32_t erase_size_blocks = 0;

	// to get the size of the SD card in the case that we're not solely writing with a 16GB card
	if (raw_logger_get_SD_specs(&chunk_num, &erase_size_blocks, 200) != W_SUCCESS) {
		return W_FAILURE;
	}
	////31283200 blocks * 512 bytes = 16,016,998,400 bytes

	chunk_num = (chunk_num / 64);

	uint32_t lo = 2;
	uint32_t hi = chunk_num;

	if (!found_address) {
		while (lo <= hi) {
			chunk_num = lo + (hi - lo) / 2;
			if (raw_logger_SD_read(init_buffer, chunk_num * LOG_BUFFER_SIZE, BLOCKSIZE, 200) !=
				W_SUCCESS) {
				return W_FAILURE;
			}

			while (HAL_SD_CARD_TRANSFER != HAL_SD_GetCardState(logging_formatter_handle)) {
				vTaskDelay(1);
			}

			if (0 == memcmp(&init_buffer, &zero_buffer, 32)) {
				hi = chunk_num - 1;
			} else {
				lo = chunk_num + 1;
			}
		}
		found_address = true;
		formatter_buffer.chunk = chunk_num;
	}

	if (raw_logger_SD_read(init_buffer, (chunk_num - 1) * LOG_BUFFER_SIZE, BLOCKSIZE, 200) !=
		    W_SUCCESS) {
		return W_FAILURE;
	}

	while (HAL_SD_CARD_TRANSFER != HAL_SD_GetCardState(logging_formatter_handle)) {
		vTaskDelay(1);
	}

	if (!found_address) {
		return W_FAILURE;
	}

	memset(formatter_buffer.writing_buffer, 0, LOG_BUFFER_SIZE);
	memcpy(formatter_buffer.writing_buffer, header, 4);

	//header_init
	if (1 != formatter_buffer.chunk) {
		uint16_t identifier = init_buffer[4];
		identifier = (identifier << 5) + (init_buffer[5] >> 3) + 1;
		formatter_buffer.file_identifier = identifier;
		formatter_buffer.writing_buffer[4] = identifier >> 5;
		formatter_buffer.writing_buffer[5] = identifier << 3;
	}

	/*if (W_SUCCESS != crc_table_init()) {
		return W_FAILURE;
	}*/

	logging_formatter_health.is_init = true;
	return W_SUCCESS;
}

// TODO: must remember to include CRC footer
w_status_t logging_formatting_write(const uint8_t *buffer, uint32_t buffer_length, formatter_file_ctx_t *ctx) {
	if (!logging_formatter_health.is_init) {
		return W_FAILURE;
	}

	if ((buffer_length + header_footer_size) != LOG_BUFFER_SIZE) {
		return W_FAILURE;
	}

	if ((NULL == buffer) || (0 == buffer_length)) {
		return W_FAILURE;
	}

	uint8_t identifier_buffer[4] = {0};

	formatter_buffer.writing_buffer[5] = (formatter_buffer.writing_buffer[5] & 11111000) + (uint8_t)(ctx->data_type & 0b0111);
	uint32_t counter = ctx->chunk_count;
	for (int i = 9; i >= 6; i--) {
		formatter_buffer.writing_buffer[i] = counter & 0b11111111;
		counter = counter >> 8;
	}
	ctx->chunk_count += 1;

	memcpy(&formatter_buffer.writing_buffer[10], buffer, buffer_length);

	formatter_buffer.crc_footer = logging_formatter_calc_crc(buffer, LOG_BUFFER_SIZE);
 
	if (W_SUCCESS != raw_logger_SD_write(formatter_buffer.writing_buffer, formatter_buffer.chunk * LOG_BUFFER_SIZE,
	                                     LOG_BUFFER_SIZE, 200)) {
		logging_formatter_health.write_error++;
		return W_FAILURE;
	}

	formatter_buffer.chunk += 1;
	logging_formatter_health.write_count++;

	return W_SUCCESS;
}

// TODO: this needs a module update in health checks for it to be actually done
health_status_t logging_formatter_get_status(void) {
	health_status_t status = {
		.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
		.module_id = CANARDS_MODULE_ID_SD_CARD,
		.error_bitfield = 0
	};

	if (!logging_formatter_health.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_NOT_INIT_OFFSET);
	}

	if (logging_formatter_health.invalid_param) {
		logging_formatter_health.invalid_param = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_INVALID_PARAM_OFFSET);
	}

	// Log operation statistics
	log_text(0, LOG_LVL_INFO, "formatter",
	         "%s, files_created=%" PRIu32 ", reads=%" PRIu32 ", writes=%" PRIu32,
	         logging_formatter_health.is_init ? "init" : "not init",
	         logging_formatter_health.write_count);

	log_text(0, LOG_LVL_INFO, "formatter",
	         "read_err=%" PRIu32 ", write_err=%" PRIu32 ", create_err=%" PRIu32
	         ", not_writable=%" PRIu32,
	         logging_formatter_health.write_error);

	return status;
}
