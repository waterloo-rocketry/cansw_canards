#include "application/logging_formatter/logging_formatter.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "common/common.h"
#include <inttypes.h>

#define CHUNK_SIZE 32768U
#define TIMEOUT_MS 1000U
#define FORMATTER_HEADER_SIZE 10U
#define FORMATTER_CRC_SIZE 2U
#define BLOCKS_PER_CHUNK 64

typedef struct {
	bool is_init;
	bool invalid_param;
	bool timed_out;
	bool card_full;
	uint16_t types_created;
	uint32_t write_count;
	uint32_t write_error;
	uint32_t data_writes;
	uint32_t error_writes;
	uint32_t text_writes;
} logging_formatter_health_t;

typedef struct {
	uint8_t writing_buffer[CHUNK_SIZE];
	uint64_t chunk;
	uint16_t file_instance_count;
} logging_formatter_buffer_t;

static logging_formatter_health_t logging_formatter_health = {0};
static logging_formatter_buffer_t formatter_buffer;

static SD_HandleTypeDef *logging_formatter_handle = NULL;

static const uint8_t HEADER[4] = {67, 107, 207, 255};

static uint64_t chunk_num = 0;

static uint16_t crc16_table[256];

static uint32_t chunk_hi = 0;

w_status_t logging_formatter_create_file(formatter_file_ctx_t *file,
                                         logging_formatter_data_type_t type) {
	if (!logging_formatter_health.is_init) {
		log_text(0, LOG_LVL_WARN, "FORMAT", "Not initialized.");
		logging_formatter_health.write_error++;
		return W_FAILURE;
	}

	if (NULL == file) {
		logging_formatter_health.invalid_param = true;
		return W_FAILURE;
	}

	// check if file of this kind has already been created
	if (logging_formatter_health.types_created & (1 << type)) {
		log_text(0, LOG_LVL_WARN, "FORMAT", "Data type was already created");
		return W_FAILURE;
	}

	logging_formatter_health.types_created |= (1 << type);

	file->chunk_count = 0;
	file->file_identifier = formatter_buffer.file_instance_count;
	file->data_type = type;

	return W_SUCCESS;
}

static w_status_t crc_table_init(void) {
	for (uint16_t i = 0; i < 256; i++) {
		uint16_t crc = i;
		for (int j = 0; j < 8; j++) {
			crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : (crc >> 1);
		}
		crc16_table[i] = crc;
	}

	return W_SUCCESS;
}

w_status_t logging_formatting_init(SD_HandleTypeDef *p_sd_handle) {
	if (logging_formatter_health.is_init) {
		return W_SUCCESS;
	}
	if (NULL == p_sd_handle) {
		return W_FAILURE;
	}

	if (W_SUCCESS != raw_logger_SD_init(p_sd_handle)) {
		log_text(0, LOG_LVL_WARN, "FORMAT", "Given task handler does not exist.");
		return W_FAILURE;
	}

	logging_formatter_handle = p_sd_handle;

	formatter_buffer.chunk = 0;
	formatter_buffer.file_instance_count = 0;
	logging_formatter_health.types_created = 0;

	uint8_t init_buffer[BLOCKSIZE] = {0};

	// first 64 blocks are reserved
	chunk_num = 1;

	uint8_t zero_buffer[BLOCKSIZE];
	memset(&zero_buffer, 0, BLOCKSIZE);

	if (raw_logger_SD_read(init_buffer, chunk_num * CHUNK_SIZE, BLOCKSIZE, TIMEOUT_MS) !=
	    W_SUCCESS) {
		return W_FAILURE;
	}

	bool found_address = false;

	if (0 == memcmp(&init_buffer, &zero_buffer, BLOCKSIZE)) {
		formatter_buffer.chunk = chunk_num;
		found_address = true;
	}

	uint32_t erase_size_blocks = 0;

	// to get the size of the SD card in the case that we're not solely writing with a 16GB card

	if (!found_address) {
		uint64_t block_count = 0;

		if (raw_logger_get_SD_specs(&block_count, &erase_size_blocks, TIMEOUT_MS) != W_SUCCESS) {
			log_text(0, LOG_LVL_WARN, "FORMAT", "Unable to get SD specs.");
			return W_FAILURE;
		}

		chunk_num = (block_count / BLOCKS_PER_CHUNK);
		uint64_t lo = 2;
		uint64_t hi = chunk_num;
		chunk_hi = hi;

		if (raw_logger_SD_read(init_buffer, hi * CHUNK_SIZE, BLOCKSIZE, TIMEOUT_MS) != W_SUCCESS) {
			return W_FAILURE;
		}

		if (0 != memcmp(&init_buffer, &zero_buffer, 32)) {
			logging_formatter_health.card_full = true;
			return W_FAILURE;
		}

		while (lo < hi) {
			chunk_num = lo + (hi - lo) / 2;
			if (raw_logger_SD_read(init_buffer, chunk_num * CHUNK_SIZE, BLOCKSIZE, TIMEOUT_MS) !=
			    W_SUCCESS) {
				return W_FAILURE;
			}

			while (HAL_SD_GetCardState(logging_formatter_handle) != HAL_SD_CARD_TRANSFER) {
				vTaskDelay(pdMS_TO_TICKS(1));
			}

			if (0 == memcmp(&init_buffer, &zero_buffer, 32)) {
				hi = chunk_num;
			} else {
				lo = chunk_num + 1;
			}
		}

		if ((block_count / BLOCKS_PER_CHUNK) <= lo) {
			logging_formatter_health.card_full = true;
			return W_FAILURE;
		}

		chunk_num = lo;
		found_address = true;
		formatter_buffer.chunk = chunk_num;

		if (raw_logger_SD_read(init_buffer, (chunk_num - 1) * CHUNK_SIZE, BLOCKSIZE, TIMEOUT_MS) !=
		    W_SUCCESS) {
			return W_FAILURE;
		}
	}

	while (HAL_SD_CARD_TRANSFER != HAL_SD_GetCardState(logging_formatter_handle)) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	if (!found_address) {
		log_text(0, LOG_LVL_WARN, "FORMAT", "Binary search failed.");
		return W_FAILURE;
	}

	crc_table_init();

	memset(formatter_buffer.writing_buffer, 0, CHUNK_SIZE);
	memcpy(formatter_buffer.writing_buffer, HEADER, 4);

	// header_init
	// identifier is a 13 bit number, take the fifth byte from init_buffer and shift it left by 5
	// and then take the sixth byte from init_buffer and shift it right by 3 to get the last 5
	// digits
	if (1 != formatter_buffer.chunk) {
		uint16_t identifier = init_buffer[4];
		identifier = (identifier << 5) + (init_buffer[5] >> 3) + 1;
		formatter_buffer.file_instance_count = identifier;
		// reversing what we did earlier, shifting right by 5 gives 8 bits (1 byte) which goes into
		// the fifth index and shifting left by three ensures we dont overwrite the data type that
		// also exists in the sixth index
		formatter_buffer.writing_buffer[4] = identifier >> 5;
		formatter_buffer.writing_buffer[5] = identifier << 3;
	}

	logging_formatter_health.is_init = true;
	return W_SUCCESS;
}

w_status_t logging_formatting_write(const uint8_t *buffer, uint32_t buffer_length,
                                    formatter_file_ctx_t *ctx) {
	if (!logging_formatter_health.is_init) {
		// TODO: log_text for errors
		return W_FAILURE;
	}

	if (logging_formatter_health.card_full) {
		return W_FAILURE;
	}

	if ((buffer_length + FORMATTER_HEADER_SIZE + FORMATTER_CRC_SIZE) != CHUNK_SIZE) {
		// TODO: log_text for errors
		return W_FAILURE;
	}

	if ((NULL == buffer) || (0 == buffer_length)) {
		logging_formatter_health.invalid_param = true;
		return W_FAILURE;
	}

	formatter_buffer.writing_buffer[5] =
		(formatter_buffer.writing_buffer[5] & 0xf8) + (uint8_t)(ctx->data_type & 0x03);
	uint32_t counter = ctx->chunk_count;
	for (int i = 9; i >= 6; i--) {
		formatter_buffer.writing_buffer[i] = (uint8_t)counter;
		counter = counter >> 8;
	}
	ctx->chunk_count += 1;

	memcpy(&formatter_buffer.writing_buffer[FORMATTER_HEADER_SIZE], buffer, buffer_length);

	uint16_t crc = 0xffff;
	for (uint32_t i = 0; i < (CHUNK_SIZE - FORMATTER_CRC_SIZE); i++) {
		uint8_t index = (uint8_t)((crc ^ formatter_buffer.writing_buffer[i]) & 0xff);
		crc = (crc >> 8) ^ crc16_table[index];
	}

	formatter_buffer.writing_buffer[CHUNK_SIZE - 2] = (uint8_t)(crc >> 8);
	formatter_buffer.writing_buffer[CHUNK_SIZE - 1] = (uint8_t)(crc & 0xff);

	// Taking the left 8 bits and then taking the right 8 bits of a 16 bit number
	// 0xFF = 0b11111111

	if (W_SUCCESS != raw_logger_SD_write(formatter_buffer.writing_buffer,
	                                     formatter_buffer.chunk * CHUNK_SIZE, CHUNK_SIZE,
	                                     TIMEOUT_MS)) {
		logging_formatter_health.write_error++;
		return W_FAILURE;
	}

	switch (ctx->data_type) {
		case LOGGING_FORMATTING_DATA:
			logging_formatter_health.data_writes++;
			break;
		case LOGGING_FORMATTING_ERROR:
			logging_formatter_health.error_writes++;
			break;
		case LOGGING_FORMATTING_TEXT:
			logging_formatter_health.text_writes++;
			break;
		default:
			break;
	}

	if (formatter_buffer.chunk == chunk_hi) {
		logging_formatter_health.card_full = true;
	}

	formatter_buffer.chunk += 1;
	logging_formatter_health.write_count++;

	return W_SUCCESS;
}

// TODO: this needs a module update in health checks for it to be actually done
// such as: timeout offset, possibly full card? idk to be considered
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
	log_text(0, LOG_LVL_INFO, "formatter", "%s, types_created=" PRIu16, ", writes=%" PRIu32,
	         ", data writes=%" PRIu32, ", error writes=%" PRIu32, ", text writes=%" PRIu32,
	         logging_formatter_health.is_init ? "init" : "not init",
	         logging_formatter_health.types_created, logging_formatter_health.write_count,
	         logging_formatter_health.data_writes, logging_formatter_health.error_writes,
	         logging_formatter_health.text_writes);

	log_text(0, LOG_LVL_INFO, "formatter", ", write_err=%" PRIu32,
	         logging_formatter_health.write_error);

	return status;
}
