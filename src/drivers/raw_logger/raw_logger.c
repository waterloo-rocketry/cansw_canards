#include "drivers/raw_logger/raw_logger.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "semphr.h"
#include <inttypes.h>

/**
 * @brief SD card module health stats
 */
typedef struct {
	bool is_init;
	bool invalid_param;
	bool semaphore_take_fail;
	uint32_t read_count;
	uint32_t write_count;
	uint32_t read_error;
	uint32_t write_error;
	uint32_t not_writable;
} raw_sd_card_health_t;

raw_sd_card_health_t raw_sd_card_health = {0};

SemaphoreHandle_t sd_semaphore = NULL;

w_status_t raw_logger_SD_init(void) {
	// attempting to init the module >1 time is fine
	if (raw_sd_card_health.is_init) {
		return W_SUCCESS;
	}

	/*
	 * Create a binary semaphore for thread-safe SD card operations.
	 * This semaphore will ensure that concurrent tasks do not access the SD card simultaneously,
	 * which helps prevent data corruption.
	 */
	sd_semaphore = xSemaphoreCreateBinary();
	if (NULL == sd_semaphore) {
		return W_FAILURE;
	}

	raw_sd_card_health.is_init = true;
	return W_SUCCESS;
}

w_status_t raw_logger_SD_recover(void) {
	return W_SUCCESS;
}

w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_blocks) {
	return W_SUCCESS;
}

w_status_t raw_logger_SD_read(uint32_t block_address, uint32_t *buffer, uint32_t num_blocks) {
	// ensure init
	if (!raw_sd_card_health.is_init) {
		return W_FAILURE;
	}

	raw_sd_card_health.read_count++;
	return W_SUCCESS;
}

w_status_t raw_logger_SD_write(uint32_t block_address, const uint32_t *buffer,
                               uint32_t num_blocks) {
	// validate args
	if (!raw_sd_card_health.is_init) {
		return W_FAILURE;
	}

	raw_sd_card_health.write_count++;
	return W_SUCCESS;
}

w_status_t raw_logger_SD_is_writable(SD_HandleTypeDef *p_sd_handle) {
	/*
	 * It uses HAL_SD_GetCardState() on the SD handle (&hsd1) to check if the card is in the
	 * transfer state (HAL_SD_CARD_TRANSFER). If the card is not ready—due to being busy,
	 * ejected, or in an error state—the function returns W_FAILURE; otherwise, it returns
	 * W_SUCCESS.
	 * NOTE: same hal_delay issue as sd_card_init(). (see comment in sd_card_init for details)
	 */
	HAL_SD_CardStateTypeDef resp = HAL_SD_GetCardState(p_sd_handle);

	// HAL transfer is the correct state to be ready for r/w. also module must be initialized
	if ((resp == HAL_SD_CARD_TRANSFER) && raw_sd_card_health.is_init) {
		return W_SUCCESS;
	} else {
		raw_sd_card_health.not_writable++;
		return W_FAILURE;
	}
}

health_status_t raw_logger_SD_get_status(void) {
	health_status_t status = {
		.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
		.module_id = CANARDS_MODULE_ID_SD_CARD,
		.error_bitfield = 0
	};

	if (!raw_sd_card_health.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_NOT_INIT_OFFSET);
	}

	if (raw_sd_card_health.invalid_param) {
		raw_sd_card_health.invalid_param = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_INVALID_PARAM_OFFSET);
	}

	if (raw_sd_card_health.semaphore_take_fail) {
		raw_sd_card_health.semaphore_take_fail = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= ((1U) << CANARDS_MODULE_E_OS_OFFSET);
	}

	// Log operation statistics
	log_text(0, LOG_LVL_INFO, "sd_card", "%s, reads=%" PRIu32 ", writes=%" PRIu32,
	         raw_sd_card_health.is_init ? "init" : "not init", raw_sd_card_health.read_count,
	         raw_sd_card_health.write_count);

	log_text(0, LOG_LVL_INFO, "sd_card", ", not_writable=%" PRIu32,
	         raw_sd_card_health.not_writable);

	return status;
}
