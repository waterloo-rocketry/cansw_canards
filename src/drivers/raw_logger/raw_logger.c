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

enum Status {
	WRITING,
	READING,
	ERROR_OCCURED,
	READY
};

SD_HandleTypeDef *raw_logger_handle;

// Consider having a previous state?
static enum Status action_state;

w_status_t raw_logger_SD_init(SD_HandleTypeDef *p_sd_handle) {
	// attempting to init the module >1 time is fine
	if (raw_sd_card_health.is_init) {
		return W_SUCCESS;
	}

	/*
	 * Initialize enum Status for thread-safe SD card operations.
	 * This enum will ensure that concurrent tasks do not access the SD card simultaneously,
	 * which helps prevent data corruption.
	 */
	action_state = READY;

	// Assigns local SD_HandleTypeDef to the given pointer
	raw_logger_handle = p_sd_handle;

	HAL_SD_Init(raw_logger_handle);

	raw_sd_card_health.is_init = true;
	return W_SUCCESS;
}

w_status_t raw_logger_SD_recover(void) {
	return W_SUCCESS;
}

w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_blocks) {
	return W_SUCCESS;
}

/**
 * I'm currently getting errors for both of these overrides, not totally sure why
 * I can't figure out if it's because they're already overridden in one of the
 * headers that we import
 * There's also a change that it's because of CubeMX doing stuff in the background
 * There is a file in cansw_canards/src/third_party/cubemx_autogen/src
 * >> bsp_driver_sd.c
 * Where there is apparently the same function which could maybe be the reason?
 * I just don't know what else it could be really
 */

/**
 * This is an override function that is triggered once DMA is done writing
 * This function is only for WRITING, not for READING
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	// maybe unused here? need to learn more about what it does
	UNUSED(hsd);

	raw_sd_card_health.write_count++;
	action_state = READY;
}

/**
 * This is an override function that is triggered once DMA is done reading
 * This function is only for READING, not for WRITING
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	// maybe unused here? need to learn more about what it does
	UNUSED(hsd);

	raw_sd_card_health.read_count++;
	action_state = READY;
}

w_status_t timeout_wait(uint32_t timeout) {
	uint32_t timer = HAL_GetTick();

	// waits for allocated period of time
	while (HAL_GetTick() - timer < timeout) {
		if (action_state == READY) {
			return W_SUCCESS;
		}
	}

	return W_FAILURE;
}

w_status_t raw_logger_SD_read(uint8_t *buffer, uint32_t block_address, uint32_t num_blocks,
                              uint32_t timeout) {
	// ensure init
	if (!raw_sd_card_health.is_init) {
		return W_FAILURE;
	}

	// validate params

	if (timeout_wait(timeout) != W_SUCCESS) {
		raw_sd_card_health.read_error++;
		return W_FAILURE;
	}

	// read
	return timeout_wait(timeout);
}

w_status_t raw_logger_SD_write(const uint8_t *buffer, uint32_t block_address, uint32_t num_blocks,
                               uint32_t timeout) {
	// ensure init
	if (!raw_sd_card_health.is_init) {
		return W_FAILURE;
	}

	// validate params somehow

	if (timeout_wait(timeout) != W_SUCCESS) {
		raw_sd_card_health.write_error++;
		return W_FAILURE;
	}

	/* WRITE
	using HAL_SD_WriteBlocks_DMA(raw_logger_handle, buffer, block_adress, num_blocks)
	HAL_SD_WriteBlocks_DMA doesn't take a timeout value, so we take that ourselves
	Process of:
	function gets called -> check params/init -> call HAL_SD_WriteBlocks_DMA
	still need to figure out if we should be using HAL_SD_GetCardState or HAL_SD_TxCpltCallback??
	Additionally, need to figure out how to only return success on completion, not on initiation
	Consider moving raw_sd_card_health.write_count++; to HAL_SD_TxCpltCallback override
	*/

	action_state = WRITING;
	// HAL_SD_WriteBlocks_DMA(raw_logger_handle, (*buffer), block_address, num_blocks);
	// ^ this is erroring for some reason fix that pls ;-;
	return timeout_wait(timeout);
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
