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
	uint32_t read_count;
	uint32_t write_count;
	uint32_t read_error;
	uint32_t write_error;
	uint32_t not_writable;
	HAL_StatusTypeDef error_state;
} raw_sd_card_health_t;

raw_sd_card_health_t raw_sd_card_health = {0};

enum Status {
	WRITING,
	READING,
	ERROR_OCCURRED,
	READY,
	RECOVERING
};

SD_HandleTypeDef *raw_logger_handle;

TaskHandle_t task_handler;

// Consider having a previous state?
static enum Status action_state;

w_status_t raw_logger_SD_init(SD_HandleTypeDef *p_sd_handle) {
	if (raw_sd_card_health.is_init) {
		return W_SUCCESS;
	}

	/*
	 * Initialize enum Status for thread-safe SD card operations.
	 * This enum will ensure that concurrent tasks do not access the SD card simultaneously,
	 * which helps prevent data corruption.
	 */
	action_state = READY;

	task_handler = NULL;

	// Assigns local SD_HandleTypeDef to the given pointer
	raw_logger_handle = p_sd_handle;

	raw_sd_card_health.is_init = true;
	return W_SUCCESS;
}

/**
 * Currently not implemented as per Shiming
 */
w_status_t raw_logger_SD_recover(void) {
	/*action_state = RECOVERING;

	HAL_SDMMC_DisableInterrupts();

	HAL_SD_DeInit(&*raw_logger_handle);

	vTaskDelay(pdMS_TO_TICKS(50));

	HAL_SD_Init(&*raw_logger_handle);*/

	return W_FAILURE;
}

w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_byte) {
	if ((NULL == block_count) || (NULL == erase_size_byte)) {
		return W_FAILURE;
	}

	HAL_SD_CardInfoTypeDef SDCardInfo;
	HAL_SD_CardStatusTypeDef SDCardStatus;

	if ((HAL_OK == HAL_SD_GetCardInfo(raw_logger_handle, &SDCardInfo)) &&
	    (HAL_OK == HAL_SD_GetCardStatus(raw_logger_handle, &SDCardStatus))) {
		*block_count = SDCardInfo.BlockNbr;
		*erase_size_byte = (uint32_t)SDCardStatus.EraseSize * 512;
	} else {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

/**
 * This is an override function that is triggered once DMA is done writing
 * This function is only for WRITING, not for READING
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	if (action_state == RECOVERING) {
		return;
	}

	UNUSED(hsd);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (task_handler == NULL) {
		return;
	}

	vTaskNotifyGiveFromISR(task_handler, &xHigherPriorityTaskWoken);

	task_handler = NULL;

	raw_sd_card_health.write_count++;
	action_state = READY;

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * This is an override function that is triggered once DMA is done reading
 * This function is only for READING, not for WRITING
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	if (action_state == RECOVERING) {
		return;
	}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (task_handler == NULL) {
		return;
	}

	vTaskNotifyGiveFromISR(task_handler, &xHigherPriorityTaskWoken);

	task_handler = NULL;

	raw_sd_card_health.read_count++;
	action_state = READY;

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

w_status_t raw_logger_SD_read(uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
                              uint32_t timeout) {
	// ensure init
	if (!raw_sd_card_health.is_init) {
		return W_FAILURE;
	}

	if (action_state == RECOVERING) {
		return W_FAILURE;
	}

	/** Validate params
	 * The reason why we require the byte_address to not be 0 is that if you attempt to use DMA
	 * to write to address 0, you get stick in an infinite loop in stm32h7xx_ll_sdmmc.c due to
	 * something (not totally sure what) reading the register as 0. If you write to byte address 512
	 * it works, but for some reason 0 doesn't. Might be due to OS reading 0 as NULL, or if that
	 * block is already being used by the microcontroller to store information.
	 */

	if ((num_bytes % 512 != 0) || (byte_address % 512 != 0) || (buffer == NULL) ||
	    (byte_address == 0) || ((buffer == NULL))) {
		raw_sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	// ensure thread-safe reading
	if (action_state != READY) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout)) == 0) {
			raw_sd_card_health.read_error++;
			return W_IO_TIMEOUT;
		}
	}

	action_state = READING;

	uint32_t num_blocks = num_bytes / 512;
	uint32_t block_address = byte_address / 512;

	// Do not currently use DCache but still good for implementation in case we do
	SCB_InvalidateDCache_by_Addr((uint32_t *)buffer, sizeof(buffer));

	/**
	 * I'm not sure if this is quite why I was failing to write/read for a while, but essentially
	 * there's something about calling xTaskGetCurrentTaskHandle() before you read/write, as
	 * if the interrupt fires while you're still calling xTaskGetCurrentTaskHandle() the callback
	 * just returns null WITHOUT NOTIFYING ANYTHING and you hang in ISR. When calling it before
	 * the code runs and benchmarks fine; when calling it after, it gets stuck in tasks.c.
	 */
	task_handler = xTaskGetCurrentTaskHandle();

	if (HAL_OK != HAL_SD_ReadBlocks_DMA(raw_logger_handle, buffer, num_blocks, block_address)) {
		task_handler = NULL;
	}

	SCB_InvalidateDCache_by_Addr((uint32_t *)buffer, sizeof(buffer));

	if (0 == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
		raw_sd_card_health.read_error++;
		return W_IO_TIMEOUT;
	}

	raw_sd_card_health.read_count++;
	return W_SUCCESS;
}

w_status_t raw_logger_SD_write(const uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
                               uint32_t timeout) {
	// ensure init
	if (!raw_sd_card_health.is_init) {
		return W_FAILURE;
	}

	if (action_state == RECOVERING) {
		return W_FAILURE;
	}

	/** Validate params
	 * The reason why we require the byte_address to not be 0 is that if you attempt to use DMA
	 * to write to address 0, you get stick in an infinite loop in stm32h7xx_ll_sdmmc.c due to
	 * something (not totally sure what) reading the register as 0. If you write to byte address 512
	 * it works, but for some reason 0 doesn't. Might be due to OS reading 0 as NULL, or if that
	 * block is already being used by the microcontroller to store information.
	 */

	if ((buffer == NULL) || (byte_address == 0) || (num_bytes == 0)) {
		raw_sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	// Validate num_bytes
	if ((num_bytes % 512 != 0) || (byte_address % 512 != 0)) {
		return W_INVALID_PARAM;
	}

	// ensure thread-safe writing
	if (action_state != READY) {
		if (0 == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
			raw_sd_card_health.write_error++;
			return W_IO_TIMEOUT;
		}
	}

	action_state = WRITING;

	uint32_t num_blocks = num_bytes / 512;
	uint32_t block_address = byte_address / 512;

	// Do not currently use DCache but still good for implementation in case we do
	SCB_CleanDCache_by_Addr((uint32_t *)buffer, sizeof(buffer));

	/**
	 * I'm not sure if this is quite why I was failing to write/read for a while, but essentially
	 * there's something about calling xTaskGetCurrentTaskHandle() before you read/write, as
	 * if the interrupt fires while you're still calling xTaskGetCurrentTaskHandle() the callback
	 * just returns null WITHOUT NOTIFYING ANYTHING and you hang in ISR. When calling it before
	 * the code runs and benchmarks fine; when calling it after, it gets stuck in tasks.c.
	 */
	task_handler = xTaskGetCurrentTaskHandle();

	if (HAL_OK != HAL_SD_WriteBlocks_DMA(raw_logger_handle, buffer, num_blocks, block_address)) {
		task_handler = NULL;
	}

	SCB_InvalidateDCache_by_Addr((uint32_t *)buffer, sizeof(buffer));

	if (0 == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
		raw_sd_card_health.write_error++;
		return W_IO_ERROR;
	}

	raw_sd_card_health.write_count++;
	return W_SUCCESS;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	raw_sd_card_health.error_state = hsd->ErrorCode;

	if (action_state == WRITING) {
		raw_sd_card_health.write_error++;
	} else if (action_state == READING) {
		raw_sd_card_health.read_error++;
	}

	action_state = ERROR_OCCURRED;

	if (task_handler == NULL) {
		return;
	}

	vTaskNotifyGiveFromISR(task_handler, &xHigherPriorityTaskWoken);
	task_handler = NULL;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

	// Log operation statistics
	log_text(0, LOG_LVL_INFO, "sd_card", "%s, reads=%" PRIu32 ", writes=%" PRIu32,
	         raw_sd_card_health.is_init ? "init" : "not init", raw_sd_card_health.read_count,
	         raw_sd_card_health.write_count);

	log_text(0, LOG_LVL_INFO, "sd_card", ", not_writable=%" PRIu32,
	         raw_sd_card_health.not_writable);

	return status;
}
