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
	bool timed_out;
	uint32_t read_count;
	uint32_t write_count;
	uint32_t read_error;
	uint32_t write_error;
	uint32_t not_writable;
	HAL_StatusTypeDef error_state;
} raw_sd_card_health_t;

raw_sd_card_health_t raw_sd_card_health = {0};

typedef enum {
	RAW_UNINIT,
	RAW_WRITING,
	RAW_READING,
	RAW_ERROR_OCCURRED,
	RAW_READY,
	RAW_RECOVERING
} raw_logger_action_status_t;

static SD_HandleTypeDef *raw_logger_handle = NULL;

static TaskHandle_t task_handler = NULL;

// TODO: consider having a previous state
static raw_logger_action_status_t action_state = RAW_UNINIT;

uint32_t au_kb[16] = {0x00,  0x10,   0x20,   0x40,   0x80,   0x100,  0x200,  0x400,
                      0x800, 0x1000, 0x2000, 0x3000, 0x4000, 0x6000, 0x8000, 0x10000};

w_status_t raw_logger_SD_init(SD_HandleTypeDef *p_sd_handle) {
	if (raw_sd_card_health.is_init) {
		return W_SUCCESS;
	}

	if (NULL == p_sd_handle) {
		log_text(0, LOG_LVL_WARN, "RAW", "Given task handler does not exist.");
		return W_FAILURE;
	}

	raw_logger_handle = p_sd_handle;

	action_state = RAW_READY;

	raw_sd_card_health.is_init = true;
	return W_SUCCESS;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	log_text(0, LOG_LVL_WARN, "RAW", "HAL Error callback triggered.");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	raw_sd_card_health.error_state = hsd->ErrorCode;

	if (RAW_WRITING == action_state) {
		raw_sd_card_health.write_error++;
	} else if (RAW_READING == action_state) {
		raw_sd_card_health.read_error++;
	}

	action_state = RAW_ERROR_OCCURRED;

	if (NULL == task_handler) {
		return;
	}

	vTaskNotifyGiveFromISR(task_handler, &xHigherPriorityTaskWoken);
	task_handler = NULL;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * This is an override function that is triggered once DMA is done writing
 * This function is only for WRITING, not for READING
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	UNUSED(hsd);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (NULL == task_handler) {
		return;
	}

	vTaskNotifyGiveFromISR(task_handler, &xHigherPriorityTaskWoken);

	task_handler = NULL;

	raw_sd_card_health.write_count++;
	action_state = RAW_READY;

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * This is an override function that is triggered once DMA is done reading
 * This function is only for READING, not for WRITING
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (NULL == task_handler) {
		return;
	}

	vTaskNotifyGiveFromISR(task_handler, &xHigherPriorityTaskWoken);

	task_handler = NULL;

	raw_sd_card_health.read_count++;
	action_state = RAW_READY;

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// TODO: Implement this later once recovery is figured out
w_status_t raw_logger_SD_recover(void) {
	return W_FAILURE;
}

/**
 * Used for conversion from taken SDCardStatus.AllocationUnitSize
 * uint32_t
 */
static uint32_t au_code_to_bytes(uint8_t code) {
	return (au_kb[code & 0x0fU] << 10U);
}

w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_byte) {
	if ((NULL == block_count) || (NULL == erase_size_byte)) {
		log_text(0, LOG_LVL_WARN, "RAW", "Invalid spec parameters.");
		return W_INVALID_PARAM;
	}

	HAL_SD_CardInfoTypeDef SDCardInfo = {0};
	HAL_SD_CardStatusTypeDef SDCardStatus = {0};

	if ((HAL_OK != HAL_SD_GetCardInfo(raw_logger_handle, &SDCardInfo)) ||
	    (HAL_OK != HAL_SD_GetCardStatus(raw_logger_handle, &SDCardStatus))) {
		log_text(0, LOG_LVL_WARN, "RAW", "Unable to receive card information.");
		return W_FAILURE;
	}

	uint32_t au_bytes = au_code_to_bytes(SDCardStatus.AllocationUnitSize);
	if ((0U == au_bytes) || (0U == SDCardStatus.EraseSize)) {
		log_text(0, LOG_LVL_WARN, "RAW", "Unable to receive allocation unit size.");
		return W_FAILURE;
	}

	*block_count = SDCardInfo.LogBlockNbr;
	*erase_size_byte = (uint32_t)SDCardStatus.EraseSize * au_bytes;

	return W_SUCCESS;
}

w_status_t raw_logger_SD_read(uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
                              uint32_t timeout) {
	if (!raw_sd_card_health.is_init) {
		log_text(0, LOG_LVL_WARN, "RAW", "Not initialized.");
		return W_FAILURE;
	}

	if (RAW_RECOVERING == action_state) {
		return W_FAILURE;
	}

	// Initial byte_address cannot be 0 as HAL SDMMC registers 0 as NULL
	if ((0U != (num_bytes % BLOCKSIZE)) || (0U != (byte_address % BLOCKSIZE))) {
		raw_sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	if ((NULL == buffer) || (0U == byte_address)) {
		raw_sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	// Ensure thread-safe reading
	if (RAW_READY != action_state) {
		if (RAW_ERROR_OCCURRED == action_state) {
			action_state = RAW_READY;
			raw_sd_card_health.read_error++;
			return W_FAILURE;
		}
		xTaskNotifyStateClear(task_handler);
		if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
			action_state = RAW_READY;
			raw_sd_card_health.timed_out = true;
			raw_sd_card_health.read_error++;
			log_text(0, LOG_LVL_WARN, "RAW", "Previous timed out.");
			return W_IO_TIMEOUT;
		}
	}

	action_state = RAW_READING;

	uint32_t num_blocks = num_bytes / BLOCKSIZE;
	uint32_t block_address = byte_address / BLOCKSIZE;

	task_handler = xTaskGetCurrentTaskHandle();

	xTaskNotifyStateClear(task_handler);

	if (HAL_OK != HAL_SD_ReadBlocks_DMA(raw_logger_handle, buffer, block_address, num_blocks)) {
		raw_sd_card_health.error_state = raw_logger_handle->ErrorCode;
		log_text(0, LOG_LVL_WARN, "RAW", "HAL unable to read", "error code=%d",
		         (int)(raw_sd_card_health.error_state));
		task_handler = NULL;
		action_state = RAW_ERROR_OCCURRED;
		return W_FAILURE;
	}

	if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
		action_state = RAW_READY;
		raw_sd_card_health.read_error++;
		log_text(0, LOG_LVL_WARN, "RAW", "Read timed out.");
		return W_FAILURE;
	}

	if (RAW_ERROR_OCCURRED == action_state) {
		action_state = RAW_READY;
		raw_sd_card_health.read_error++;
		log_text(0, LOG_LVL_WARN, "RAW", "Read failed.");
		return W_FAILURE;
	}

	int32_t checknumbytes = (int32_t)num_bytes;

	raw_sd_card_health.read_count++;
	return W_SUCCESS;
}

w_status_t raw_logger_SD_write(const uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
                               uint32_t timeout) {
	if (!raw_sd_card_health.is_init) {
		log_text(0, LOG_LVL_WARN, "RAW", "Not initialized.");
		return W_FAILURE;
	}

	if (RAW_RECOVERING == action_state) {
		return W_FAILURE;
	}

	// Initial byte_address cannot be 0 as HAL SDMMC registers 0 as NULL
	if ((NULL == buffer) || (0 == byte_address) || (0 == num_bytes)) {
		raw_sd_card_health.invalid_param = true;
		log_text(0, LOG_LVL_WARN, "RAW", "Invalid write params.");
		return W_INVALID_PARAM;
	}

	// Validate num_bytes
	if ((0 != (num_bytes % BLOCKSIZE)) || (0 != (byte_address % BLOCKSIZE))) {
		log_text(0, LOG_LVL_WARN, "RAW", "Wrong write param sizes.");
		return W_INVALID_PARAM;
	}

	// ensure thread-safe writing
	if (RAW_READY != action_state) {
		if (RAW_ERROR_OCCURRED == action_state) {
			action_state = RAW_READY;
			raw_sd_card_health.write_error++;
			return W_FAILURE;
		}
		xTaskNotifyStateClear(task_handler);
		if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
			action_state = RAW_READY;
			raw_sd_card_health.timed_out = true;
			raw_sd_card_health.write_error++;
			log_text(0, LOG_LVL_WARN, "RAW", "Previous timed out.");
			return W_IO_TIMEOUT;
		}
	}

	action_state = RAW_WRITING;

	uint32_t num_blocks = num_bytes / BLOCKSIZE;
	uint32_t block_address = byte_address / BLOCKSIZE;
	int32_t checknumbytes = (int32_t)num_bytes;

	task_handler = xTaskGetCurrentTaskHandle();

	xTaskNotifyStateClear(task_handler);

	if (HAL_OK != HAL_SD_WriteBlocks_DMA(raw_logger_handle, buffer, block_address, num_blocks)) {
		raw_sd_card_health.error_state = raw_logger_handle->ErrorCode;
		log_text(0, LOG_LVL_WARN, "RAW", "HAL unable to write", "error code=%d",
		         (int)(raw_sd_card_health.error_state));
		action_state = RAW_ERROR_OCCURRED;
		task_handler = NULL;
		return W_FAILURE;
	}

	if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout))) {
		action_state = RAW_READY;
		raw_sd_card_health.write_error++;
		log_text(0, LOG_LVL_WARN, "RAW", "Write timed out.");
		return W_IO_TIMEOUT;
	}

	if (RAW_ERROR_OCCURRED == action_state) {
		action_state = RAW_READY;
		raw_sd_card_health.write_error++;
		log_text(0, LOG_LVL_WARN, "RAW", "Write failed.");
		return W_FAILURE;
	}

	raw_sd_card_health.write_count++;
	return W_SUCCESS;
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

	if (raw_sd_card_health.timed_out) {
		raw_sd_card_health.timed_out = false;
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
