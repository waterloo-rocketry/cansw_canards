#include "drivers/raw_logger/raw_logger.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
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
	uint32_t error_state;
} raw_sd_card_health_t;

raw_sd_card_health_t raw_sd_card_health = {0};

/**
 * @brief Action state enumerations
 *
 * RAW_RECOVERING is entered when an operation times out or the driver can no
 * longer safely continue normal operation. New read/writes are rejected
 * until recovery restores the driver to RAW_READY.
 *
 * However, this does not guarantee that recovery is currently running
 * or that an outstanding DMA transfer has stopped.
 */
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

static SemaphoreHandle_t sd_mutex = NULL;

// TODO: consider having a previous state
static raw_logger_action_status_t action_state = RAW_UNINIT;

w_status_t raw_logger_SD_init(SD_HandleTypeDef *p_sd_handle) {
	if (raw_sd_card_health.is_init) {
		return W_SUCCESS;
	}

	if (NULL == p_sd_handle) {
		log_text(0, LOG_LVL_WARN, "RAW", "Given task handler does not exist.");
		return W_FAILURE;
	}

	sd_mutex = xSemaphoreCreateMutex();

	if (NULL == sd_mutex) {
		return W_FAILURE;
	}

	raw_logger_handle = p_sd_handle;

	action_state = RAW_READY;

	raw_sd_card_health.is_init = true;
	return W_SUCCESS;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	(void)hsd;
	if ((RAW_WRITING != action_state) && (RAW_READING != action_state)) {
		return;
	}

	// log_text(0, LOG_LVL_WARN, "RAW", "HAL Error callback triggered.");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	raw_sd_card_health.error_state = hsd->ErrorCode;

	if (RAW_WRITING == action_state) {
		raw_sd_card_health.write_error++;
	} else if (RAW_READING == action_state) {
		raw_sd_card_health.read_error++;
	}

	action_state = RAW_ERROR_OCCURRED;

	TaskHandle_t task = task_handler;
	task_handler = NULL;

	if (NULL != task) {
		vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	return;
}

/**
 * This is an override function that is triggered once DMA is done writing
 * This function is only for WRITING, not for READING
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	(void)hsd;
	if (RAW_WRITING != action_state) {
		return;
	}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (NULL == task_handler) {
		return;
	}

	raw_sd_card_health.write_count++;
	action_state = RAW_READY;

	TaskHandle_t task = task_handler;
	task_handler = NULL;

	if (NULL != task) {
		vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
 * This is an override function that is triggered once DMA is done reading
 * This function is only for READING, not for WRITING
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	(void)hsd;
	if (RAW_READING != action_state) {
		return;
	}

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (NULL == task_handler) {
		return;
	}

	raw_sd_card_health.read_count++;
	action_state = RAW_READY;

	TaskHandle_t task = task_handler;
	task_handler = NULL;

	if (NULL != task) {
		vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

// TODO: Implement this later once recovery is figured out
w_status_t raw_logger_SD_recover(void) {
	return W_FAILURE;
}

#define SD_AU_TABLE_SIZE (16U)

static const uint32_t AU_SIZE_KB_LOOKUP[SD_AU_TABLE_SIZE] = {0U,     16U,    32U,    64U,
                                                             128U,   256U,   512U,   1024U,
                                                             2048U,  4096U,  8192U,  12288U,
                                                             16384U, 24576U, 32768U, 65536U};

/**
 * @brief Converts SD Status AU (Allocation Unit) code to size in bytes
 *
 * @param[in] code 4-bit Allocation Unit code from SD_STATUS register
 * @return uint32_t AU size in bytes, or 0U if code is invalid/unspecified
 */
static uint32_t au_code_to_bytes(uint8_t code) {
	uint8_t index = code & 0x0fU;

	if (index == 0U) {
		return 0U;
	}

	// convert KiB to bytes
	return AU_SIZE_KB_LOOKUP[index] << 10U;
}

w_status_t raw_logger_get_SD_specs(uint64_t *block_count, uint32_t *erase_size_byte,
                                   uint32_t timeout_ms) {
	if (!raw_sd_card_health.is_init) {
		log_text(0, LOG_LVL_WARN, "RAW", "Not initialized.");
		return W_FAILURE;
	}

	if ((NULL == block_count) || (NULL == erase_size_byte)) {
		log_text(0, LOG_LVL_WARN, "RAW", "Invalid spec parameters.");
		return W_INVALID_PARAM;
	}

	HAL_SD_CardInfoTypeDef SDCardInfo = {0};
	HAL_SD_CardStatusTypeDef SDCardStatus = {0};

	if (pdTRUE != xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(timeout_ms))) {
		raw_sd_card_health.timed_out = true;
		return W_IO_TIMEOUT;
	}

	if (RAW_READY != action_state) {
		xSemaphoreGive(sd_mutex);
		return W_FAILURE;
	}

	if ((HAL_OK != HAL_SD_GetCardInfo(raw_logger_handle, &SDCardInfo)) ||
	    (HAL_OK != HAL_SD_GetCardStatus(raw_logger_handle, &SDCardStatus))) {
		log_text(0, LOG_LVL_WARN, "RAW", "Unable to receive card information.");
		xSemaphoreGive(sd_mutex);
		return W_FAILURE;
	}

	xSemaphoreGive(sd_mutex);

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
                              uint32_t timeout_ms) {
	if (!raw_sd_card_health.is_init) {
		log_text(0, LOG_LVL_WARN, "RAW", "Not initialized.");
		return W_FAILURE;
	}

	if (RAW_RECOVERING == action_state) {
		return W_FAILURE;
	}

	// Initial byte_address cannot be 0 as HAL SDMMC registers 0 as NULL
	if ((NULL == buffer) || (0U == byte_address) || (0U == num_bytes)) {
		raw_sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	// Validate alignment
	if ((0U != (num_bytes % BLOCKSIZE)) || (0U != (byte_address % BLOCKSIZE))) {
		raw_sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	uint32_t start_time_ms = 0;
	if (timer_get_ms(&start_time_ms) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (pdTRUE != xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(timeout_ms))) {
		raw_sd_card_health.timed_out = true;
		return W_IO_TIMEOUT;
	}

	if (RAW_READY != action_state) {
		xSemaphoreGive(sd_mutex);
		return W_FAILURE;
	}

	while (HAL_SD_CARD_TRANSFER != HAL_SD_GetCardState(raw_logger_handle)) {
		vTaskDelay(1);
		uint32_t cur_time_ms = 0;
		if (timer_get_ms(&cur_time_ms) != W_SUCCESS) {
			xSemaphoreGive(sd_mutex);
			return W_FAILURE;
		}
		if ((cur_time_ms - start_time_ms) >= timeout_ms) {
			action_state = RAW_RECOVERING;
			raw_sd_card_health.timed_out = true;
			xSemaphoreGive(sd_mutex);
			return W_IO_TIMEOUT;
		}
	}

	uint32_t pre_DMA_time_ms = 0;
	if ((timer_get_ms(&pre_DMA_time_ms) != W_SUCCESS) ||
	    ((pre_DMA_time_ms - start_time_ms) >= timeout_ms)) {
		raw_sd_card_health.timed_out = true;
		xSemaphoreGive(sd_mutex);
		return W_IO_TIMEOUT;
	}

	action_state = RAW_READING;

	uint32_t num_blocks = num_bytes / BLOCKSIZE;
	uint32_t block_address = byte_address / BLOCKSIZE;

	task_handler = xTaskGetCurrentTaskHandle();

	ulTaskNotifyTake(pdTRUE, 0);

	if (HAL_OK != HAL_SD_ReadBlocks_DMA(raw_logger_handle, buffer, block_address, num_blocks)) {
		raw_sd_card_health.error_state = raw_logger_handle->ErrorCode;
		log_text(0, LOG_LVL_WARN, "RAW", "HAL unable to read", "error code=0x%08lX",
		         (unsigned long)(raw_sd_card_health.error_state));
		task_handler = NULL;
		action_state = RAW_ERROR_OCCURRED;
		raw_sd_card_health.read_error++;
		xSemaphoreGive(sd_mutex);
		return W_FAILURE;
	}

	uint32_t cur_time_ms = 0;
	if ((W_SUCCESS != timer_get_ms(&cur_time_ms)) ||
	    ((cur_time_ms - start_time_ms) >= timeout_ms) ||
	    (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms + start_time_ms - cur_time_ms)))) {
		action_state = RAW_RECOVERING;
		task_handler = NULL;
		raw_sd_card_health.read_error++;
		raw_sd_card_health.timed_out = true;
		log_text(0, LOG_LVL_WARN, "RAW", "Read timed out.");
		xSemaphoreGive(sd_mutex);
		return W_IO_TIMEOUT;
	}

	xSemaphoreGive(sd_mutex);

	if (RAW_ERROR_OCCURRED == action_state) {
		log_text(0, LOG_LVL_WARN, "RAW", "Read failed.");
		return W_FAILURE;
	}

	if (RAW_READY != action_state) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

w_status_t raw_logger_SD_write(const uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
                               uint32_t timeout_ms) {
	if (!raw_sd_card_health.is_init) {
		log_text(0, LOG_LVL_WARN, "RAW", "Not initialized.");
		return W_FAILURE;
	}

	if (RAW_RECOVERING == action_state) {
		return W_FAILURE;
	}

	// Initial byte_address cannot be 0 as HAL SDMMC registers 0 as NULL
	if ((NULL == buffer) || (0U == byte_address) || (0U == num_bytes)) {
		raw_sd_card_health.invalid_param = true;
		log_text(0, LOG_LVL_WARN, "RAW", "Invalid write params.");
		return W_INVALID_PARAM;
	}

	// Validate alignment
	if ((0U != (num_bytes % BLOCKSIZE)) || (0U != (byte_address % BLOCKSIZE))) {
		raw_sd_card_health.invalid_param = true;
		log_text(0, LOG_LVL_WARN, "RAW", "Wrong write param sizes.");
		return W_INVALID_PARAM;
	}

	uint32_t start_time_ms = 0;
	if (timer_get_ms(&start_time_ms) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (pdTRUE != xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(timeout_ms))) {
		raw_sd_card_health.timed_out = true;
		return W_IO_TIMEOUT;
	}

	if (RAW_READY != action_state) {
		xSemaphoreGive(sd_mutex);
		return W_FAILURE;
	}

	while (HAL_SD_CARD_TRANSFER != HAL_SD_GetCardState(raw_logger_handle)) {
		vTaskDelay(1);
		uint32_t cur_time_ms = 0;
		if (timer_get_ms(&cur_time_ms) != W_SUCCESS) {
			xSemaphoreGive(sd_mutex);
			return W_FAILURE;
		}
		if ((cur_time_ms - start_time_ms) >= timeout_ms) {
			action_state = RAW_RECOVERING;
			raw_sd_card_health.timed_out = true;
			xSemaphoreGive(sd_mutex);
			return W_IO_TIMEOUT;
		}
	}

	uint32_t pre_DMA_time_ms = 0;
	if ((timer_get_ms(&pre_DMA_time_ms) != W_SUCCESS) ||
	    ((pre_DMA_time_ms - start_time_ms) >= timeout_ms)) {
		raw_sd_card_health.timed_out = true;
		xSemaphoreGive(sd_mutex);
		return W_IO_TIMEOUT;
	}

	action_state = RAW_WRITING;

	uint32_t num_blocks = num_bytes / BLOCKSIZE;
	uint32_t block_address = byte_address / BLOCKSIZE;

	task_handler = xTaskGetCurrentTaskHandle();

	ulTaskNotifyTake(pdTRUE, 0);

	if (HAL_OK != HAL_SD_WriteBlocks_DMA(raw_logger_handle, buffer, block_address, num_blocks)) {
		raw_sd_card_health.error_state = raw_logger_handle->ErrorCode;
		log_text(0, LOG_LVL_WARN, "RAW", "HAL unable to write", "error code=0x%08lX",
		         (unsigned long)(raw_sd_card_health.error_state));
		action_state = RAW_ERROR_OCCURRED;
		task_handler = NULL;
		raw_sd_card_health.write_error++;
		xSemaphoreGive(sd_mutex);
		return W_FAILURE;
	}

	uint32_t cur_time_ms = 0;
	if ((W_SUCCESS != timer_get_ms(&cur_time_ms)) ||
	    ((cur_time_ms - start_time_ms) >= timeout_ms) ||
	    (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms + start_time_ms - cur_time_ms)))) {
		action_state = RAW_RECOVERING;
		task_handler = NULL;
		raw_sd_card_health.write_error++;
		raw_sd_card_health.timed_out = true;
		log_text(0, LOG_LVL_WARN, "RAW", "Write timed out.");
		xSemaphoreGive(sd_mutex);
		return W_IO_TIMEOUT;
	}

	xSemaphoreGive(sd_mutex);

	if (RAW_ERROR_OCCURRED == action_state) {
		log_text(0, LOG_LVL_WARN, "RAW", "Write failed.");
		return W_FAILURE;
	}

	if (RAW_READY != action_state) {
		return W_FAILURE;
	}

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
