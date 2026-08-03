#include "drivers/sd_card/sd_card.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "fatfs.h"
#include "semphr.h"
#include <inttypes.h>

FATFS g_fs_obj;

/**
 * @brief SD card module health stats
 */
typedef struct {
	bool is_init;
	uint32_t file_create_count;
	uint32_t read_count;
	uint32_t write_count;
	uint32_t file_read_error;
	uint32_t file_write_error;
	uint32_t file_create_error;
	uint32_t not_writable;
	bool invalid_param; /**< Flag indicating a NULL/invalid argument was passed */
	bool semaphore_take_fail; /**< Flag indicating the SD mutex could not be acquired */
	bool file_error; /**< Flag indicating a file open/read/write/seek operation failed */
} sd_card_health_t;

sd_card_health_t sd_card_health = {0};

// Only 1 SD card mutex is needed because only 1 sd card exists
SemaphoreHandle_t sd_mutex = NULL;

w_status_t sd_card_init(void) {
	// attempting to init the module >1 time is fine
	if (sd_card_health.is_init) {
		return W_SUCCESS;
	}
	/*
	 * Mount the filesystem.
	 * The f_mount() function links the FATFS file system object (SDFatFS) with the logical drive.
	 * The second parameter (an empty string "") indicates the default drive.
	 * The third parameter (1) forces the mount. Decided to force mount here so this init
	 * function can check if the volume mount actually worked or not.
	 */
	// NOTE: f_mount internally calls a HAL sd init, which uses HAL_Delay(), which
	// depends on the systick timer, which is disabled if all interrupts are masked.
	// Freertos masks all interrupts before scheduler starts. Thus this function
	// must only be called AFTER scheduler starts.
	if (f_mount(&g_fs_obj, "", 1) != FR_OK) {
		return W_FAILURE;
	}

	/*
	 * Create a mutex for thread-safe SD card operations.
	 * This mutex will ensure that concurrent tasks do not access the SD card simultaneously,
	 * which helps prevent data corruption.
	 */
	sd_mutex = xSemaphoreCreateMutex();
	if (NULL == sd_mutex) {
		f_mount(NULL, "", 0); // Unmount
		return W_FAILURE;
	}

	sd_card_health.is_init = true;
	return W_SUCCESS;
}

w_status_t sd_card_file_read(const char *file_name, char *buffer, uint32_t bytes_to_read,
							 uint32_t *bytes_read) {
	// ensure init
	if (!sd_card_health.is_init) {
		return W_FAILURE;
	}

	// validate args
	if ((NULL == file_name) || (NULL == buffer) || (NULL == bytes_read)) {
		sd_card_health.file_read_error++;
		sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	/* Ensure thread-safe access to the SD card. */
	// use timeout 0 to avoid blocking
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		sd_card_health.file_read_error++;
		sd_card_health.semaphore_take_fail = true;
		return W_FAILURE;
	}

	FIL file;
	FRESULT res;

	/* Open the file in read mode. */
	res = f_open(&file, file_name, FA_READ);
	if (res != FR_OK) {
		xSemaphoreGive(sd_mutex);
		sd_card_health.file_read_error++;
		sd_card_health.file_error = true;
		return W_FAILURE;
	}

	// read into provided buffer
	res = f_read(&file, buffer, bytes_to_read, (UINT *)bytes_read);
	if (res != FR_OK) {
		f_close(&file);
		xSemaphoreGive(sd_mutex);
		sd_card_health.file_read_error++;
		sd_card_health.file_error = true;
		return W_FAILURE;
	}

	/* Close the file and release the mutex. */
	f_close(&file);
	xSemaphoreGive(sd_mutex);
	sd_card_health.read_count++;
	return W_SUCCESS;
}

w_status_t sd_card_file_write(const char *file_name, const char *buffer, uint32_t bytes_to_write,
							  bool append, uint32_t *bytes_written) {
	// validate args
	if (!sd_card_health.is_init) {
		return W_FAILURE;
	}

	if ((NULL == file_name) || (NULL == buffer) || (NULL == bytes_written)) {
		sd_card_health.file_write_error++;
		sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	/* Acquire the mutex */
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		sd_card_health.file_write_error++;
		sd_card_health.semaphore_take_fail = true;
		return W_FAILURE;
	}

	FIL file;
	FRESULT res;

	/* Open the file in write mode.
	 * Use FA_WRITE | FA_OPEN_APPEND to open always for safety in case file wasn't created
	 * successfully for some reason. This is a failsafe
	 */
	res = f_open(&file, file_name, FA_WRITE | FA_OPEN_APPEND);
	if (res != FR_OK) {
		f_mount(NULL, "", 0); // unmount then remount
		f_mount(&g_fs_obj, "", 1);
		xSemaphoreGive(sd_mutex);
		sd_card_health.file_write_error++;
		sd_card_health.file_error = true;
		return W_FAILURE;
	}

	// must deliberately move r/w ptr to start of file if not appending
	if (false == append) {
		if (f_lseek(&file, 0) != FR_OK) {
			f_close(&file);
			xSemaphoreGive(sd_mutex);
			sd_card_health.file_write_error++;
			sd_card_health.file_error = true;
			return W_FAILURE;
		}
	}

	/* Write data from buffer to file. */
	res = f_write(&file, buffer, bytes_to_write, (UINT *)bytes_written);
	if (res != FR_OK) {
		f_close(&file);
		xSemaphoreGive(sd_mutex);
		sd_card_health.file_write_error++;
		sd_card_health.file_error = true;
		return W_FAILURE;
	}

	/* Close the file and release the mutex. */
	f_close(&file);
	xSemaphoreGive(sd_mutex);
	sd_card_health.write_count++;
	return W_SUCCESS;
}

w_status_t sd_card_file_create(const char *file_name) {
	// validate args
	if (!sd_card_health.is_init) {
		return W_FAILURE;
	}

	if (NULL == file_name) {
		sd_card_health.file_create_error++;
		sd_card_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	/* Acquire the mutex  */
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		sd_card_health.file_create_error++;
		sd_card_health.semaphore_take_fail = true;
		return W_FAILURE;
	}

	FIL file;
	FRESULT res;

	/* Create a new file. The FA_CREATE_NEW flag causes the function to fail if the file already
	 * exists. */
	res = f_open(&file, file_name, FA_WRITE | FA_CREATE_NEW);
	if (res != FR_OK) {
		xSemaphoreGive(sd_mutex);
		sd_card_health.file_create_error++;
		sd_card_health.file_error = true;
		return W_FAILURE;
	}

	/* Close the file immediately since we just want to create it. */
	f_close(&file);
	xSemaphoreGive(sd_mutex);
	sd_card_health.file_create_count++;
	return W_SUCCESS;
}

w_status_t sd_card_is_writable(SD_HandleTypeDef *sd_handle) {
	/*
	 * It uses HAL_SD_GetCardState() on the SD handle (&hsd1) to check if the card is in the
	 * transfer state (HAL_SD_CARD_TRANSFER). If the card is not ready—due to being busy,
	 * ejected, or in an error state—the function returns W_FAILURE; otherwise, it returns
	 * W_SUCCESS.
	 * NOTE: same hal_delay issue as sd_card_init(). (see comment in sd_card_init for details)
	 */
	HAL_SD_CardStateTypeDef resp = HAL_SD_GetCardState(sd_handle);

	// HAL transfer is the correct state to be ready for r/w. also module must be initialized
	if ((resp == HAL_SD_CARD_TRANSFER) && sd_card_health.is_init) {
		return W_SUCCESS;
	} else {
		sd_card_health.not_writable++;
		return W_FAILURE;
	}
}

health_status_t sd_card_get_status(void) {
	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_SD_CARD,
							  .error_bitfield = 0};

	if (!sd_card_health.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_NOT_INIT_OFFSET;
	}

	if (sd_card_health.file_error) {
		sd_card_health.file_error = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_FILE_SYSTEM_OFFSET;
	}

	if (sd_card_health.invalid_param) {
		sd_card_health.invalid_param = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET;
	}

	if (sd_card_health.semaphore_take_fail) {
		sd_card_health.semaphore_take_fail = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_OS_OFFSET;
	}

	// Log operation statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "sd_card",
			 "%s, files_created=%" PRIu32 ", reads=%" PRIu32 ", writes=%" PRIu32,
			 sd_card_health.is_init ? "init" : "not init",
			 sd_card_health.file_create_count,
			 sd_card_health.read_count,
			 sd_card_health.write_count);

	log_text(0,
			 LOG_LVL_INFO,
			 "sd_card",
			 "read_err=%" PRIu32 ", write_err=%" PRIu32 ", create_err=%" PRIu32
			 ", not_writable=%" PRIu32,
			 sd_card_health.file_read_error,
			 sd_card_health.file_write_error,
			 sd_card_health.file_create_error,
			 sd_card_health.not_writable);

	return status;
}
