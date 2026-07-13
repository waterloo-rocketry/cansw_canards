#include "FreeRTOS.h"
#include "canlib.h"
#include "littlefs/lfs.h"
#include "rocketlib/include/stm32/littlefs_sd_shim.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

#include "drivers/sd_card/sd_card.h"

#include "application/logger/log.h"

static uint32_t total_bytes = 0;
static uint32_t num_writes = 0;
extern volatile uint32_t write_dma_sum_count;
extern volatile uint32_t read_dma_sum_count;
extern volatile uint32_t error_dma_count;
extern volatile uint32_t error_timeout_count;
extern volatile uint32_t error_other_count;
extern volatile uint32_t total_sd_write_time;
extern volatile uint32_t total_num_sd_write;
extern volatile uint32_t max_write_time;
extern volatile uint32_t sd_total_data_written;
extern volatile uint32_t total_sd_read_time;
extern volatile uint32_t total_num_sd_read;
extern volatile uint32_t max_read_time;
extern volatile uint32_t sd_total_data_read;
lfs_t g_fs_obj;

extern SD_HandleTypeDef hsd2;

extern const struct lfs_config cfg;

sd_card_health_t sd_card_health = {0};

// Only 1 SD card mutex is needed because only 1 sd card exists
SemaphoreHandle_t sd_mutex = NULL;

#define SD_FILE_BUFFER_SIZE 2048

// Static 32-byte aligned file buffer for lfs_file_opencfg.
// Prevents LittleFS from heap-allocating an unaligned per-file cache which
// causes SDMMC IDMA to fault when it targets a non-DMA-accessible address.
// Safe to share: the mutex guarantees only one file is open at a time.
static uint8_t sd_file_buf[SD_FILE_BUFFER_SIZE] = {0};
static const struct lfs_file_config sd_file_cfg = {.buffer = sd_file_buf};

w_status_t sd_card_init(void) {
	// attempting to init the module >1 time is fine
	if (sd_card_health.is_init) {
		return W_SUCCESS;
	}

	// Try to mount directly at block offset 0 (no MBR)
	// NOTE: must be called AFTER scheduler starts; HAL SD uses HAL_Delay() which
	// depends on systick, which FreeRTOS masks before the scheduler starts.

    // attempt with partition first, if not use mount from 0
	if (lfsshim_sd_mount(&g_fs_obj, &hsd2, 0) != W_SUCCESS) {

		// // Get card geometry so lfs_format knows the block count
		// HAL_SD_CardInfoTypeDef card_info;
		// if (HAL_SD_GetCardInfo(&hsd2, &card_info) != HAL_OK) {
		// 	return W_FAILURE;
		// }

		// // lfs_format requires block_count != 0; supply it via a local config copy
		// struct lfs_config format_cfg = cfg;
		// format_cfg.block_count = card_info.BlockNbr;

		// if (lfs_format(&g_fs_obj, &format_cfg) != 0) {
		// 	return W_FAILURE;
		// }
		return W_FAILURE;
	}

	/*
	 * Create a mutex for thread-safe SD card operations.
	 * This mutex will ensure that concurrent tasks do not access the SD card simultaneously,
	 * which helps prevent data corruption.
	 */
	sd_mutex = xSemaphoreCreateMutex();
	if (NULL == sd_mutex) {
		lfs_unmount(&g_fs_obj);
		return W_FAILURE;
	}

	sd_card_health.is_init = true;
	return W_SUCCESS;
}

w_status_t sd_card_file_read(const char *file_name, char *buffer, uint32_t bytes_to_read,
							 uint32_t *bytes_read) {
	// validate args
	if ((!sd_card_health.is_init) || (NULL == file_name) || (NULL == buffer) ||
		(NULL == bytes_read)) {
		return W_INVALID_PARAM;
	}

	/* Ensure thread-safe access to the SD card. */
	// use timeout 0 to avoid blocking
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		return W_FAILURE;
	}

	lfs_file_t file;
	int res;

	/* Open the file in read mode. */
	res = lfs_file_opencfg(&g_fs_obj, &file, file_name, LFS_O_RDONLY, &sd_file_cfg);
	if (res != 0) {
		printf("lfs_file_open failed with error code: %d\n", res);
		xSemaphoreGive(sd_mutex);
		sd_card_health.err_count++;
		return W_FAILURE;
	}

	// read into provided buffer
	res = lfs_file_read(&g_fs_obj, &file, buffer, bytes_to_read);
	if (res < 0) {
		*bytes_read = 0;
		lfs_file_close(&g_fs_obj, &file);
		xSemaphoreGive(sd_mutex);
		sd_card_health.err_count++;
		return W_FAILURE;
	}
	*bytes_read = (uint32_t)res;

	/* Close the file and release the mutex. */
	lfs_file_close(&g_fs_obj, &file);
	xSemaphoreGive(sd_mutex);
	sd_card_health.read_count++;
	return W_SUCCESS;
}

w_status_t sd_card_file_write(const char *file_name, const char *buffer, uint32_t bytes_to_write,
							  bool append, uint32_t *bytes_written) {
	// validate args
	if ((!sd_card_health.is_init) || (NULL == file_name) || (NULL == buffer) ||
		(NULL == bytes_written)) {
		return W_INVALID_PARAM;
	}

	/* Acquire the mutex */
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		return W_FAILURE;
	}

	lfs_file_t file;
	int res;

	/* Open the file in write mode.
	 * Use LFS_O_WRONLY | LFS_O_APPEND to open always for safety in case file wasn't created
	 * successfully for some reason. This is a failsafe
	 */
	int flags = LFS_O_WRONLY | LFS_O_CREAT | (append ? LFS_O_APPEND : LFS_O_TRUNC);
	res = lfs_file_opencfg(&g_fs_obj, &file, file_name, flags, &sd_file_cfg);
	if (res != 0) {
		// lfs_unmount(&g_fs_obj);
		// lfsshim_sd_mount(&g_fs_obj, &hsd2, 0);
		// lfs_file_close(&g_fs_obj, &file);
		xSemaphoreGive(sd_mutex);
		sd_card_health.err_count++;
		return W_FAILURE;
	}

	/* Write data from buffer to file. */
	res = lfs_file_write(&g_fs_obj, &file, buffer, bytes_to_write);
	if (res < 0) {
		*bytes_written = 0;
		lfs_file_close(&g_fs_obj, &file);
		xSemaphoreGive(sd_mutex);
		sd_card_health.err_count++;
		return W_FAILURE;
	}
	*bytes_written = (uint32_t)res;

	total_bytes +=  (uint32_t)res;
	num_writes++;

	/* Close the file and release the mutex. */
	lfs_file_close(&g_fs_obj, &file);
	xSemaphoreGive(sd_mutex);
	sd_card_health.write_count++;
	return W_SUCCESS;
}

w_status_t sd_card_file_create(const char *file_name) {
	// validate args
	if ((!sd_card_health.is_init) || (NULL == file_name)) {
		return W_INVALID_PARAM;
	}

	/* Acquire the mutex  */
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		return W_FAILURE;
	}

	lfs_file_t file;
	int res;

	/* Create a new file. The LFS_O_CREAT flag causes the function to fail if the file already
	 * exists. */
	res = lfs_file_opencfg(
		&g_fs_obj, &file, file_name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL, &sd_file_cfg);
	if (res != 0) {
		xSemaphoreGive(sd_mutex);
		sd_card_health.err_count++;
		return W_FAILURE;
	}

	/* Close the file immediately since we just want to create it. */
	lfs_file_close(&g_fs_obj, &file);
	xSemaphoreGive(sd_mutex);
	sd_card_health.file_create_count++;
	return W_SUCCESS;
}


w_status_t sd_card_log_open(sd_lfs_file_t *lfs_file,
							  bool is_append) {
	// validate args
	if ((!sd_card_health.is_init) || (NULL == lfs_file)) {
		return W_INVALID_PARAM;
	}
	if ((lfs_file->state) != SD_FILE_NOT_INIT) {
		return W_FAILURE;
	}

	/* Acquire the mutex */
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		return W_FAILURE;
	}


	/* Open the file in write mode.
	 * Use LFS_O_WRONLY | LFS_O_APPEND to open always for safety in case file wasn't created
	 * successfully for some reason. This is a failsafe
	 */
	int flags = LFS_O_WRONLY | LFS_O_CREAT | (is_append ? LFS_O_APPEND : LFS_O_TRUNC);
	int res = lfs_file_opencfg(&g_fs_obj, &(lfs_file->file), lfs_file->filename, flags, &(lfs_file->sd_file_cfg));
	if (res != 0) {
		xSemaphoreGive(sd_mutex);
		lfs_file->state = SD_FILE_ERROR;
		sd_card_health.err_count++;
		return W_FAILURE;
	}

	lfs_file->state = (is_append? SD_FILE_WRITE_APPEND: SD_FILE_WRITE_TRUNC);
	xSemaphoreGive(sd_mutex);
	return W_SUCCESS;
}


w_status_t sd_card_log_write(sd_lfs_file_t *lfs_file, const char *buffer, uint32_t bytes_to_write,
							  bool is_append, uint32_t *bytes_written) {
	// validate args
	if ((!sd_card_health.is_init) || (NULL == lfs_file)) {
		return W_INVALID_PARAM;
	}
	if ((lfs_file->state) != (is_append? SD_FILE_WRITE_APPEND: SD_FILE_WRITE_TRUNC)) {
		return W_FAILURE;
	}

	/* Acquire the mutex */
	if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
		return W_FAILURE;
	}

	/* Write data from buffer to file. */
	int res = lfs_file_write(&g_fs_obj, &(lfs_file->file), buffer, bytes_to_write);
	if (res < 0) {
		sd_card_health.err_count++;
		lfs_file->state = SD_FILE_ERROR;
		return W_FAILURE;
	}
	*bytes_written = (uint32_t)res;

	total_bytes +=  (uint32_t)res;
	num_writes++;
	res = lfs_file_sync(&g_fs_obj, &(lfs_file->file));
	if (res < 0) {
		xSemaphoreGive(sd_mutex);
		sd_card_health.err_count++;
		lfs_file->state = SD_FILE_ERROR;
		return W_FAILURE;
	}
	xSemaphoreGive(sd_mutex);
	sd_card_health.write_count++;
	return W_SUCCESS;
}

w_status_t sd_card_is_writable(SD_HandleTypeDef *sd_handle) {
	/*
	 * It uses HAL_SD_GetCardState() on the SD handle (&hsd2) to check if the card is in the
	 * transfer state (HAL_SD_CARD_TRANSFER). If the card is not ready—due to being busy,
	 * ejected, or in an error state—the function returns W_FAILURE; otherwise, it returns
	 * W_SUCCESS.
	 * NOTE: same hal_delay issue as sd_card_init(). (see comment in sd_card_init for details)
	 */
	HAL_SD_CardStateTypeDef resp = HAL_SD_GetCardState(sd_handle);

	// HAL transfer is the correct state to be ready for r/w. also module must be initialized
	if ((resp == HAL_SD_CARD_TRANSFER) && (true == sd_card_health.is_init)) {
		return W_SUCCESS;
	} else {
		return W_FAILURE;
	}
}

health_status_t sd_card_get_status(void) {
	health_status_t status = {
		.severity = HEALTH_OK, .module_id = MODULE_SD_CARD, .error_bitfield = 0};

	if (sd_card_health.is_init == false) {
		status.severity = HEALTH_ERROR;
		status.error_bitfield |= 1 << MODULE_ERR_NOT_INIT;
	}

	// Log operation statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "sd_card",
			 "%s files_created=%lu, reads=%lu, writes=%lu, error=%lu",
			 sd_card_health.is_init ? "init" : "not init",
			 sd_card_health.file_create_count,
			 sd_card_health.read_count,
			 sd_card_health.write_count,
			sd_card_health.err_count);

	log_text(10, LOG_LVL_INFO, "sd_card", 
		"ERR DMA %" PRIu32 ", ERR TIME %" PRIu32 ", ERR OTHER %" PRIu32 , error_dma_count, error_timeout_count, error_other_count);

	log_text(10, LOG_LVL_INFO, "sd_card", 
		 "WR TIME %" PRIu32 ", # WR %" PRIu32 ", WR BYTE %"PRIu32" AVE WR %lf, MAX: %" PRIu32, total_sd_write_time, total_num_sd_write, sd_total_data_written, ((float64_t)sd_total_data_written/(float64_t)total_sd_write_time), max_write_time);

	log_text(10, LOG_LVL_INFO, "sd_card", 
		 "RD TIME %" PRIu32 ", # RD %" PRIu32 ", RD BYTE %"PRIu32" AVE RD %lf, MAX: %" PRIu32, total_sd_read_time, total_num_sd_read, sd_total_data_read, ((float64_t)sd_total_data_read/(float64_t)total_sd_read_time), max_read_time);

	return status;
}
