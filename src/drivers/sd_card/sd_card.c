#include "drivers/sd_card/sd_card.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "littlefs/lfs.h"
#include "semphr.h"
#include "stm32h7xx_hal_sd.h"
#include "rocketlib/include/littlefs_sd_shim.h"

lfs_t g_fs_obj;

extern SD_HandleTypeDef hsd1;

extern const struct lfs_config cfg;

sd_card_health_t sd_card_health = {0};

// Only 1 SD card mutex is needed because only 1 sd card exists
SemaphoreHandle_t sd_mutex = NULL;

w_status_t sd_card_init(void) {
    // attempting to init the module >1 time is fine
    if (sd_card_health.is_init) {
        return W_SUCCESS;
    }

    // Try to mount directly at block offset 0 (no MBR)
    // NOTE: must be called AFTER scheduler starts; HAL SD uses HAL_Delay() which
    // depends on systick, which FreeRTOS masks before the scheduler starts.
    if (lfsshim_sd_mount(&g_fs_obj, &hsd1, 0) != W_SUCCESS) {
        printf("Mount failed, formatting SD card\n");

        // Get card geometry so lfs_format knows the block count
        HAL_SD_CardInfoTypeDef card_info;
        if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK) {
            printf("Could not get SD card info\n");
            return W_FAILURE;
        }

        // lfs_format requires block_count != 0; supply it via a local config copy
        struct lfs_config format_cfg = cfg;
        format_cfg.block_count = card_info.BlockNbr;

        if (lfs_format(&g_fs_obj, &format_cfg) != 0) {
            printf("Could not format SD card\n");
            return W_FAILURE;
        }

        printf("Formatted SD card\n");

        if (lfsshim_sd_mount(&g_fs_obj, &hsd1, 0) != W_SUCCESS) {
            printf("Remount after format failed\n");
            return W_FAILURE;
        }
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

w_status_t sd_card_file_read(
    const char *file_name, char *buffer, uint32_t bytes_to_read, uint32_t *bytes_read
) {
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
    res = lfs_file_open(&g_fs_obj, &file, file_name, LFS_O_RDONLY);
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

w_status_t sd_card_file_write(
    const char *file_name, const char *buffer, uint32_t bytes_to_write, bool append,
    uint32_t *bytes_written
) {
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
    int flags = LFS_O_WRONLY | LFS_O_CREAT | (append ? LFS_O_APPEND : 0);
    res = lfs_file_open(&g_fs_obj, &file, file_name, flags);
    if (res != 0) {
        lfs_unmount(&g_fs_obj);
        lfsshim_sd_mount(&g_fs_obj, &hsd1, 0);
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        return W_FAILURE;
    }

    // when not appending, truncate to zero so the write fully replaces the file contents
    if (!append) {
        if (lfs_file_truncate(&g_fs_obj, &file, 0) < 0) {
            lfs_file_close(&g_fs_obj, &file);
            xSemaphoreGive(sd_mutex);
            sd_card_health.err_count++;
            return W_FAILURE;
        }
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
    res = lfs_file_open(&g_fs_obj, &file, file_name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL);
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

// commenting this out to prevent accidentally deleting files somehow
// w_status_t sd_card_file_delete(char *file_name) {
//     /* Acquire the mutex. */
//     if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
//         return W_FAILURE;
//     }
//
//     int res = lfs_remove(&g_fs_obj, file_name);
//     xSemaphoreGive(sd_mutex);
//
//     if (res != 0) {
//         return W_FAILURE;
//     }
//     return W_SUCCESS;
// }

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
    if ((resp == HAL_SD_CARD_TRANSFER) && (true == sd_card_health.is_init)) {
        return W_SUCCESS;
    } else {
        return W_FAILURE;
    }
}

uint32_t sd_card_get_status(void) {
    uint32_t status_bitfield = 0;

    if (sd_card_health.is_init == false) {
        status_bitfield |= (1 << E_FS_ERROR_OFFSET);
    }

    // Log operation statistics
    log_text(
        0,
        "sd_card",
        "%s files_created=%lu, reads=%lu, writes=%lu",
        sd_card_health.is_init ? "init" : "not init",
        sd_card_health.file_create_count,
        sd_card_health.read_count,
        sd_card_health.write_count
    );

    return status_bitfield;
}
