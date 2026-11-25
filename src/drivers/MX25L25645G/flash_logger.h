#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H
#include "common/common.h"
#include "littlefs/lfs.h"

#define MAX_FILE_NAME (16)

/**
 * @brief All statuses possible for files on flash chip.
 */
typedef enum {
	FLASH_FILE_NOT_INIT, // only present while file is being opened
	FLASH_FILE_OPENING,
	FLASH_FILE_OPEN, // no operations in progress
	FLASH_FILE_WRITING,
	FLASH_FILE_READING,
	FLASH_FILE_SYNCING,
	FLASH_FILE_CLOSED // if closed, file should not accept any new operations
} flash_file_status_t;

/**
 * @brief File handle type for files (includes littlefs handle and status)
 */
typedef struct {
	flash_file_status_t status;
	lfs_file_t file;
	bool waiting_for_sync;
	uint16_t errors; // counter for errors occurring on file
	char name[MAX_FILE_NAME];
	int16_t flags;
	bool reopened_last; // tracker for if the file was reopened last time it reached the threshold
	                    // for reopening
} flash_file_t;

/**
 * @brief Initializes the flash chip logging module
 *
 * Initializes the flash chip and mount it using littlefs.
 * *
 * @return w_status_t - W_SUCCESS if initialized and mounted. W_FAILURE if not.
 */
w_status_t flash_init();

/**
 * @brief Open a file
 *
 * @param[in, out] p_file Pointer to file handle
 * @param[in] p_path Filepath to open
 * @param[in] flags Flags to open the file. See littlefs docs for more info
 *
 * @return w_status_t - W_SUCCESS if opened. W_INVALID_PARAM if file or filename
 * passed is NULL or if file status says file should not be opened. W_FAILURE in all other cases.
 */
w_status_t flash_file_open(flash_file_t *p_file, const char *p_path, int16_t flags);

/**
 * @brief Close a file
 *
 * @param[in, out] p_file Pointer to file handle
 *
 * @return w_status_t - W_SUCCESS if closed. W_INVALID_PARAM if file passed is
 * NULL. W_FAILURE in all other cases.
 */
w_status_t flash_file_close(flash_file_t *p_file);

/**
 * @brief Close a file
 *
 * @param[in, out] p_file Pointer to file handle
 *
 * @return w_status_t - W_SUCCESS if synced. W_FAILURE if errored. W_NO_DATA
 * if sync is not needed. W_INVALID_PARAM if file passed is NULL.
 */
w_status_t flash_file_sync(flash_file_t *p_file);

/**
 * @brief use lfs_file_seek() to set the cursor and use lfs_file_write()
 * to write
 *
 * It will not close the file. (need to call a separate func for that)
 * It will not open the file
 *
 * @param[in, out] p_file Pointer to file handle
 * @param[in, out] p_buffer Pointer to the memory buffer that stores the data we want to write
 * @param[in] size number of byte we want to write
 * @param[in] offset number of bytes to move the cursor
 * @param[in] whence reference point of the offset
 *
 * @return W_FAILURE if file was not open or any error during seek or write
 * W_INVALID_PARAM for parameter issues
 * W_SUCCESS if successful
 */
w_status_t flash_file_write(flash_file_t *p_file, const void *p_buffer, lfs_size_t size,
                            lfs_soff_t offset, enum lfs_whence_flags whence);

/**
 * @brief use lfs_file_seek() and lfs_file_read()
 *
 * does not open or close file
 *
 * @param[in, out] p_file Pointer to file handle
 * @param[in, out] p_buffer Pointer to the memory buffer that stores the data we want to read
 * @param[in] size number of byte we want to read
 * @param[in] offset number of bytes to move the cursor
 * @param[in] whence reference point of the offset
 * @param[in] p_read_size size of what has actually been read from file. Needed
 * to detect outdated data in buffer if size of buffer is longer than read size.
 *
 * @return W_SUCCESS if read the content
 * W_FAILURE for any error during seek or read
 * W_INVALID_PARAM for parameter errorss
 */
w_status_t flash_file_read(flash_file_t *p_file, void *p_buffer, lfs_size_t size, lfs_soff_t offset,
                           enum lfs_whence_flags whence, lfs_size_t *p_read_size);

/**
 * @brief removes all files in the format (8 hexadecimal digit).(txt/bin)
 *
 * opens and closes the root directory and scans all files inside it
 *
 * @return W_SUCCESS if all files are removed and the directory is closed
 * W_FAILURE otherwise if 1 log file failed to be removed, will return directly
 */
w_status_t flash_clear_logs();

/**
 * @brief Reformat the file system
 *
 * Attempts to close all files before formatting. Errors when a file cannot be
 * closed successfully, unless force true is passed.
 *
 * @param force If an error in closing files should be ignored and the file
 * system reformated anyway
 *
 * @return w_status_t W_SUCCESS on successful reformat, W_FAILURE otherwise.
 */
w_status_t flash_reformat(bool force);

#endif // FLASH_LOGGER_H
