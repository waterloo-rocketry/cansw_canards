#ifndef CANSW_CANARDS_WRAPPER_H
#define CANSW_CANARDS_WRAPPER_H
#include "common/common.h"
#include "littlefs/lfs.h"

/**
 * @brief All statuses possible for files on flash chip.
 */
typedef enum {
	FLASH_FILE_NOT_INIT, // only present while file is being opened
	FLASH_FILE_OPEN, // no operations in progress
	FLASH_FILE_WRITING,
	FLASH_FILE_READING,
	FLASH_FILE_WAITING_FOR_SYNC, // set after finished writing to file
	FLASH_FILE_SYNCING,
	FLASH_FILE_CLOSED // if closed, file should not accept any new operations
} flash_file_status_t;

/**
 * @brief File handle type for files (includes littlefs handle and status)
 */
typedef struct {
	flash_file_status_t status;
	lfs_file_t file;
} flash_file_t;

/**
 * @brief Initializes the flash chip logging module
 *
 * Initializes the flash chip and mount it using littlefs.
 *
 * @return w_status_t - W_SUCCESS if initialized and mounted. W_FAILURE if not.
 */
w_status_t flash_init();

/**
 * @brief Open a file
 *
 * @param[in, out] p_file Pointer to file handle
 * @param[in] p_path Filepath to open
 * @param[in] flags Flags to open the file. See littlefs docs for more info
 * @return w_status_t - W_SUCCESS if opened. W_FAILURE in all other cases.
 */
w_status_t flash_file_open(flash_file_t *p_file, const char *p_path, int16_t flags);

/**
 * @brief Close a file
 *
 * @param[in, out] p_file Pointer to file handle
 * @return w_status_t - W_SUCCESS if closed . W_FAILURE in all other cases.
 */
w_status_t flash_file_close(flash_file_t *p_file);

/**
 * @brief Close a file
 *
 * @param[in, out] p_file Pointer to file handle
 * @return w_status_t - W_SUCCESS if synced. W_FAILURE if errored or sync
 * is not needed.
 */
w_status_t flash_file_sync(flash_file_t *p_file);

/**
 * @brief use lfs_file_seek() and lfs_file_read()
 * does not close file
 * does not open file
 * @param[in, out] p_file Pointer to file handle
 * @param[in, out] p_buffer Pointer to the memory buffer that stores the data we want to read
 * @param[in] size number of byte we want to read
 * @param[in] offset number of bytes to move the cursor
 * @param[in] whence reference point of the offset
 * @return W_SUCCESS if read the content
 * W_FAILURE for any error during seek or read
 * W_INVALID_PARAM for parameter errorss
 */
w_status_t flash_file_read(flash_file_t *p_file, void *p_buffer, lfs_size_t size, lfs_soff_t offset,
                           enum lfs_whence_flags whence);
/**
 * @brief removes all files in the format (8 hexadecimal digit).(txt/bin)
 * opens and closes the directory
 * @return W_SUCCESS if all files are removed and the directory is closed
 * W_FAILURE otherwise
 * if 1 log file failed to be removed, will return directly
 */
w_status_t flash_clear_logs();

/**
 * @brief use lfs_file_seek() to set the cursor and use lfs_file_write() to write
 * It will not close the file. (need to call a separate func for that)
 * It will not open the file
 * @param[in, out] p_file Pointer to file handle
 * @param[in, out] p_buffer Pointer to the memory buffer that stores the data we want to write
 * @param[in] size number of byte we want to write
 * @param[in] offset number of bytes to move the cursor
 * @param[in] whence reference point of the offset
 * @return W_FAILURE if file was not open or any error during seek or write
 * W_INVALID_PARAM for parameter issues
 * W_SUCCESS if successful
 */
w_status_t flash_file_write(flash_file_t *p_file, const void *p_buffer, lfs_size_t size,
                            lfs_soff_t offset, enum lfs_whence_flags whence);

lfs_t get_lfs(); // used for testing
#endif // CANSW_CANARDS_WRAPPER_H
