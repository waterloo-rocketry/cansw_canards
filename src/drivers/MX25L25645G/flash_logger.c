#include "flash_logger.h"

#include "MX25L25645G.h"
#include <ctype.h>

#define MAX_NUM_FILES (8)

// TODO: set actual values for these constants, right now their just vibes

// Percent of files that need to be in an error state for the file system
// to remount (in percentage points). 0 to never remount.
#define THRESHOLD_FOR_REMOUNT (100)

// Amount of errors that must stack up on a file for it to be reopened. 0 to never reopen.
#define THRESHOLD_FOR_FILE_REOPEN (100)
// Amount of errors that must stack up on a file for it to be considered in an error state
// Must be smaller than THRESHOLD_FOR_FILE_REOPEN for reopen to always be attempted before more
// drastic measures
#define THRESHOLD_FILE_IN_ERROR (200)
static_assert(THRESHOLD_FOR_FILE_REOPEN < THRESHOLD_FILE_IN_ERROR,
              "Flash logger: file reopening must come before a file is considered to "
              "be in an error state");

#define VAL_DECREMENT_ON_REOPEN (70)

static lfs_t lfs;

static flash_file_t *open_files[MAX_NUM_FILES] = {NULL};
static uint8_t num_open_files = 0;

static bool is_init = false;

/**
 * @brief Checks if a certain file is considered to be a log file or not.
 *
 * Log files are all files in for XXXXXXXX.txt or XXXXXXXX.bin, where XXXXXXXX
 * is a hexadecimal number.
 *
 * @param filename name of file to check if is a log file
 * @return true if is a log file, false otherwise.
 */
static bool is_log_file(const char *filename) {
	if ((NULL == filename) ||
	    (12 != strlen(filename))) { // filename is "xxxxxxxx.txt" 8 hexadecimal value + 4 chars that
	                                // are either ".txt" or ".bin"
		return false;
	}
	for (int8_t filename_char_i = 0; filename_char_i < 8; filename_char_i++) {
		if (!isxdigit((unsigned char)filename[filename_char_i])) {
			return false;
		}
	}
	return ((0 == strcmp(filename + 8, ".txt")) || (0 == strcmp(filename + 8, ".bin")));
}

/**
 * @brief Remount the file system if too many errors occur
 *
 * This doesn't do a lot of error checking since it is intended to only be run
 * in cases when most files are not writable/readable, i.e. when the system is
 * in a state of "full" error. In a sense, this force remounts the file system,
 * while hopefully closing files. Possible data loss.
 * W_FAILURE is returned only when unmounting fails, which should not be possible.
 * Reopens open files after remounting.
 *
 * @return w_status_t from flash_init
 */
static w_status_t flash_remount() {
	for (uint8_t file_i = 0; file_i < MAX_NUM_FILES; file_i++) {
		if (NULL != open_files[file_i]) {
			lfs_file_close(&lfs, &(open_files[file_i]->file));
			(open_files[file_i]->status) = FLASH_FILE_CLOSED;
			(open_files[file_i]->waiting_for_sync) = false; // closing a file syncs it
		}
	}
	if (LFS_ERR_OK != lfs_unmount(&lfs)) {
		return W_FAILURE;
	}
	flash_init();
	for (uint8_t file_i = 0; file_i < MAX_NUM_FILES; file_i++) {
		if (NULL != open_files[file_i]) {
			lfs_file_open(&lfs, &(open_files[file_i]->file), (open_files[file_i]->name),
			              (open_files[file_i]->flags));
			(open_files[file_i]->status) = FLASH_FILE_OPEN;
			(open_files[file_i]->errors) = 0;
		}
	}
	return W_SUCCESS;
}

/**
 * @brief Handle error on file operation
 *
 * At and after THRESHOLD_FOR_FILE_REOPEN, reopens the file.
 * At and after THRESHOLD_FILE_IN_ERROR, remounts the file system.
 * Decrements error when a possible solution has been found.
 *
 * Increments error by 2 if no action was taken, otherwise only by 1. This
 *
 * @param p_file File that caused the error to occur
 *
 * @return w_status_t W_FAILRE if remount fails. W_SUCCESS otherwise
 */
static w_status_t handle_error(flash_file_t *p_file) {
	(p_file->errors) += 1;

	if (THRESHOLD_FOR_FILE_REOPEN != 0) {
		if (THRESHOLD_FOR_FILE_REOPEN <= (p_file->errors)) {
			// This doesn't use the open/close functions declared in this file
			// to avoid an infinite loop of errors.
			if (!(p_file->reopened_last)) {
				lfs_file_close(&lfs, &(p_file->file));
				lfs_file_open(&lfs, &(p_file->file), (p_file->name), (p_file->flags));
				(p_file->reopened_last) = true;
				(p_file->errors) -= VAL_DECREMENT_ON_REOPEN;
			} else {
				(p_file->reopened_last) = false;
			}
		}
	}

	if (0 != THRESHOLD_FOR_REMOUNT) {
		uint8_t num_files_in_error = 0;
		for (uint8_t file_i = 0; MAX_NUM_FILES > file_i; file_i++) {
			if (NULL != open_files[file_i]) {
				if (THRESHOLD_FILE_IN_ERROR <= open_files[file_i]->errors) {
					num_files_in_error++;
				}
			}
		}
		// Check if percent of files in error is >= than threshold.
		// Integer division is intended.
		if (THRESHOLD_FOR_REMOUNT <= ((num_files_in_error * 100) / num_open_files)) {
			return flash_remount();
		}
	}

	return W_SUCCESS;
}

w_status_t flash_init() {
	// already_init = no need to check that initialization works
	if (!is_init) {
		if (W_SUCCESS != mx25l25645g_init()) {
			return W_FAILURE;
		} else {
			is_init = true;
		}
	}
	return mx25l25645g_mount(&lfs);
}

w_status_t flash_file_open(flash_file_t *p_file, const char *p_path, const int16_t flags) {
	if ((NULL == p_file) || (NULL == p_path)) {
		return W_INVALID_PARAM;
	}

	switch (p_file->status) {
		// Intended to fall through since a closed file would already be
		// in the open_files array and should not be readded.
		case FLASH_FILE_NOT_INIT:
			if (MAX_NUM_FILES <= 1 + num_open_files) {
				return W_FAILURE;
			}
			for (uint8_t file_i = 0; file_i < MAX_NUM_FILES; file_i++) {
				if (NULL == open_files[file_i]) {
					open_files[num_open_files] = p_file;
					num_open_files++;
					strcpy(p_file->name, p_path);
					(p_file->flags) = flags;
					(p_file->reopened_last) = false;
					break;
				}
			}
			__attribute__((fallthrough));
		case FLASH_FILE_CLOSED:
			(p_file->status) = FLASH_FILE_OPENING;
			if (LFS_ERR_OK != lfs_file_open(&lfs, &(p_file->file), p_path, flags)) {
				handle_error(p_file);
				return W_FAILURE;
			}

			(p_file->status) = FLASH_FILE_OPEN;
			return W_SUCCESS;
		default:
			return W_FAILURE;
	}
}

w_status_t flash_file_close(flash_file_t *p_file) {
	if ((NULL == p_file)) {
		return W_INVALID_PARAM;
	}

	// If file needs syncing, sync before closing. This is done
	// automatically by lfs_file_close(), so this is more of a precaution.
	if (true == (p_file->waiting_for_sync)) {
		(p_file->status) = FLASH_FILE_SYNCING;
		if (LFS_ERR_OK != lfs_file_sync(&lfs, &(p_file->file))) {
			handle_error(p_file);
			return W_FAILURE;
		}
		(p_file->waiting_for_sync) = false;
		(p_file->status) = FLASH_FILE_OPEN;
	}

	// Only close files that aren't currently in use.
	if (FLASH_FILE_OPEN != (p_file->status)) {
		return W_FAILURE;
	}

	if (LFS_ERR_OK != lfs_file_close(&lfs, &(p_file->file))) {
		(p_file->status) = FLASH_FILE_CLOSED;
		return W_FAILURE;
	}
	for (uint8_t file_i = 0; file_i < MAX_NUM_FILES; file_i++) {
		if ((open_files[file_i]) == (p_file)) {
			open_files[file_i] = NULL;
			num_open_files -= 1;
			break;
		}
	}
	(p_file->status) = FLASH_FILE_CLOSED;
	return W_SUCCESS;
}

w_status_t flash_file_sync(flash_file_t *p_file) {
	if ((NULL == p_file)) {
		return W_INVALID_PARAM;
	}

	// Only sync files that need syncing.
	if (true == (p_file->waiting_for_sync)) {
		// Try reopening a closed file before syncing
		if (FLASH_FILE_CLOSED == (p_file->status)) {
			if (W_SUCCESS != flash_file_open(p_file, p_file->name, p_file->flags)) {
				return W_FAILURE;
			}
		}

		(p_file->status) = FLASH_FILE_SYNCING;
		if (0 != lfs_file_sync(&lfs, &(p_file->file))) {
			handle_error(p_file);
			return W_FAILURE;
		}
		(p_file->status) = FLASH_FILE_OPEN;
		(p_file->waiting_for_sync) = false;
	} else {
		return W_NO_DATA;
	}

	return W_SUCCESS;
}

w_status_t flash_file_write(flash_file_t *p_file, const void *p_buffer, lfs_size_t size,
                            lfs_soff_t offset, enum lfs_whence_flags whence) {
	if ((NULL == p_file) || ((NULL == p_buffer) && (0 != size))) {
		return W_INVALID_PARAM;
	}
	if (LFS_O_WRONLY != (LFS_O_WRONLY & (p_file->flags))) {
		return W_INVALID_PARAM;
	}

	// Try reopening a closed file before writing
	if (FLASH_FILE_CLOSED == (p_file->status)) {
		if (W_SUCCESS != flash_file_open(p_file, p_file->name, p_file->flags)) {
			return W_FAILURE;
		}
	}

	(p_file->status) = FLASH_FILE_WRITING;
	const int32_t err_seek =
		lfs_file_seek(&lfs, &(p_file->file), offset, whence); // sets the cursor
	if (LFS_ERR_OK > err_seek) {
		handle_error(p_file);
		return W_FAILURE;
	}

	const int32_t res_write = lfs_file_write(&lfs, &(p_file->file), p_buffer, size);
	if (LFS_ERR_OK > res_write) {
		handle_error(p_file);
		return W_FAILURE;
	}
	if (((uint32_t)res_write) != size) {
		handle_error(p_file);
		return W_FAILURE;
	}
	(p_file->status) = FLASH_FILE_OPEN;
	(p_file->waiting_for_sync) = true;
	return W_SUCCESS;
}

w_status_t flash_file_read(flash_file_t *p_file, void *p_buffer, lfs_size_t size, lfs_soff_t offset,
                           enum lfs_whence_flags whence, lfs_size_t *p_read_size) {
	if ((NULL == p_file) || ((0 == size))) {
		return W_INVALID_PARAM;
	}
	if (LFS_O_RDONLY != (LFS_O_RDONLY & (p_file->flags))) {
		return W_INVALID_PARAM;
	}

	// Try reopening a closed file before reading
	if (FLASH_FILE_CLOSED == (p_file->status)) {
		if (W_SUCCESS != flash_file_open(p_file, p_file->name, p_file->flags)) {
			return W_FAILURE;
		}
	}

	(p_file->status) = FLASH_FILE_READING;
	const int32_t err_seek =
		lfs_file_seek(&lfs, &(p_file->file), offset, whence); // sets the cursor
	if (LFS_ERR_OK > err_seek) {
		handle_error(p_file);
		return W_FAILURE;
	}

	const int32_t res_read = lfs_file_read(&lfs, &(p_file->file), p_buffer, size);
	if (LFS_ERR_OK > res_read) {
		handle_error(p_file);
		return W_FAILURE;
	}
	(*p_read_size) = res_read;
	(p_file->status) = FLASH_FILE_OPEN;
	return W_SUCCESS;
}

w_status_t flash_clear_logs() {
	lfs_dir_t dir;
	struct lfs_info info;
	int32_t err = lfs_dir_open(&lfs, &dir, "/"); // need to iterate over log files
	// assumed everything is in the root directory
	if (LFS_ERR_OK > err) {
		return W_FAILURE;
	}
	// Iterate over all entries in a directory (end signified by 0).
	while ((err = lfs_dir_read(&lfs, &dir, &info)) > 0) {
		if ((LFS_TYPE_REG == (info.type)) && (is_log_file(info.name))) {
			err = lfs_remove(&lfs, (info.name));
			if (LFS_ERR_OK > err) {
				lfs_dir_close(&lfs, &dir);
				return W_FAILURE;
			}
		}
	}
	const int32_t err1 = lfs_dir_close(&lfs, &dir);
	if ((LFS_ERR_OK > err) || (LFS_ERR_OK > err1)) {
		return W_FAILURE;
	}
	return W_SUCCESS;
}

w_status_t flash_reformat(bool force) {
	// Close all open files before formatting
	for (uint8_t file_i = 0; MAX_NUM_FILES > file_i; file_i++) {
		if (NULL != open_files[file_i]) {
			int8_t err = flash_file_close(open_files[file_i]);
			if (LFS_ERR_OK != err) {
				if (!force) {
					return W_FAILURE;
				} else {
					// File is marked as closed anyway for
					// opening to be attempted on next
					// read/write/sync after formatting
					// since we cannot assume that existing
					// file handles will remain valid after
					// reformat.
					open_files[file_i]->status = FLASH_FILE_CLOSED;
				}
			}
		}
	}
	if (LFS_ERR_OK != lfs_format(&lfs, lfs.cfg)) {
		return W_FAILURE;
	}

	return flash_init();
}
