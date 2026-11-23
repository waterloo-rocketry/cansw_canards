#include "flash_logger.h"

#include "MX25L25645G.h"
#include <ctype.h>

static lfs_t lfs;

w_status_t flash_init() {
	if (W_SUCCESS != mx25l25645g_init()) {
		return W_FAILURE;
	}
	return mx25l25645g_mount(&lfs);
}

w_status_t flash_file_open(flash_file_t *p_file, const char *p_path, const int16_t flags) {
	if (FLASH_FILE_CLOSED == (p_file->status)) {
		if (0 != lfs_file_open(&lfs, &(p_file->file), p_path, flags)) {
			return W_FAILURE;
		}
		(p_file->status) = FLASH_FILE_OPEN;
		return W_SUCCESS;
	}

	(p_file->status) = FLASH_FILE_NOT_INIT;

	if (0 != lfs_file_open(&lfs, &(p_file->file), p_path, flags)) {
		return W_FAILURE;
	}

	(p_file->status) = FLASH_FILE_OPEN;
	return W_SUCCESS;
}

w_status_t flash_file_close(flash_file_t *p_file) {
	// If file needs syncing, sync before closing. This is done
	// automatically by lfs_file_close(), so this is more of a precaution.
	if (FLASH_FILE_WAITING_FOR_SYNC == (p_file->status)) {
		if (0 != lfs_file_sync(&lfs, &(p_file->file))) {
			return W_FAILURE;
		}
		p_file->status = FLASH_FILE_OPEN;
	}

	// Only close files that aren't currently in use.
	if (FLASH_FILE_OPEN != (p_file->status)) {
		return W_FAILURE;
	}

	if (0 != lfs_file_close(&lfs, &(p_file->file))) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

w_status_t flash_file_sync(flash_file_t *p_file) {
	// Only sync files that need syncing.
	if (FLASH_FILE_WAITING_FOR_SYNC == (p_file->status)) {
		if (0 != lfs_file_sync(&lfs, &(p_file->file))) {
			return W_FAILURE;
		}
	} else {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

static bool is_log_file(const char *filename) {
	if (filename == NULL || strlen(filename) != 12) {
		return false;
	}
	for (int i = 0; i < 8; i++) {
		if (!isxdigit((unsigned char)filename[i])) {
			return false;
		}
	}
	return (strcmp(filename + 8, ".txt") == 0 || strcmp(filename + 8, ".bin") == 0);
}

w_status_t flash_clear_logs() {
	lfs_dir_t dir;
	struct lfs_info info;
	int err = lfs_dir_open(&lfs, &dir, "/"); // need to iterate over log files
	// assumed everything is in the root directory
	if (err < 0) {
		return W_FAILURE;
	}
	while ((err = lfs_dir_read(&lfs, &dir, &info)) > 0) {
		if ((info.type == LFS_TYPE_REG) && (is_log_file(info.name))) {
			err = lfs_remove(&lfs, info.name);
			if (err < 0) {
				lfs_dir_close(&lfs, &dir);
				return W_FAILURE;
			}
		}
	}
	int err1 = lfs_dir_close(&lfs, &dir);
	if (err < 0 || err1 < 0) {
		return W_FAILURE;
	}
	return W_SUCCESS;
}

w_status_t flash_file_write(flash_file_t *p_file, const void *p_buffer, lfs_size_t size,
                            lfs_soff_t offset, enum lfs_whence_flags whence) {
	if (p_file == NULL || (p_buffer == NULL && size != 0)) {
		return W_INVALID_PARAM;
	}
	if (LFS_O_WRONLY != (LFS_O_WRONLY & ((p_file->file).flags))) {
		return W_INVALID_PARAM;
	}
	int err = lfs_file_seek(&lfs, &(p_file->file), offset, whence); // sets the cursor
	if (err < 0) {
		return W_FAILURE;
	}
	err = lfs_file_write(&lfs, &(p_file->file), p_buffer, size);
	if (err < 0 || err != size) {
		return W_FAILURE;
	}
	p_file->status = FLASH_FILE_WAITING_FOR_SYNC;
	return W_SUCCESS;
}

w_status_t flash_file_read(flash_file_t *p_file, void *p_buffer, lfs_size_t size, lfs_soff_t offset,
                           enum lfs_whence_flags whence) {
	if (p_file == NULL || (p_buffer == NULL && size != 0)) {
		return W_INVALID_PARAM;
	}
	if (LFS_O_RDONLY != (LFS_O_RDONLY & ((p_file->file).flags))) {
		return W_INVALID_PARAM;
	}
	int err = lfs_file_seek(&lfs, &(p_file->file), offset, whence); // sets the cursor
	if (err < 0) {
		return W_FAILURE;
	}
	err = lfs_file_read(&lfs, &(p_file->file), p_buffer, size);
	if (err == size) {
		return W_SUCCESS;
	}
	return W_FAILURE;
}
