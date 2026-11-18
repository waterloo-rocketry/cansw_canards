#include "flash_logger.h"

#include "MX25L25645G.h"
#include <ctype.h>

// TODO: To be removed. Only here temporarily for everything to compile
typedef enum {
	E_LFS_FILE_ALREADY_OPEN = -10,
	E_LFS_FILE_NOT_OPEN = -11,
	E_LFS_FORMAT_EMPTY = -13
} FLASH_EXTRA_LFS_ERRORS_T;

typedef enum {
	FILE_NOT_INIT, // only present while file is being opened
	FILE_OPEN, // no operations in progress
	FILE_WRITING,
	FILE_READING,
	FILE_WAITING_FOR_SYNC, // set after finished writing to file
	FILE_SYNCING,
	FILE_CLOSED // if closed, file should not accept any new operations
} flash_file_status_t;

typedef struct {
	lfs_file_t file;
	flash_file_status_t status;
} flash_file_t;

static lfs_t lfs;

static bool flash_is_file_open(flash_file_t *p_file) {
	for (struct lfs_mlist **p = (&(lfs.mlist)); *p; p = (&((*p)->next))) {
		if (*p == (struct lfs_mlist *)(&(p_file->file))) {
			return true;
		}
	}
	return false;
}

w_status_t flash_init(const uint32_t first_block_offset) {
	return mx25l25645g_mount(&lfs, first_block_offset); // off is usually 0
}

int mx25l25645g_format(const struct lfs_config *cfg_i) {
	if ((cfg_i->block_count) == 0) {
		return E_LFS_FORMAT_EMPTY;
	}
	return lfs_format(&lfs, cfg_i);
}

int mx25l25645g_unmount(lfs_t *p_lfs) {
	return lfs_unmount(p_lfs);
}

int mx25l25645g_file_open(flash_file_t *p_file, const char *p_path, const int16_t flags) {
	if (flash_is_file_open(p_file)) {
		return E_LFS_FILE_ALREADY_OPEN;
		// Error code for file already opened
	}
	return lfs_file_open(&lfs, &(p_file->file), p_path, flags);
}

lfs_soff_t mx25l25645g_file_seek(flash_file_t *p_file, const lfs_soff_t off, const int whence) {
	if (!(flash_is_file_open(p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}

	return lfs_file_seek(&lfs, &(p_file->file), off, whence);
}

lfs_ssize_t mx25l25645g_file_read(flash_file_t *p_file, void *p_buffer, const lfs_size_t size,
                                  const lfs_soff_t offset, const enum lfs_whence_flags whence) {
	if (!(flash_is_file_open(p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}
	if (LFS_O_RDONLY != (LFS_O_RDONLY & ((p_file->file).flags))) {
		return LFS_ERR_INVAL;
	}
	const int err = lfs_file_seek(&lfs, &(p_file->file), offset, whence);
	if (0 > err) {
		return err;
	}
	return lfs_file_read(&lfs, &(p_file->file), p_buffer, size);
}

lfs_ssize_t mx25l25645g_file_write(flash_file_t *p_file, const void *p_buffer,
                                   const lfs_size_t size) {
	if (!(flash_is_file_open(p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}
	if (LFS_O_WRONLY != (LFS_O_WRONLY & ((p_file->file).flags))) {
		return LFS_ERR_INVAL;
	}
	return lfs_file_write(&lfs, &(p_file->file), p_buffer, size);
}

int mx25l25645g_file_close(flash_file_t *p_file) {
	if (!(flash_is_file_open(p_file))) {
		return E_LFS_FILE_NOT_OPEN;
		// Error code accessing a file that isn't open.
	}
	return lfs_file_close(&lfs, &(p_file->file));
}

static bool is_log_file(const char *filename) {
	if (strlen(filename) != 12) {
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
