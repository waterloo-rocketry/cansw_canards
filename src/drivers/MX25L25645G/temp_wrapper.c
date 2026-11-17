#include "temp_wrapper.h"

#include "MX25L25645G.h"

static bool mx25l25645g_is_file_open(lfs_t *p_lfs, lfs_file_t *p_file) {
	for (struct lfs_mlist **p = &(p_lfs->mlist); *p; p = &((*p)->next)) {
		if (*p == (struct lfs_mlist *)p_file) {
			return true;
		}
	}

	return false;
}

int mx25l25645g_format(lfs_t *p_lfs, const struct lfs_config *cfg_i) {
	if ((cfg_i->block_count) == 0) {
		return E_LFS_FORMAT_EMPTY;
	}
	return lfs_format(p_lfs, cfg_i);
}

int mx25l25645g_unmount(lfs_t *p_lfs) {
	return lfs_unmount(p_lfs);
}

int mx25l25645g_file_open(lfs_t *p_lfs, lfs_file_t *p_file, const char *p_path,
                          const int16_t flags) {
	if (mx25l25645g_is_file_open(p_lfs, p_file)) {
		return E_LFS_FILE_ALREADY_OPEN;
		// Error code for file already opened
	}
	return lfs_file_open(p_lfs, p_file, p_path, flags);
}

lfs_soff_t mx25l25645g_file_seek(lfs_t *p_lfs, lfs_file_t *p_file, const lfs_soff_t off,
                                 const int whence) {
	if (!(mx25l25645g_is_file_open(p_lfs, p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}

	return lfs_file_seek(p_lfs, p_file, off, whence);
}

lfs_ssize_t mx25l25645g_file_read(lfs_t *p_lfs, lfs_file_t *p_file, void *p_buffer,
                                  const lfs_size_t size, const lfs_soff_t offset,
                                  const enum lfs_whence_flags whence) {
	if (!(mx25l25645g_is_file_open(p_lfs, p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}
	if (LFS_O_RDONLY != (LFS_O_RDONLY & (p_file->flags))) {
		return LFS_ERR_INVAL;
	}
	const int err = lfs_file_seek(p_lfs, p_file, offset, whence);
	if (0 > err) {
		return err;
	}
	return lfs_file_read(p_lfs, p_file, p_buffer, size);
}

lfs_ssize_t mx25l25645g_file_write(lfs_t *p_lfs, lfs_file_t *p_file, const void *p_buffer,
                                   const lfs_size_t size) {
	if (!(mx25l25645g_is_file_open(p_lfs, p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}
	if (LFS_O_WRONLY != (LFS_O_WRONLY & (p_file->flags))) {
		return LFS_ERR_INVAL;
	}
	return lfs_file_write(p_lfs, p_file, p_buffer, size);
}

int mx25l25645g_file_close(lfs_t *p_lfs, lfs_file_t *p_file) {
	if (!(mx25l25645g_is_file_open(p_lfs, p_file))) {
		return E_LFS_FILE_NOT_OPEN;
		// Error code accessing a file that isn't open.
	}
	return lfs_file_close(p_lfs, p_file);
}

int mx25l25645g_file_truncate(lfs_t *p_lfs, lfs_file_t *p_file, lfs_off_t size) {
	if (!(mx25l25645g_is_file_open(p_lfs, p_file))) {
		return E_LFS_FILE_NOT_OPEN;
	}
	if (LFS_O_WRONLY != (LFS_O_WRONLY & (p_file->flags))) {
		return LFS_ERR_INVAL;
	}
	return lfs_file_truncate(p_lfs, p_file, size);
}
