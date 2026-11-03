#ifndef MX25L25645G
#define MX25L25645G
#include "littlefs/lfs.h"
#include "octospi.h"
#include "third_party/rocketlib/common/common.h"

enum mx25l25645g_extra_lfs_error_t
{
    E_LFS_FILE_ALREADY_OPEN = -10,
    E_LFS_FILE_NOT_OPEN = -11,
    E_LFS_FORMAT_EMPTY = -13,
};


/// General flash functions
w_status_t mx25l25645g_init();

w_status_t mx25l25645g_mount(lfs_t * p_lfs, uint32_t first_block_offset);

int mx25l25645g_format(lfs_t * p_lfs, const struct lfs_config * cfg_i);

int mx25l25645g_unmount(lfs_t * p_lfs);


/// File functions
int mx25l25645g_file_open(lfs_t * p_lfs, lfs_file_t * p_file,
                          const char * p_path,
                          int16_t flags);

lfs_ssize_t mx25l25645g_file_read(lfs_t * p_lfs, lfs_file_t * p_file,
                                  void * p_buffer,
                                  lfs_size_t size, lfs_soff_t offset,
                                  enum lfs_whence_flags whence);

lfs_ssize_t mx25l25645g_file_write(lfs_t * p_lfs, lfs_file_t * p_file,
                                   const void * p_buffer,
                                   lfs_size_t size);

lfs_soff_t mx25l25645g_file_seek(lfs_t * p_lfs, lfs_file_t * p_file,
                                 lfs_soff_t off, int whence);

int mx25l25645g_file_close(lfs_t * p_lfs, lfs_file_t * p_file);

int mx25l25645g_file_truncate(lfs_t * p_lfs, lfs_file_t * p_file,
                              lfs_off_t size);

#endif
