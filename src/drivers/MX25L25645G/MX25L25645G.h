#ifndef MX25L25645G
#define MX25L25645G
#include "littlefs/lfs.h"
#include "third_party/rocketlib/common/common.h"

enum mx25l25645g_extra_lfs_error_t {
	E_LFS_FILE_ALREADY_OPEN = -10,
	E_LFS_FILE_NOT_OPEN = -11,
	E_LFS_FORMAT_EMPTY = -13
};

w_status_t mx25l25645g_init();

w_status_t mx25l25645g_mount(lfs_t *p_lfs, uint32_t first_block_offset);

#endif
