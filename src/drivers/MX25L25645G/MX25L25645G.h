#ifndef MX25L25645G
#define MX25L25645G
#include "littlefs/lfs.h"
#include "third_party/rocketlib/common/common.h"

w_status_t mx25l25645g_init();

w_status_t mx25l25645g_mount(lfs_t *p_lfs, uint32_t first_block_offset);

#endif
