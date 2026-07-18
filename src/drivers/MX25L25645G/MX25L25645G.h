#ifndef MX25L25645G
#define MX25L25645G
#include "littlefs/lfs.h"
#include "octospi.h"
#include "third_party/rocketlib/include/common.h"

w_status_t MX25L25645G_init();

w_status_t MX25L25645G_lfs_mount(lfs_t *lfs,
								 uint32_t first_block_offset);

#endif
