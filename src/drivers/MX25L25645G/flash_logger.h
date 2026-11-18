#ifndef CANSW_CANARDS_WRAPPER_H
#define CANSW_CANARDS_WRAPPER_H
#include "common/common.h"
#include "littlefs/lfs.h"

w_status_t flash_init(uint32_t first_block_offset);

w_status_t flash_clear_logs();

#endif // CANSW_CANARDS_WRAPPER_H
