#ifndef FS_H
#define FS_H

#include "rocketlib/include/common.h"

w_status_t fs_init(void);
void fs_write_page(const uint8_t *page);
w_status_t status_report(void);
uint32_t fs_get_log_written_size(void);
uint32_t fs_get_sd_log_file_name(void);
uint32_t fs_get_sd_used(void);
uint32_t fs_get_sd_free(void);

#endif