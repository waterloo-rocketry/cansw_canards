#ifndef MX25L25645G_H
#define MX25L25645G_H
#include "littlefs/lfs.h"
#include "third_party/rocketlib/common/common.h"

/**
 * @brief Initialize flash chip.
 *
 * Enables quad reads & writes to chip for faster communication. Enables 4 byte
 * addresses instead of the default 3 bytes due to large size of flash chip.
 *
 * @return w_status_t - W_SUCCESS on success. W_IO_ERROR if error in
 * communicating with the flash chip. W_FAILURE if already init or flash chip
 * registers are misconfigured.
 */
w_status_t mx25l25645g_init();

/**
 * @brief Mount flash chip using littlefs file system.
 *
 * Mounts the entire flash as a littlefs file system. On a failed mount, will
 * try to reformat the flash chip.
 *
 * @param[out] p_lfs Pointer to file system.
 *
 * @return w_status_t - W_SUCCESS on success. W_FAILURE if already init or
 * mount&format fails. W_INVALID_PARAM if block offset is invalid.
 */
w_status_t mx25l25645g_mount(lfs_t *p_lfs);

#endif
