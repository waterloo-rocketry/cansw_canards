#include "rocketlib/include/common.h"
#include "drivers/timer/timer.h"
#include "littlefs/lfs.h"


w_status_t MX25L25645G_init() {

}
// Read a region in a block. Negative error codes are propagated
// to the user.
int MX25L25645G_lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int MX25L25645G_lfs_prg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {

}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int MX25L25645G_lfs_erase(const struct lfs_config *c, lfs_block_t block) {

}

// Sync the state of the underlying block device. Negative error codes
// are propagated to the user.
int MX25L25645G_lfs_sync(const struct lfs_config *c) {
    return LFS_ERR_OK;
}


// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
	// block device operations
	.read = MX25L25645G_lfs_read,
	.prog = MX25L25645G_lfs_prg,
	.erase = MX25L25645G_lfs_erase,
	.sync = MX25L25645G_lfs_sync,

	// block device configuration
	.read_size = 512,
	.prog_size = 512,
	.block_size = 512,
	.block_count = 0,
	.block_cycles = 500,
	.cache_size = 512,
	.lookahead_size = 512,
	.compact_thresh = -1,
	.name_max = 0,
	.file_max = 0,
	.attr_max = 0,
	.metadata_max = 0,
	.inline_max = -1
};