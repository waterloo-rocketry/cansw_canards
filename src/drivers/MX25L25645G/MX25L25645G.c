#include "rocketlib/include/common.h"
#include "drivers/timer/timer.h"
#include "littlefs/lfs.h"
#include "drivers/gpio/gpio.h"
#include "octospi.h"

static const OSPI_HandleTypeDef *FLASH_OSPI = &hospi1;
static const uint32_t FLASH_DELAY_MS = 100;
static const gpio_pin_t NCS_PIN = GPIO_PIN_FLASH_CS;


w_status_t MX25L25645G_init() {

}

static bool check_flash_complete() {
	while ();
}

// complete a WREN
static void send_wren(bool select_ncs) {

	// select chip
	if (select_ncs) {
		gpio_write(NCS_PIN, GPIO_LEVEL_LOW, 0);
	}

	OSPI_RegularCmdTypeDef cmd_wren = {0};

	cmd_wren.Instruction = 0x06; // WREN
	cmd_wren.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_wren.AddressMode = HAL_OSPI_ADDRESS_NONE;
	cmd_wren.DataMode = HAL_OSPI_DATA_NONE;
	cmd_wren.DummyCycles = 0;
	cmd_wren.NbData = 0;

	HAL_StatusTypeDef hal_status = HAL_OSPI_Command(FLASH_OSPI, &cmd_wren, FLASH_DELAY_MS);

	// make sure command is fully sent
	while (__HAL_OSPI_GET_FLAG(&hospi1, HAL_OSPI_FLAG_BUSY) != RESET);
	// unselect chip
	if (select_ncs) {
		gpio_write(NCS_PIN, GPIO_LEVEL_HIGH, 0);
	}
}
// Read a region in a block. Negative error codes are propagated
// to the user.
static int MX25L25645G_lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    uint32_t start_address = block * c->block_size + off;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int MX25L25645G_lfs_prg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    uint32_t start_address = block * c->block_size + off;

	uint32_t overflow_amount = 0;
    // overflows page
    if (((start_address % c->prog_size) + size) > c->prog_size) {
    	uint32_t overflow_amount = (start_address + size) % c->prog_size;
	}



}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int MX25L25645G_lfs_erase(const struct lfs_config *c, lfs_block_t block) {

}

// Sync the state of the underlying block device. Negative error codes
// are propagated to the user.
static int MX25L25645G_lfs_sync(const struct lfs_config *c) {
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
	.read_size = 16,
	.prog_size = 256,
	.block_size = 4096,
	.block_count = 0,
	.block_cycles = 500,
	.cache_size = 2048,
	.lookahead_size = 2048 / 8,
	.compact_thresh = -1,
	.name_max = 0,
	.file_max = 0,
	.attr_max = 0,
	.metadata_max = 0,
	.inline_max = -1
};