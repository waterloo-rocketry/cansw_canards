#include "drivers/MX25L25645G/MX25L25645G.h"
#include "drivers/gpio/gpio.h"
#include "drivers/timer/timer.h"
#include "littlefs/lfs.h"
#include "octospi.h"
#include "third_party/rocketlib/include/common.h"

#define SPI_4_ENABLED // so 1 - (1 or 4) - 4

// TODO: add INIT Checks
static const uint32_t FLASH_SINGLE_DELAY_MS =
	100; // this repersent the maximum amount of delay for a single operation
static const uint32_t MAX_RDStatus_ATTEMPTS = 2;
static const uint16_t WEL_BIT_MASK = 0x2;
static const uint16_t WIP_BIT_MASK = 0x1;
static const uint16_t P_FAIL_BIT_MASK = 0x20;
static const uint16_t E_FAIL_BIT_MASK = 0x40;

// make sure cache size is always less than 2^16 times of Page size
static const uint16_t CACHE_SIZE = 2048;
static const uint16_t PAGE_SIZE = 256;
static const uint16_t SECTOR_SIZE = 4096;

static const uint32_t PROG_US_PER_BYTE = 15;
static const uint32_t MS_PER_US = 1000;
static const uint32_t FLASH_PROG_DELAY_MS = 100;

static const uint32_t flash_read_dummy_cycle = 8;

typedef enum {
	FLASH_SPI_NOT_INIT,
	FLASH_SPI_READY,
	FLASH_SPI_COMMANDING,
	FLASH_SPI_WRITING,
	FLASH_SPI_READING,
	FLASH_SPI_ERROR
} flash_spi_state_t;

typedef struct {
	flash_spi_state_t spi_state;
	uint32_t lfs_first_block_offset;
	OSPI_HandleTypeDef *spi_handle;
	gpio_pin_t ncs_pin;
	bool is_init;
} flash_lfs_ctx;

static flash_lfs_ctx g_flash_ctx = {.spi_state = FLASH_SPI_NOT_INIT, .ncs_pin = GPIO_PIN_FLASH_CS};

void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	g_flash_ctx.spi_state = FLASH_SPI_ERROR;
}

void HAL_OSPI_AbortCpltCallback(OSPI_HandleTypeDef *hospi) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	g_flash_ctx.spi_state = FLASH_SPI_ERROR;
}

void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef *hospi) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	if (FLASH_SPI_COMMANDING == g_flash_ctx.spi_state) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
	} else {
		g_flash_ctx.spi_state = FLASH_SPI_ERROR;
	}
}

void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	if (FLASH_SPI_WRITING == g_flash_ctx.spi_state) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
	} else {
		g_flash_ctx.spi_state = FLASH_SPI_ERROR;
	}
}

void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi) {
	if (hospi != (g_flash_ctx.spi_handle)) {
		return;
	}
	if (FLASH_SPI_READING == g_flash_ctx.spi_state) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
	} else {
		g_flash_ctx.spi_state = FLASH_SPI_ERROR;
	}
}

// this is a busy wait loop for flash to complete
static w_status_t wait_flash_complete(const uint32_t timeout_ms) {
	uint32_t start_time_ms = 0;
	uint32_t curr_time_ms = 0;
	timer_get_ms(&start_time_ms);

	while ((g_flash_ctx.spi_state != FLASH_SPI_READY) &&
		   (g_flash_ctx.spi_state != FLASH_SPI_ERROR)) {
		timer_get_ms(&curr_time_ms);
		if ((curr_time_ms - start_time_ms) > timeout_ms) {
			g_flash_ctx.spi_state = FLASH_SPI_READY; // set back to ready after a timeout
			// add logs
			return W_IO_TIMEOUT;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	};

	if (FLASH_SPI_ERROR == g_flash_ctx.spi_state) {
		// add logs
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

static w_status_t send_spi_cmd(OSPI_RegularCmdTypeDef *cmd, bool select_ncs) {
	// select chip
	if (select_ncs) {
		if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0) != W_SUCCESS) {
			return W_IO_ERROR;
		}
	}

	// check to make sure we are SPI bus is not busy
	if (g_flash_ctx.spi_state != FLASH_SPI_READY) {
		return W_FAILURE;
	}

	g_flash_ctx.spi_state = FLASH_SPI_COMMANDING;
	HAL_StatusTypeDef hal_status = HAL_OSPI_Command_IT((g_flash_ctx.spi_handle), cmd);

	// make sure command is fully sent
	if ((hal_status != HAL_OK) || (wait_flash_complete(FLASH_SINGLE_DELAY_MS) != W_SUCCESS)) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	// unselect chip
	if (select_ncs) {
		if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0) != W_SUCCESS) {
			return W_IO_ERROR;
		}
	}

	return W_SUCCESS;
}

// complete a RDSR
static w_status_t get_single_reg(OSPI_RegularCmdTypeDef *cmd, uint8_t *status, bool select_ncs) {
	// select chip
	if (select_ncs) {
		if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0) != W_SUCCESS) {
			return W_IO_ERROR;
		}
	}

	// check to make sure we are SPI bus is not busy
	if (g_flash_ctx.spi_state != FLASH_SPI_READY) {
		return W_FAILURE;
	}

	// if needed will add a block
	HAL_StatusTypeDef hal_status =
		HAL_OSPI_Command((g_flash_ctx.spi_handle), cmd, FLASH_SINGLE_DELAY_MS);

	if (hal_status != HAL_OK) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	// ready to recieve data
	g_flash_ctx.spi_state = FLASH_SPI_READING;
	hal_status = HAL_OSPI_Receive_IT(&hospi1, status);
	// make sure command is fully sent
	if ((hal_status != HAL_OK) || (wait_flash_complete(FLASH_SINGLE_DELAY_MS) != W_SUCCESS)) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	// unselect chip
	if (select_ncs) {
		if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0) != W_SUCCESS) {
			return W_IO_ERROR;
		}
	}

	return W_SUCCESS;
}

static w_status_t get_rdsr(uint8_t *status, bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_readstatus = {0};
	cmd_readstatus.Instruction = 0x05; // Read Status
	cmd_readstatus.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_readstatus.DataMode = HAL_OSPI_DATA_1_LINE;
	cmd_readstatus.AddressMode = HAL_OSPI_ADDRESS_NONE;
	cmd_readstatus.NbData = 1;

	return get_single_reg(&cmd_readstatus, status, select_ncs);
}

// get RDSCUR
static w_status_t get_rdscur(uint8_t *status, bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_readstatus = {0};
	cmd_readstatus.Instruction = 0x2B; // Read Status
	cmd_readstatus.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_readstatus.DataMode = HAL_OSPI_DATA_1_LINE;
	cmd_readstatus.AddressMode = HAL_OSPI_ADDRESS_NONE;
	cmd_readstatus.NbData = 1;

	return get_single_reg(&cmd_readstatus, status, select_ncs);
}

// get a RDCR
static w_status_t get_rdcr(uint8_t *status, bool select_ncs) {
	OSPI_RegularCmdTypeDef cmd_readstatus = {0};
	cmd_readstatus.Instruction = 0x15; // Read Status
	cmd_readstatus.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_readstatus.DataMode = HAL_OSPI_DATA_1_LINE;
	cmd_readstatus.AddressMode = HAL_OSPI_ADDRESS_NONE;
	cmd_readstatus.NbData = 1;

	return get_single_reg(&cmd_readstatus, status, select_ncs);
}

// grab data for WREN
static w_status_t send_wren(bool select_ncs) {
	// select chip
	if (select_ncs) {
		if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0) != W_SUCCESS) {
			return W_IO_ERROR;
		}
	}

	OSPI_RegularCmdTypeDef cmd_wren = {0};

	cmd_wren.Instruction = 0x06; // WREN
	cmd_wren.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_wren.AddressMode = HAL_OSPI_ADDRESS_NONE;
	cmd_wren.DataMode = HAL_OSPI_DATA_NONE;
	cmd_wren.DummyCycles = 0;
	cmd_wren.NbData = 0;

	return send_spi_cmd(&cmd_wren, select_ncs);
	;
}

static w_status_t flash_read(uint32_t address, uint8_t *data, uint32_t len) {
	// write
	OSPI_RegularCmdTypeDef cmd_read = {0};
#ifdef SPI_4_ENABLED
	cmd_read.Instruction = 0xEB; //  Quard read
	cmd_read.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_read.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
	cmd_read.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	cmd_read.Address = address;
	cmd_read.DataMode = HAL_OSPI_DATA_4_LINES;
	cmd_read.DummyCycles = flash_read_dummy_cycle;
	cmd_read.NbData = len; // ← bytes
#else
	cmd_read.Instruction = 0x03; //  Page Program
	cmd_read.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_read.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	cmd_read.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	cmd_read.Address = address;
	cmd_read.DataMode = HAL_OSPI_DATA_1_LINE;
	cmd_read.DummyCycles = 0;
	cmd_read.NbData = len; // ← bytes
#endif

	if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	// check to make sure we are SPI bus is not busy
	if (g_flash_ctx.spi_state != FLASH_SPI_READY) {
		return W_FAILURE;
	}

	HAL_StatusTypeDef hal_status =
		HAL_OSPI_Command((g_flash_ctx.spi_handle), &cmd_read, FLASH_SINGLE_DELAY_MS);
	if (hal_status != HAL_OK) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	g_flash_ctx.spi_state = FLASH_SPI_READING;
	hal_status = HAL_OSPI_Receive_IT((g_flash_ctx.spi_handle), data);
	// MANDATORY: Wait for the hardware peripheral to physically finish
	// If you don't do this, the next line will pull CS high while bits are still moving.
	if ((hal_status != HAL_OK) || (wait_flash_complete(FLASH_SINGLE_DELAY_MS) != W_SUCCESS)) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}
	if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

static w_status_t flash_spi_transmit(OSPI_RegularCmdTypeDef *cmd, uint8_t *data) {
	// write
	uint8_t rdsr_response = 0x0;
	for (uint8_t i = 0; i < MAX_RDStatus_ATTEMPTS; ++i) {
		if ((send_wren(true) != W_SUCCESS) && (get_rdsr(&rdsr_response, true) != W_SUCCESS)) {
			return W_FAILURE;
		}

		if ((rdsr_response & WEL_BIT_MASK) != 0x0) {
			break;
		}
	}

	if ((rdsr_response & WEL_BIT_MASK) == 0x0) { // if still can't stop WIP then that's a WIP error
		return W_FAILURE;
	}

	if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0) != W_SUCCESS) {
		return W_IO_ERROR;
	}
	// check to make sure we are SPI bus is not busy
	if (g_flash_ctx.spi_state != FLASH_SPI_READY) {
		return W_FAILURE;
	}

	HAL_StatusTypeDef hal_status =
		HAL_OSPI_Command((g_flash_ctx.spi_handle), cmd, FLASH_SINGLE_DELAY_MS);
	if (hal_status != HAL_OK) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	g_flash_ctx.spi_state = FLASH_SPI_WRITING;
	hal_status = HAL_OSPI_Transmit_IT((g_flash_ctx.spi_handle), data);
	// MANDATORY: Wait for the hardware peripheral to physically finish
	// If you don't do this, the next line will pull CS high while bits are still moving.
	if ((hal_status != HAL_OK) || (wait_flash_complete(FLASH_SINGLE_DELAY_MS) != W_SUCCESS)) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	// check WIP is complete
	uint32_t start_time_ms = 0;
	uint32_t curr_time_ms = 0;
	timer_get_ms(&start_time_ms);
	while ((curr_time_ms - start_time_ms) > FLASH_PROG_DELAY_MS) {
		vTaskDelay(pdMS_TO_TICKS(5));
		if (get_rdsr(&rdsr_response, true) != W_SUCCESS) {
			return W_IO_ERROR;
		}

		if ((rdsr_response & WIP_BIT_MASK) != 0x01) {
			break;
		}
	}

	if ((rdsr_response & WIP_BIT_MASK) != 0x0) {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

static w_status_t flash_write_page(uint32_t address, uint8_t *data, uint32_t len) {
	// will not write for more than a page
	if (((address % PAGE_SIZE) + len) > PAGE_SIZE) {
		return W_INVALID_PARAM;
	}

	// write
	OSPI_RegularCmdTypeDef cmd_write = {0};
#ifdef SPI_4_ENABLED
	cmd_write.Instruction = 0x38; //  Page Program
	cmd_write.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_write.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
	cmd_write.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	cmd_write.Address = address;
	cmd_write.DataMode = HAL_OSPI_DATA_4_LINES;
	cmd_write.DummyCycles = 0;
	cmd_write.NbData = len; // ← bytes
#else
	cmd_write.Instruction = 0x02; //  Page Program
	cmd_write.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_write.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	cmd_write.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	cmd_write.Address = address;
	cmd_write.DataMode = HAL_OSPI_DATA_1_LINE;
	cmd_write.DummyCycles = 0;
	cmd_write.NbData = len; // ← bytes
#endif

	if (flash_spi_transmit(&cmd_write, data) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	// check if wrote sucessfully
	uint8_t rdscur_response = 0;
	if (get_rdscur(&rdscur_response, true) != W_SUCCESS) {
		return W_IO_ERROR;
	} else if ((rdscur_response & P_FAIL_BIT_MASK) != 0x0) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

static w_status_t flash_erase_sector(uint32_t address) {
	// erase
	uint8_t rdsr_response = 0x0;
	for (uint8_t i = 0; i < MAX_RDStatus_ATTEMPTS; ++i) {
		if ((send_wren(true) != W_SUCCESS) && (get_rdsr(&rdsr_response, true) != W_SUCCESS)) {
			return W_FAILURE;
		}

		if ((rdsr_response & WEL_BIT_MASK) != 0x0) {
			break;
		}
	}

	if ((rdsr_response & WEL_BIT_MASK) == 0x0) { // if still can't stop WIP then that's a WIP error
		return W_FAILURE;
	}

	// write
	OSPI_RegularCmdTypeDef cmd_erase = {0};
	cmd_erase.Instruction = 0x20;
	cmd_erase.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	cmd_erase.DataMode = HAL_OSPI_DATA_NONE;
	cmd_erase.DummyCycles = 0;
	cmd_erase.NbData = 0;
#if (defined(SPI_4_ENABLED) && defined(QPI_ENABLED))
	cmd_erase.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
	cmd_erase.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
#else
	cmd_erase.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	cmd_erase.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
#endif

	if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_LOW, 0) != W_SUCCESS) {
		return W_IO_ERROR;
	}
	// check to make sure we are SPI bus is not busy
	if (g_flash_ctx.spi_state != FLASH_SPI_READY) {
		return W_FAILURE;
	}

	g_flash_ctx.spi_state = FLASH_SPI_COMMANDING;
	HAL_StatusTypeDef hal_status = HAL_OSPI_Command_IT((g_flash_ctx.spi_handle), &cmd_erase);

	// MANDATORY: Wait for the hardware peripheral to physically finish
	// If you don't do this, the next line will pull CS high while bits are still moving.
	if ((hal_status != HAL_OK) || (wait_flash_complete(FLASH_SINGLE_DELAY_MS) != W_SUCCESS)) {
		g_flash_ctx.spi_state = FLASH_SPI_READY;
		return W_IO_ERROR;
	}

	if (gpio_write(g_flash_ctx.ncs_pin, GPIO_LEVEL_HIGH, 0) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	// check WIP is complete
	uint32_t start_time_ms = 0;
	uint32_t curr_time_ms = 0;
	timer_get_ms(&start_time_ms);
	while ((curr_time_ms - start_time_ms) > FLASH_PROG_DELAY_MS) {
		vTaskDelay(pdMS_TO_TICKS(5));
		if (get_rdsr(&rdsr_response, true) != W_SUCCESS) {
			return W_IO_ERROR;
		}

		if ((rdsr_response & WIP_BIT_MASK) != 0x01) {
			break;
		}
	}
	if ((rdsr_response & WIP_BIT_MASK) != 0x01) {
		return W_IO_ERROR;
	}

	// check if wrote sucessfully
	uint8_t rdscur_response = 0;
	if (get_rdscur(&rdscur_response, true) != W_SUCCESS) {
		return W_IO_ERROR;
	} else if ((rdscur_response & E_FAIL_BIT_MASK) != 0x0) {
		return W_FAILURE;
	}

	return W_SUCCESS;
}

// Read a region in a block. Negative error codes are propagated
// to the user.
static int MX25L25645G_lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
								void *buffer, lfs_size_t size) {
	if ((size % (c->read_size)) != 0) {
		return LFS_ERR_INVAL;
	}

	uint32_t start_address = (g_flash_ctx.lfs_first_block_offset) + (block * c->block_size) + off;

	// no limit to
	if (flash_read(start_address, buffer, size) != W_SUCCESS) {
		return LFS_ERR_IO;
	}
	return LFS_ERR_OK;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int MX25L25645G_lfs_prg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
							   const void *buffer, lfs_size_t size) {
	uint32_t curr_address = (g_flash_ctx.lfs_first_block_offset) + (block * c->block_size) + off;

	uint32_t overflow_amount = 0;

	uint32_t curr_write_size = size;
	uint8_t *curr_buffer = (uint8_t *)buffer;

	// overflows page
	for (uint16_t i = 0; i < ((c->cache_size) / (c->prog_size)); ++i) {
		if ((size + (curr_address % (c->prog_size))) > (c->prog_size)) {
			curr_write_size = (c->prog_size) - (curr_address % (c->prog_size));
		} else {
			curr_write_size = size;
		}

		// write to end of page first
		if (flash_write_page(curr_address, curr_buffer, curr_write_size) != W_SUCCESS) {
			return LFS_ERR_IO;
		}

		// increment for next write
		curr_address += curr_write_size;
		curr_buffer += curr_write_size;

		// calculate the
		if (size > curr_write_size) {
			size -= curr_write_size;
		} else { // if we have written everything at this point
			break;
		}
	}

	if (size > 0) {
		return LFS_ERR_IO;
	} else {
		return LFS_ERR_OK;
	}
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int MX25L25645G_lfs_erase(const struct lfs_config *c, lfs_block_t block) {
	if ((c->block_size % SECTOR_SIZE) != 0) {
		return LFS_ERR_INVAL;
	}

	// TODO: future implement block erase if we can
	if (flash_erase_sector((g_flash_ctx.lfs_first_block_offset) + (block * c->block_size)) !=
		W_SUCCESS) {
		return LFS_ERR_IO;
	}

	return LFS_ERR_OK;
}

// Sync the state of the underlying block device. Negative error codes
// are propagated to the user.
static int MX25L25645G_lfs_sync(const struct lfs_config *c) {
	return LFS_ERR_OK;
}

// TODO: set static buffers!!!!!!!!!!!!!
// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
	// block device operations
	.read = MX25L25645G_lfs_read,
	.prog = MX25L25645G_lfs_prg,
	.erase = MX25L25645G_lfs_erase,
	.sync = MX25L25645G_lfs_sync,

	// block device configuration
	.read_size = 1,
	.prog_size = PAGE_SIZE,
	.block_size = SECTOR_SIZE,
	.block_count = 0,
	.block_cycles = 500,
	.cache_size = CACHE_SIZE,
	.lookahead_size = 2048 / 8,
	.compact_thresh = -1,
	.name_max = 0,
	.file_max = 0,
	.attr_max = 0,
	.metadata_max = 0,
	.inline_max = -1};

w_status_t MX25L25645G_init() {
	g_flash_ctx.spi_state = FLASH_SPI_READY;

	// SET UP THE CHIP

	// enable QE and set dc 0 and dc 1 to 10 for Quard IO Fast Read
	OSPI_RegularCmdTypeDef wrsr_cmd = {0};

	// Enable 4 Byte address
	wrsr_cmd.Instruction = 0x01; // WREN
	wrsr_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	wrsr_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	wrsr_cmd.DataMode = HAL_OSPI_DATA_NONE;
	wrsr_cmd.DummyCycles = 0;
	wrsr_cmd.NbData = 2;

	uint8_t update_reg[2] = {
		0x40, // Status Reg set QE to enabled
		0xA0 // CONFIG Reg set DC[1:0] to 10 and 4Byte on
	};

	// write the setting
	if (flash_spi_transmit(&wrsr_cmd, update_reg) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	OSPI_RegularCmdTypeDef en4b_cmd = {0};

	// Enable 4 Byte address
	en4b_cmd.Instruction = 0xB7; // WREN
	en4b_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	en4b_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	en4b_cmd.DataMode = HAL_OSPI_DATA_NONE;
	en4b_cmd.DummyCycles = 0;
	en4b_cmd.NbData = 0;

	if (send_spi_cmd(&en4b_cmd, true) != W_SUCCESS) {
		return W_IO_ERROR;
	}

	// check that our registers have been set correctly
	// check rdsr
	uint8_t rdsr_response = 0;
	if (get_rdsr(&rdsr_response, true) != W_SUCCESS) {
		return W_IO_ERROR;
	}
	if ((rdsr_response & 0xFC) != update_reg[0]) { // checks all fields except WEL adn WIP
		return W_FAILURE;
	}

	// check rdcr
	uint8_t rdcr_response = 0;
	if (get_rdcr(&rdcr_response, true) != W_SUCCESS) {
		return W_IO_ERROR;
	}
	if ((rdcr_response & 0xF8) != update_reg[1]) { // checks all fields except ODS and reserved
		return W_FAILURE;
	}

	g_flash_ctx.is_init = true;

	return W_SUCCESS;
}

w_status_t MX25L25645G_lfs_mount(lfs_t *lfs, OSPI_HandleTypeDef *hospi,
								 uint32_t first_block_offset) {
	memset(lfs, 0, sizeof(lfs_t));

	g_flash_ctx.lfs_first_block_offset = first_block_offset;
	g_flash_ctx.spi_handle = hospi;

	if (lfs_mount(lfs, &cfg) != 0) {
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}
