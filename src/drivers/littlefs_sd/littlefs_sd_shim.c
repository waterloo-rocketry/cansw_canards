#include "lfs.h"
#include "stm32h7xx_hal.h"
#include "sdmmc.h"
#include "FreeRTOS.h"
#include "task.h"

#include "third_party/rocketlib/include/common.h"
#include "third_party/rocketlib/include/mbr.h"
#include "application/logger/log.h"

#define SD_RW_TIMEOUT_MS 1000

#define CACHE_SIZE 512
#define LOOKAHEAD_SIZE 512

volatile uint64_t write_dma_count = 0;
volatile uint64_t read_dma_count = 0;

static SD_HandleTypeDef *sd_bus_handle = &hsd2;

static SD_HandleTypeDef *lfsshim_sd_hsd;
static uint32_t lfsshim_sd_first_block_offset = 0;

typedef enum {
	SD_DMA_FREE,
	SD_DMA_WRITE_BUSY,
	SD_DMA_READ_BUSY,
	SD_DMA_ERROR
} sd_dma_bus_state_t;

static volatile sd_dma_bus_state_t sd_dma_state = SD_DMA_FREE; 

// set the interrupts
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}

	if (SD_DMA_WRITE_BUSY == sd_dma_state) {
		write_dma_count--;
	}

	sd_dma_state = SD_DMA_FREE;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}

	if (SD_DMA_READ_BUSY == sd_dma_state) {
		read_dma_count--;
	}
	sd_dma_state = SD_DMA_FREE;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}

	if (SD_DMA_WRITE_BUSY == sd_dma_state) {
		write_dma_count--;
	}
	sd_dma_state = SD_DMA_ERROR;
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}

	if (SD_DMA_WRITE_BUSY == sd_dma_state) {
		write_dma_count--;
	}
	sd_dma_state = SD_DMA_ERROR;
}

static int lfsshim_sd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
						   void *buffer, lfs_size_t size) {
	// make sure sd bus free
	if (sd_dma_state != SD_DMA_FREE) {
		return -1; 
	}
	
	uint32_t block_addr = block + lfsshim_sd_first_block_offset;

	w_assert((size % c->block_size) == 0);
	w_assert(off == 0);

	uint32_t num_blocks = size / c->block_size;

	sd_dma_state = SD_DMA_READ_BUSY;
	read_dma_count++;
	HAL_StatusTypeDef hal = HAL_SD_ReadBlocks_DMA(
		lfsshim_sd_hsd, (uint8_t *)buffer, block_addr, num_blocks);
	if (hal != HAL_OK) {
		sd_dma_state = SD_DMA_FREE;
		return -1; // LFS_ERR_IO
	}

	// Wait for card to be ready (polling)
	uint32_t start = HAL_GetTick();
	while (((sd_dma_state != SD_DMA_FREE) && (sd_dma_state != SD_DMA_ERROR)) || (HAL_SD_GetCardState(lfsshim_sd_hsd) != HAL_SD_CARD_TRANSFER)) {
		if ((HAL_GetTick() - start) > SD_RW_TIMEOUT_MS) {
			sd_dma_state = SD_DMA_FREE;
			return -1; // timeout -> LFS_ERR_IO
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// allow for bus to still be used however do return a fault
	if (SD_DMA_ERROR == sd_dma_state) {
		sd_dma_state = SD_DMA_FREE;
		return -1;
	}

	return 0; // success
}

static int lfsshim_sd_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
							const void *buffer, lfs_size_t size) {
	// make sure sd bus free
	if (sd_dma_state != SD_DMA_FREE) {
		return -1; 
	}

	uint32_t block_addr = block + lfsshim_sd_first_block_offset;

	w_assert((size % c->block_size) == 0);
	w_assert(off == 0);

	uint32_t num_blocks = size / c->block_size;

	sd_dma_state = SD_DMA_WRITE_BUSY;
	write_dma_count++;
	HAL_StatusTypeDef hal = HAL_SD_WriteBlocks_DMA(
		lfsshim_sd_hsd, (uint8_t *)buffer, block_addr, num_blocks);
	if (hal != HAL_OK) {
		sd_dma_state = SD_DMA_FREE;
		return -1; // LFS_ERR_IO
	}

	// Wait for card to be ready (polling)
	uint32_t start = HAL_GetTick();
	while (((sd_dma_state != SD_DMA_FREE) && (sd_dma_state != SD_DMA_ERROR)) || (HAL_SD_GetCardState(lfsshim_sd_hsd) != HAL_SD_CARD_TRANSFER)) {
		if ((HAL_GetTick() - start) > SD_RW_TIMEOUT_MS) {
			sd_dma_state = SD_DMA_FREE;
			return -1; // timeout -> LFS_ERR_IO
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// allow for bus to still be used however do return a fault
	if (SD_DMA_ERROR == sd_dma_state) {
		sd_dma_state = SD_DMA_FREE;
		return -1;
	}

	// if (HAL_SD_GetCardState(lfsshim_sd_hsd) != HAL_SD_CARD_TRANSFER) {
	// 	return -1;
	// }

	return 0; // success
}

static int lfsshim_sd_erase(const struct lfs_config *c, lfs_block_t block) {
	return 0; // SD does not require explicit erase
}

static int lfsshim_sd_sync(const struct lfs_config *c) {
	return 0;
}

static uint8_t read_buffer[CACHE_SIZE] __attribute__((section(".sram4"))) = {0};
static uint8_t prog_buffer[CACHE_SIZE] __attribute__((section(".sram4"))) = {0};
static uint8_t lookahead_buffer[CACHE_SIZE] __attribute__((section(".sram4"))) = {0};

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
	// block device operations
	.read = lfsshim_sd_read,
	.prog = lfsshim_sd_write,
	.erase = lfsshim_sd_erase,
	.sync = lfsshim_sd_sync,

	// block device configuration
	.read_size = 512,
	.prog_size = 512,
	.block_size = 512,
	.block_count = 0,
	.block_cycles = -1,
	.cache_size = CACHE_SIZE,
	.lookahead_size = LOOKAHEAD_SIZE,
	.compact_thresh = -1,
	.name_max = 0,
	.file_max = 0,
	.attr_max = 0,
	.metadata_max = 0,
	.inline_max = -1,

	.read_buffer = read_buffer,
	.prog_buffer = prog_buffer,
	.lookahead_buffer = lookahead_buffer
};

w_status_t lfsshim_sd_mount(lfs_t *lfs, SD_HandleTypeDef *hsd, uint32_t first_block_offset) {
	memset(lfs, 0, sizeof(lfs_t));

	lfsshim_sd_hsd = hsd;
	lfsshim_sd_first_block_offset = first_block_offset;
	int32_t res = lfs_mount(lfs, &cfg);
	if (res != 0) {
		if (res == -84) HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
		return W_IO_ERROR;
	}

	return W_SUCCESS;
}

w_status_t lfsshim_sd_mount_mbr(lfs_t *lfs, SD_HandleTypeDef *hsd) {
	uint8_t mbr_sector[512];

	HAL_StatusTypeDef hal = HAL_SD_ReadBlocks(hsd, mbr_sector, 0, 1, 50U);
	if (hal != HAL_OK) {
		return W_FAILURE;
	}

	uint32_t first_block_offset = 0;
	w_status_t status;
	if ((status = mbr_parse(mbr_sector, 0x83, &first_block_offset)) != W_SUCCESS) {
		return status;
	}

	if (lfsshim_sd_mount(lfs, hsd, first_block_offset) != 0) {
		return W_IO_ERROR;
	}
	return W_SUCCESS;
}
