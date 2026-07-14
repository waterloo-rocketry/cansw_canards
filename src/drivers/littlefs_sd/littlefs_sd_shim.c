#include "FreeRTOS.h"
#include "lfs.h"
#include "sdmmc.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "application/logger/log.h"
#include "third_party/rocketlib/include/common.h"
#include "third_party/rocketlib/include/mbr.h"

#define SD_RW_TIMEOUT_MS 2000

#define SD_SECTOR_SIZE 512 
#define LFS_BLK_SIZE 512
#define LOOKAHEAD_SIZE ((LOG_BUFFER_SIZE-1)/(SD_SECTOR_SIZE * 8)) + 1 // should be just over the size

static uint8_t read_buffer[LFS_BLK_SIZE] __attribute__((section(".sdmmc2_ram"))) = {0};
static uint8_t prog_buffer[LFS_BLK_SIZE] __attribute__((section(".sdmmc2_ram"))) = {0};
static uint8_t lookahead_buffer[LOOKAHEAD_SIZE] __attribute__((section(".sdmmc2_ram"))) = {0};

volatile uint32_t write_dma_sum_count = 0;
volatile uint32_t read_dma_sum_count = 0;
volatile uint32_t error_dma_count = 0;
volatile uint32_t error_timeout_count = 0;
volatile uint32_t error_other_count = 0;
volatile uint32_t total_sd_write_time = 0;
volatile uint32_t total_num_sd_write = 0;
volatile uint32_t max_write_time = 0;
volatile uint32_t sd_total_data_written = 0;
volatile uint32_t total_sd_read_time = 0;
volatile uint32_t total_num_sd_read = 0;
volatile uint32_t max_read_time = 0;
volatile uint32_t sd_total_data_read = 0;

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
		write_dma_sum_count--;
	}

	sd_dma_state = SD_DMA_FREE;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}

	if (SD_DMA_READ_BUSY == sd_dma_state) {
		read_dma_sum_count--;
	}
	sd_dma_state = SD_DMA_FREE;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}
	sd_dma_state = SD_DMA_ERROR;
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
	if (hsd != sd_bus_handle) {
		return;
	}
	sd_dma_state = SD_DMA_ERROR;
}

static void lfsshim_sd_reset() {
	HAL_SD_DeInit(lfsshim_sd_hsd);
	MX_SDMMC2_SD_Init();
	vTaskDelay(pdMS_TO_TICKS(500));
}

static int lfsshim_sd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
						   void *buffer, lfs_size_t size) {
	// make sure sd bus free
	if (sd_dma_state != SD_DMA_FREE) {
		error_other_count++;
		return LFS_ERR_IO;
	}

	uint32_t block_addr = ((block) * LFS_BLK_SIZE / SD_SECTOR_SIZE)  + lfsshim_sd_first_block_offset + (off / c->read_size);

	w_assert((size % c->prog_size) == 0);
	w_assert((off % c->prog_size) == 0);

	uint32_t num_blocks = size / c->prog_size;

	sd_dma_state = SD_DMA_READ_BUSY;
	read_dma_sum_count++;
	HAL_StatusTypeDef hal =
		HAL_SD_ReadBlocks_DMA(lfsshim_sd_hsd, (uint8_t *)buffer, block_addr, num_blocks);
	if (hal != HAL_OK) {
		sd_dma_state = SD_DMA_FREE;
		error_other_count++;
		return LFS_ERR_IO; // LFS_ERR_IO
	}

	// Wait for card to be ready (polling)
	uint32_t start = HAL_GetTick();
	while ((sd_dma_state != SD_DMA_FREE) && (sd_dma_state != SD_DMA_ERROR)) {
		if ((HAL_GetTick() - start) > SD_RW_TIMEOUT_MS) {
			sd_dma_state = SD_DMA_FREE;
			error_timeout_count++;
			return LFS_ERR_IO; // timeout -> LFS_ERR_IO
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// continue to wait if card is not ready
	while (HAL_SD_GetCardState(lfsshim_sd_hsd) != HAL_SD_CARD_TRANSFER) {
		if ((HAL_GetTick() - start) > SD_RW_TIMEOUT_MS) {
			sd_dma_state = SD_DMA_FREE;
			error_timeout_count++;
			return LFS_ERR_IO;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// allow for bus to still be used however do return a fault
	if (SD_DMA_ERROR == sd_dma_state) {
		sd_dma_state = SD_DMA_FREE;
		error_dma_count++;
		lfsshim_sd_reset();
		return LFS_ERR_IO;
	}

	uint32_t read_time = (HAL_GetTick() - start);
	total_sd_read_time += read_time;
	total_num_sd_read++;
	sd_total_data_read += size;

	if (read_time > max_read_time) {
		max_read_time = read_time;
	}

	return 0; // success
}

static int lfsshim_sd_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
							const void *buffer, lfs_size_t size) {
	// make sure sd bus free
	if (sd_dma_state != SD_DMA_FREE) {
		error_other_count++;
		return LFS_ERR_IO;
	}

	uint32_t block_addr = ((block) * LFS_BLK_SIZE / SD_SECTOR_SIZE)  + lfsshim_sd_first_block_offset + (off / c->read_size);

	w_assert((size % c->read_size) == 0);
	w_assert((off % c->read_size) == 0);

	uint32_t num_blocks = size / c->read_size;

	sd_dma_state = SD_DMA_WRITE_BUSY;
	write_dma_sum_count++;
	HAL_StatusTypeDef hal =
		HAL_SD_WriteBlocks_DMA(lfsshim_sd_hsd, (uint8_t *)buffer, block_addr, num_blocks);
	if (hal != HAL_OK) {
		sd_dma_state = SD_DMA_FREE;
		error_other_count++;
		return LFS_ERR_IO;
	}

	// Wait for card to be ready (polling)
	uint32_t start = HAL_GetTick();
	while ((sd_dma_state != SD_DMA_FREE) && (sd_dma_state != SD_DMA_ERROR)) {
		if ((HAL_GetTick() - start) > SD_RW_TIMEOUT_MS) {
			sd_dma_state = SD_DMA_FREE;
			error_timeout_count++;
			return LFS_ERR_IO;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// continue to wait if card is not ready
	while (HAL_SD_GetCardState(lfsshim_sd_hsd) != HAL_SD_CARD_TRANSFER) {
		if ((HAL_GetTick() - start) > SD_RW_TIMEOUT_MS) {
			sd_dma_state = SD_DMA_FREE;
			error_timeout_count++;
			return LFS_ERR_IO;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	uint32_t write_time = (HAL_GetTick() - start);
	total_sd_write_time += write_time;
	total_num_sd_write++;
	sd_total_data_written += size;

	if (write_time > max_write_time) {
		max_write_time = write_time;
	}

	// allow for bus to still be used however do return a fault
	if (SD_DMA_ERROR == sd_dma_state) {
		sd_dma_state = SD_DMA_FREE;
		error_dma_count++;
		lfsshim_sd_reset();
		return LFS_ERR_IO;
	}

	return 0; // success
}

static int lfsshim_sd_erase(const struct lfs_config *c, lfs_block_t block) {
	return 0; // SD does not require explicit erase
}

static int lfsshim_sd_sync(const struct lfs_config *c) {
	return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
	// block device operations
	.read = lfsshim_sd_read,
	.prog = lfsshim_sd_write,
	.erase = lfsshim_sd_erase,
	.sync = lfsshim_sd_sync,

	// block device configuration
	.read_size = SD_SECTOR_SIZE,
	.prog_size = SD_SECTOR_SIZE,
	.block_size = LFS_BLK_SIZE,
	.block_count = 0,
	.block_cycles = -1,
	.cache_size = LFS_BLK_SIZE,
	.lookahead_size = LOOKAHEAD_SIZE,
	.compact_thresh = -1, // potentially turn off if too much performance penailty

	.read_buffer = read_buffer,
	.prog_buffer = prog_buffer,
	.lookahead_buffer = lookahead_buffer,

	.name_max = 0,
	.file_max = 0,
	.attr_max = 0,
	.metadata_max = 0,
	.inline_max = -1,
};

w_status_t lfsshim_sd_mount(lfs_t *lfs, SD_HandleTypeDef *hsd, uint32_t first_block_offset) {
	memset(lfs, 0, sizeof(lfs_t));

	lfsshim_sd_hsd = hsd;
	lfsshim_sd_first_block_offset = first_block_offset;
	int32_t res = lfs_mount(lfs, &cfg);
	if (res != 0) {
		if (res == -5) HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
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
