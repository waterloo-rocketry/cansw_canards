#include "rocketlib/include/common.h"
#include <stdint.h>
#include <stdio.h>

#include "lfs.h"
#include "main.h"
#include "sdmmc.h"

#include "fs.h"
#include "log.h"
#include "rocketlib/include/mbr.h"
#include "rocketlib/include/littlefs_sd_shim.h"

#define MAX_FILE_PER_DIR 1000

lfs_t lfs;
lfs_file_t logfile;

uint32_t index_counter = 0;
uint32_t page_counter = 0;

static uint32_t log_written_size;
static uint32_t sd_log_file_name;
static uint32_t sd_used = 0;
static uint32_t sd_free = 0;
static uint32_t flash_log_file_name;
static uint32_t flash_used = 0;
static uint32_t flash_free = 0;

static void fs_new_file(void) {
	// Create directory as nessary
	if ((index_counter % MAX_FILE_PER_DIR) == 0) {
		char dir_name[100];
		sprintf(dir_name, "dir_%04lu", index_counter / MAX_FILE_PER_DIR);
		lfs_mkdir(&lfs, dir_name);
	}

	// Choose file name
	char log_filename[100];
	sprintf(log_filename,
			"dir_%04lu/log_%04lu.bin",
			index_counter / MAX_FILE_PER_DIR,
			index_counter % MAX_FILE_PER_DIR);

	++index_counter;

	// Update counter file
	lfs_file_t counter_file;
	lfs_file_open(&lfs, &counter_file, "/counter.bin", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
	lfs_file_write(&lfs, &counter_file, &index_counter, sizeof(index_counter));
	lfs_file_close(&lfs, &counter_file);

	if (lfs_file_open(&lfs, &logfile, log_filename, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL) != 0) {
	}

	page_counter = 0;
	log_written_size = 0;
}

w_status_t fs_init(void) {
	HAL_SD_InitCard(&hsd1);

	// LittleFS mount — skip MBR, use whole card at offset 0 for testing
	// Format first if not yet formatted, then mount
	if (lfsshim_sd_mount(&lfs, &hsd1, 0) != W_SUCCESS) {
		// if (lfsshim_sd_format(&lfs, &hsd1, 0) != W_SUCCESS) {
		// 	return W_FAILURE;
		// }
		if (lfsshim_sd_mount(&lfs, &hsd1, 0) != W_SUCCESS) {
			return W_FAILURE;
		}
	}

	// Read the file count counter
	lfs_file_t counter_file;
	if (lfs_file_open(&lfs, &counter_file, "counter.bin", LFS_O_RDONLY) == 0) {
		lfs_file_read(&lfs, &counter_file, &index_counter, sizeof(index_counter));
		lfs_file_close(&lfs, &counter_file);
	}

	fs_new_file();

	return W_SUCCESS;
}

void fs_write_page(const uint8_t *page) {
	if (lfs_file_write(&lfs, &logfile, page, PAGE_SIZE) != 0) {}
	++page_counter;

	if (page_counter >= MAX_FILE_SIZE_PAGES) {
		lfs_file_close(&lfs, &logfile);
		fs_new_file();
	} else {
		lfs_file_sync(&lfs, &logfile);
	}

	// Increment the written size of the current file
	log_written_size += PAGE_SIZE;
}

w_status_t status_report(void) {
	sd_log_file_name = index_counter;

	struct lfs_fsinfo fsinfo;
	int result = lfs_fs_stat(&lfs, &fsinfo);
	if (result < 0) {
		return W_FAILURE;
	}

	sd_used = (lfs_fs_size(&lfs) * fsinfo.block_size) >> 20;
	sd_free = ((fsinfo.block_count - lfs_fs_size(&lfs)) * fsinfo.block_size) >> 20;
	return W_SUCCESS;
}

uint32_t fs_get_log_written_size(void) {
	return log_written_size;
}

uint32_t fs_get_sd_log_file_name(void) {
	return sd_log_file_name;
}

uint32_t fs_get_sd_used(void) {
	return sd_used;
}

uint32_t fs_get_sd_free(void) {
	return sd_free;
}