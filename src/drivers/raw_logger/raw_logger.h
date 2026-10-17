#ifndef RAW_LOGGER_H
#define RAW_LOGGER_H

#include <stdint.h>

#include "application/health_checks/health_checks.h"

#include "rocketlib/common/common.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Initialize the SD card hardware.
 * @pre MUST be called after scheduler starts. The HAL sd init inside this uses hal_delay
 * in it, so it will hang forever if the timer interrupt is masked (freertos masks interrupts
 * before scheduler starts).
 * Should also verify that the SD card is writable and in the READY state using HAL_SD_GetCardState.
 * Also checks that the module is initialized (enum assigned, etc)
 * @param p_sd_handle pointer to the HAL handle for the sd card to check
 *
 * Allocate block 0 for pointers and storage tracking.
 * Will reset storage tracking and SD bits to zeros.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_init(SD_HandleTypeDef *p_sd_handle);

/**
 * @brief Recover the SDMMC in case of errors via resetting.
 * Upon recovery, must flag the faulty block and handle block skipping if necessary.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_recover(void);

/**
 * @brief Get SD card specs, values should get cached during init
 *
 * @param[out] block_count - Number of 512-byte blocks. Block_count * 512 bytes = Total Byte
 * Capacity
 * @param[out] erase_size_blocks - The SD cards preferred erase size.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_blocks);

/**
 * @brief Read data synchronously from the beginning of a block.
 * @pre Buffer should be 32 byte aligned
 * @pre Buffer must reside in IDMA-accessible memory, not DTCM (this is already the case)
 *
 * Shall invalidate the cache after DMA completion.
 * Shall use an enum that tracks if an action (or an error) is currently taking place as to
 * avoid concurrency issues.
 *
 * @param[in] block_address - The address of the block to read.
 * @param[in] buffer - The buffer to read the block into.
 * @param[in] num_bytes - Number of bytes to read from the start of the block.
 * Num_bytes should be multiple of 512, but the driver should handle arbitrary numbers.
 * @param[out] timeout - Number of ticks to wait before determining read is unavailable
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_read(uint32_t block_address, uint32_t *buffer, uint32_t num_bytes,
                              uint32_t timeout);

/**
 * @brief Write data synchronously to a block on the SD card.
 * @pre Buffer should be 32 byte aligned for DMA
 * @pre Buffer must reside in IDMA-accessible memory, not DTCM (this is already the case)
 *
 * Shall clean the cache line prior to writing.
 * Shall use an enum that tracks if an action (or an error) is currently taking place as to
 * avoid concurrency issues.
 * Shall include storage tracking to see how much storage is left.
 * If another write is attempted while a current write is occcuring, create a timeout to
 * let the current write finish before allowing the next
 *
 * @param[in]  block_address    The address of the block to write to.
 * @param[in]  buffer       Pointer to the data that is to be written.
 * @param[in]  num_blocks    Number of blocks (512*n bytes) from buffer to write.
 * @param[out] timeout - Number of ticks to wait before determining write is unavailable

 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_write(uint32_t block_address, const uint32_t *buffer, uint32_t num_blocks,
                               uint32_t timeout);

/**
 * @brief Report SD card module health status
 *
 * Retrieves and reports SD card error statistics and initialization status
 * through log messages.
 *
 * @return callers read CAN board specific err bitfield from the returned struct
 */
health_status_t raw_logger_SD_get_status(void);

#endif // raw_logger_H
