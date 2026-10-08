#ifndef raw_logger_H
#define raw_logger_H

#include <stdbool.h>
#include <stdint.h>

#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"

#include "stm32h7xx_hal.h"

#include "rocketlib/include/common.h"

/**
 * @brief Initialize the SD card hardware.
 * @pre Must be called after scheduler starts. The HAL sd init inside this uses hal_delay
 * in it, so it will hang forever if the timer interrupt is masked (freertos masks interrupts
 * before scheduler starts).
 * Will reset storage tracking and SD bits to zeros.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_init(void);

/**
 * @brief Recover the SDMMC in case of errors via resetting.
 * Upon recovery, must flag the faulty block and handle block skipping if necessary.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_recover(void);

// waiting for Shiming on what he wants specs to return
/**
 * @brief Get SD card specs, values should get cached during init
 *
 * @param[out] block_count - Number of 512-byte blocks. Block_count * 512 bytes = Total Byte Capacity
 * @param[out] erase_size_blocks - The SD cards preferred erase size.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_blocks);

/**
 * @brief Read from the beginning of a block.
 * @pre Buffer should be 32 byte aligned
 * 
 * Use a binary semaphore, signalling for processes to sleep and wake up.
 * (Architecture allows for future asynchronous implementation)
 *
 * @param[in] block_address - The address of the block to read.
 * @param[in] buffer - The buffer to read the block into.
 * @param[in] num_blocks - Number of blocks (512*n bytes) to read from the start of the block.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_read(uint32_t block_address, uint32_t *buffer, uint32_t num_blocks);

/**
 * @brief Write data to a block on the SD card.
 * @pre Buffer should be 32 byte aligned
 *
 * Use a binary semaphore, signalling for processes to sleep and wake up.
 * Need a function var to include data tracking to see how much storage is left.
 *
 * @param[in]  block_address    The address of the block to write to.
 * @param[in]  buffer       Pointer to the data that is to be written.
 * @param[in]  num_blocks    Number of blocks (512*n bytes) from buffer to write.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_write(uint32_t block_address, const uint32_t *buffer, uint32_t num_blocks);

/**
 * @brief Check if the SD card is writable.
 * 
 * This function verifies that the SD card is in the READY state using HAL_SD_GetCardState.
 * Also checks that the module is initialized (mutex created, etc)
 * @pre MUST be called only after scheduler starts.
 * @param p_sd_handle pointer to the HAL handle for the sd card to check
 * @return w_status_t - W_SUCCESS if the SD card is in READY state, W_FAILURE otherwise.
 */
w_status_t raw_logger_SD_is_writable(SD_HandleTypeDef *p_sd_handle);

/**
 * @brief Report SD card module health status
 *
 * Retrieves and reports SD card error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t raw_logger_SD_get_status(void);

#endif // raw_logger_H
