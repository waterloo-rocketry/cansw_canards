#ifndef RAW_LOGGER_H
#define RAW_LOGGER_H

#include <stdint.h>

#include "application/health_checks/health_checks.h"

#include "rocketlib/common/common.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Initialize the SD card hardware.
 *
 * @pre Must be called before logging tasks start.
 *
 * @details Allocate block 0 for pointers and storage tracking.
 * @details Will reset storage tracking and SD bits to zeros.
 *
 * @param p_sd_handle Pointer to the HAL handle for tasks and DMA operations.
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_init(SD_HandleTypeDef *p_sd_handle);

/**
 * @brief   Recover the SDMMC in case of errors via resetting.
 *
 * @details Upon recovery, must flag the faulty block and handle block skipping if necessary.
 *
 * @return  w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_SD_recover(void);

/**
 * @brief Get SD card specs, values should get cached during init
 *
 * @param[out] block_count          Number of 512-byte blocks.
 * @param[out] erase_size_blocks    The SD cards preferred erase size.
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t raw_logger_get_SD_specs(uint32_t *block_count, uint32_t *erase_size_blocks);

/**
 * @brief Read data synchronously from the beginning of a block.
 *
 * @details Handles cache line invalidation and internal read timeout handling.
 *
 * @pre Buffer should be 32 byte aligned for DMA operations.
 * @pre Buffer must reside in IDMA-accessible memory (not DTCM).
 * @pre Byte_address and num_bytes must be 512 byte aligned
 *
 * @param[in] buffer       Pointer to the data on card to read.
 * @param[in] byte_address Target byte offset on the card (512-byte aligned).
 * @param[in] num_bytes    Total bytes to read; must be a non-zero multiple of 512.
 * @param[in] timeout      Maximum milliseconds to wait for driver availability.
 *
 * @return w_status_t W_SUCCESS on completion, W_FAILURE, W_INVALID PARAM,
 *         or W_IO_TIMEOUT on timeout or error.
 */
w_status_t raw_logger_SD_read(uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
                              uint32_t timeout);

/**
 * @brief Writes data synchronously to a block on the SD card.
 *
 * @details Handles cache line invalidation and internal write timeout handling.
 *
 * @pre Buffer must be 32-byte aligned for DMA operations.
 * @pre Buffer must reside in IDMA-accessible memory (not DTCM).
 * @pre Byte_address and num_bytes must be 512-byte aligned.
 *
 * @param[in] buffer       Pointer to the payload data to write.
 * @param[in] byte_address Target byte offset on the card (512-byte aligned).
 * @param[in] num_bytes    Total bytes to write; must be a non-zero multiple of 512.
 * @param[in] timeout      Maximum milliseconds to wait for driver availability.
 *
 * @return w_status_t W_SUCCESS on completion, W_FAILURE, W_INVALID PARAM,
 *         or W_IO_TIMEOUT on timeout or error.
 *
 * @satisfies REQ_RAW_LOG_012, REQ_RAW_LOG_015
 */
w_status_t raw_logger_SD_write(const uint8_t *buffer, uint32_t byte_address, uint32_t num_bytes,
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
