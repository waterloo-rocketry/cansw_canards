#ifndef LOGGING_FORMATTER_H
#define LOGGING_FORMATTER_H

#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "drivers/raw_logger/raw_logger.h"
#include "rocketlib/common/common.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Search for last written chunk and take in a peripheral bus (SDMMC, SDMMC2)
 *
 * @param p_sd_handle Pointer to the HAL handle for tasks and DMA operations.
 *
 * @implements FIRM-LLR-LFO-009, FIRM-LLR-LFO-010, FIRM-LLR-LFO-011, FIRM-LLR-LFO-012,
 * FIRM-LLR-LFO-014
 */
w_status_t logging_formatting_init(SD_HandleTypeDef *p_sd_handle);

/**
 * @brief Creates a 10 byte header for each 'file' and ends with a 2 byte CRC footer
 *
 * @param buffer The data buffer to be written
 * @param buffer_length The amount of bytes in the buffer
 * @param data_type Specifies if the buffer is a Data, Text, or Error buffer
 *
 * @implements FIRM-LLR-LFO-001, FIRM-LLR-LFO-003, FIRM-LLR-LFO-004, FIRM-LLR-LFO-006,
 * FIRM-LLR-LFO-007
 */
w_status_t logging_formatting_write(const uint8_t *buffer, uint32_t buffer_length,
                                    uint8_t data_type);

#endif // logging_formatter_H
