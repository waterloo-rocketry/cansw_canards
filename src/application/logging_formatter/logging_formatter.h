#ifndef LOGGING_FORMATTER_H
#define LOGGING_FORMATTER_H

#include "drivers/raw_logger/raw_logger.h"

typedef enum {
	LOGGING_FORMATTING_DATA = 1,
	LOGGING_FORMATTING_ERROR = 2,
	LOGGING_FORMATTING_TEXT = 4
} logging_formatter_data_type_t;

/**
 * @brief Searches for the last unwritten, uncorrupted chunk to write to.
 *
 * @param p_sd_handle Pointer to the HAL handle for tasks and DMA operations.
 *
 * @implements FIRM-LLR-LFO-009, FIRM-LLR-LFO-010, FIRM-LLR-LFO-011, FIRM-LLR-LFO-012,
 * FIRM-LLR-LFO-014
 */
w_status_t logging_formatting_init(SD_HandleTypeDef *p_sd_handle);

/**
 * @brief Creates a header and a CRC footer for each 'file', and writes in an append only format
 *
 * @param buffer The data buffer to be written
 * @param buffer_length The amount of bytes in the buffer
 * @param data_type Specifies if the buffer is a Data, Text, or Error buffer from the designated
 * logging_formatter_data_type enum
 *
 * @implements FIRM-LLR-LFO-001, FIRM-LLR-LFO-003, FIRM-LLR-LFO-004, FIRM-LLR-LFO-006,
 * FIRM-LLR-LFO-007
 */
w_status_t logging_formatting_write(const uint8_t *buffer, uint32_t buffer_length,
                                    logging_formatter_data_type_t data_type);

#endif // logging_formatter_H
