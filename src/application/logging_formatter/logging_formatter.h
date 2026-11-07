#ifndef LOGGING_FORMATTER_H
#define LOGGING_FORMATTER_H

#include "drivers/raw_logger/raw_logger.h"
#include "application/health_checks/health_checks.h"
#include "rocketlib/common/common.h"
#include "stm32h7xx_hal.h"
#include "application/logger/log.h"

/**
 * @brief Search for last written chunk
 * 
 * @param p_sd_handle Pointer to the HAL handle for tasks and DMA operations.
 */
w_status_t logging_formatting_init(SD_HandleTypeDef *p_sd_handle);

#endif // logging_formatter_H