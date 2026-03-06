#ifndef EXAMPLE_MODULE_H
#define EXAMPLE_MODULE_H

#include "application/health_checks/health_checks.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Error codes for the example module
 * 
 * Each module defines its own error codes starting from 1.
 * Code 0 is reserved for "no error" (used with HEALTH_OK).
 */
#define EXAMPLE_ERR_NOT_INITIALIZED     1   
#define EXAMPLE_ERR_HARDWARE_FAULT      2   
#define EXAMPLE_ERR_TIMEOUT             3   

/**
 * @brief Get health status of the example module
 * 
 * @return health_status_t indicating module health
 */
health_status_t example_module_get_status(void);

#endif