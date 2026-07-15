#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

#include "GNC_codegen_types.h"
#include "application/health_checks/health_checks.h"
#include "common/gnc/gnc_types.h"

/* Enums/Types */

/**
 * @brief Structure to track controller errors and status
 */
typedef struct {
	bool is_init; /**< Initialization status flag */
	uint32_t null_ctx_count;
	uint32_t controller_not_run_count;

	bool ctx_is_null;
	bool controller_not_run;
} controller_error_data_t;

/**
 * state of a controller instance.
 */
typedef struct {
	uint32_t last_run_tenth_ms; // previous timestamp
	gnc_controller_ctx_t gnc_controller_ctx;
	GNC_codegenStackData *p_gnc_stack_data;
} controller_ctx_t;

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void);

/**
 * @brief run 1 step of the controller
 * @param p_input pointer to new inputs for this iteration
 * @param timestamp_tenth_ms is the current timestamp in tenth of a ms
 * @param p_ctx pointer to controller global context
 * @param p_output pointer to the output struct to update with new command
 */
w_status_t controller_step(const controller_input_t *p_input, const uint32_t timestamp_tenth_ms,
						   controller_ctx_t *p_ctx, controller_output_t *p_output);

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument);

/**
 * @brief Report controller module health status
 *
 * Retrieves and reports controller error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t controller_get_status(void);

#endif // CONTROLLER_H_
