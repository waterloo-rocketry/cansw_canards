#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "application/controller/gain_table.h"
#include "application/health_checks/health_checks.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "common/gnc/gnc_types.h"
#include "third_party/rocketlib/include/common.h"

/* Enums/Types */

// main controller state using in task
typedef struct {
	uint32_t last_ms; // currently unused...
	uint32_t can_send_errors;
	uint32_t data_miss_counter;
} controller_t;

/**
 * @brief Structure to track controller errors and status
 */
typedef struct {
	bool is_init; /**< Initialization status flag */
	uint32_t can_send_errors; /**< Number of CAN send failures */
	uint32_t data_miss_counter; /**< Number of missed data updates from estimator */
	uint32_t timestamp_errors; /**< Number of timer/timestamp retrieval failures */
	uint32_t gain_interpolation_errors; /**< Number of gain interpolation failures */
	uint32_t angle_calculation_errors; /**< Number of commanded angle calculation failures */
	uint32_t log_errors; /**< Number of logging failures */
} controller_error_data_t;

/**
 * state of a controller instance.
 */
typedef struct {
	uint32_t last_run_ms; // last time the controller did a full loop. use for rate-limiting
} controller_ctx_t;

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void);

/**
 * @brief run 1 step of the controller
 * @param context pointer to controller global context
 * @param input pointer to new inputs for this iteration
 * @param output pointer to the output struct to update with new command
 * @param act_allowed_timestamp_ms the timestamp at which act_allowed was set (only used when passed
 * actuation allowed)
 * @param curr_timestamp_ms the currrent timestamp
 */
w_status_t controller_step(controller_ctx_t *context, const controller_input_t *input,
						   controller_output_t *output, const uint32_t act_allowed_timestamp_ms,
						   const uint32_t curr_timestamp_ms);

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
