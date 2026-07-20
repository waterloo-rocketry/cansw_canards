#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "third_party/rocketlib/include/common.h"

#include "GNC_codegen_types.h"
#include "common/gnc/gnc_types.h"

/**
 * persistent state updated by navigator and fsm
 */
typedef struct {
	uint32_t last_run_tenth_ms; // previous timestamp
	gnc_navigator_ctx_t gnc_navigator_ctx;
	GNC_codegenStackData *p_gnc_stack_data;
} navigator_ctx_t;

/**
 * @brief Structure to track navigator errors and status
 */
typedef struct {
	bool is_init; /**< Initialization status flag */
	uint32_t imu_data_timeouts; /**< Count of IMU data receive timeouts */
	uint32_t encoder_data_fails; /**< Count of encoder data receive failures */
	uint32_t controller_data_fails; /**< Count of controller output retrieval failures */
	uint32_t pad_filter_fails; /**< Count of pad filter run failures */
	uint32_t can_log_fails; /**< Count of CAN logging failures */
	uint32_t invalid_phase_errors; /**< Count of invalid flight phase errors */
} navigator_error_data_t;

/**
 * @brief initialize navigator module
 */
w_status_t navigator_init();

// TODO: to be REVIVED
// /**
//  * @brief Sends the complete state estimation data over CAN.
//  *
//  * Iterates through each state ID, builds a CAN message for it using the
//  * current state data, and transmits it.
//  *
//  * @param current_state Pointer to the current state estimation data (x_state_t).
//  * @return W_SUCCESS if all messages were sent successfully, W_FAILURE otherwise.
//  */
// w_status_t navigator_log_state_to_can(const x_state_t *current_state);

/**
 * @brief Report navigator module health status
 *
 * Retrieves and reports navigator error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t navigator_get_status(void);

/**
 * @brief 1 step of navigator
 * @param p_input pointer to the new navigator input
 * @param timestamp_tenth_ms is the current timestamp in tenth of a ms
 * @param p_ctx pointer to navigator context
 * @param p_output pointer to navigator output to update with new results
 * update with new actuation info
 */
w_status_t navigator_step(const navigator_input_t *p_input, const uint32_t timestamp_tenth_ms,
						  navigator_ctx_t *p_ctx, navigator_output_t *p_output);

#endif
