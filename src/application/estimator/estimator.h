// TODO: remember to rename the file name to navigator.h
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "application/estimator/estimator_types.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "common/gnc/gnc_types.h"
#include "common/math/math.h"

#include "third_party/rocketlib/include/common.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * this holds persistent data for 1 instance of a pad_filter (ie, its context)
 */
typedef struct {
	y_imu_t filtered_1;
	y_imu_t filtered_2;
	bool is_initialized;
} pad_filter_ctx_t;

/**
 * persistent state updated by estimator and fsm
 */
typedef struct {
	x_state_t x;
	double P_flat[SIZE_STATE * SIZE_STATE];
	y_imu_t bias_movella;
	y_imu_t bias_pololu;
	double t_sec; // previous timestamp
	// estimator ctx must have exactly 1 pad filter ctx
	pad_filter_ctx_t pad_filter_ctx;
} estimator_module_ctx_t; // TODO: rename to simply navigator_ctx_t

/**
 * @brief Structure to track estimator errors and status
 */
typedef struct {
	bool is_init; /**< Initialization status flag */
	uint32_t imu_data_timeouts; /**< Count of IMU data receive timeouts */
	uint32_t encoder_data_fails; /**< Count of encoder data receive failures */
	uint32_t controller_data_fails; /**< Count of controller output retrieval failures */
	uint32_t pad_filter_fails; /**< Count of pad filter run failures */
	uint32_t can_log_fails; /**< Count of CAN logging failures */
	uint32_t invalid_phase_errors; /**< Count of invalid flight phase errors */
} estimator_error_data_t;

/**
 * @brief initialize estimator module
 */
w_status_t estimator_init();

/**
 * @brief Sends the complete state estimation data over CAN.
 *
 * Iterates through each state ID, builds a CAN message for it using the
 * current state data, and transmits it.
 *
 * @param current_state Pointer to the current state estimation data (x_state_t).
 * @return W_SUCCESS if all messages were sent successfully, W_FAILURE otherwise.
 */
w_status_t estimator_log_state_to_can(const x_state_t *current_state);

/**
 * @brief Report estimator module health status
 *
 * Retrieves and reports estimator error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t estimator_get_status(void);

/**
 * @brief 1 step of estimator
 * @param ctx pointer to estimator context
 * @param p_input pointer to the new navigator input
 * @param p_output pointer to navigator output to update with new results
 * update with new actuation info
 */
w_status_t estimator_step(estimator_module_ctx_t *ctx, const navigator_input_t *p_input,
						  navigator_output_t *p_output);

#endif
