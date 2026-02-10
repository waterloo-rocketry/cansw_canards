#ifndef ESTIMATOR_MODULE_H
#define ESTIMATOR_MODULE_H

#include "arm_math.h"
#include <stdbool.h>
#include <stdint.h>

#include "application/controller/controller.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/pad_filter.h"
#include "application/flight_phase/flight_phase.h"

/**
 * persistent state updated by estimator_module
 */
typedef struct {
	x_state_t x;
	double P_flat[SIZE_STATE * SIZE_STATE];
	y_imu_t bias_movella;
	y_imu_t bias_pololu;
	double t; // previous timestamp
	// estimator ctx must have exactly 1 pad filter ctx
	pad_filter_ctx_t pad_filter_ctx;
} estimator_module_ctx_t;

/**
 * input to estimator_module function
 */
typedef struct {
	float timestamp; // new timestamp (sec)
	y_imu_t movella; // latest movella data
	y_imu_t pololu; // latest pololu data
	bool movella_is_dead; // true if movella is dead
	bool pololu_is_dead; // true if pololu is dead
	controller_output_t cmd; // latest controller cmd
	float encoder; // latest encoder val (rad)
	bool encoder_is_dead; // true if encoder is dead
} estimator_module_input_t;

/**
 * @param input latest input data into estimator
 * @param flight_phase current flight phase
 * @param ctx persistent estimator context to read and update
 * @param output_to_controller pointer to write output cmd for controller
 */
w_status_t estimator_module(const estimator_module_input_t *input,
							flight_phase_state_t flight_phase, estimator_module_ctx_t *ctx,
							controller_input_t *output_to_controller);

#endif // ESTIMATOR_MODULE_H