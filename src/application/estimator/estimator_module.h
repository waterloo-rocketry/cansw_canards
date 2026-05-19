#ifndef ESTIMATOR_MODULE_H
#define ESTIMATOR_MODULE_H

#include "arm_math.h"
#include <stdbool.h>
#include <stdint.h>

#include "application/estimator/estimator.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/pad_filter.h"
#include "application/fsm/fsm.h"
#include "common/gnc/gnc_types.h"

/**
 * input to estimator_module function
 */
typedef struct {
	float64_t timestamp_sec; // new timestamp (seconds)
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
w_status_t estimator_module(const estimator_module_input_t *input, fsm_state_t flight_phase,
							estimator_module_ctx_t *ctx, controller_input_t *output_to_controller);

#endif // ESTIMATOR_MODULE_H
