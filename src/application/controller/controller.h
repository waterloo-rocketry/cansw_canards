#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "application/controller/gain_table.h"
#include "third_party/rocketlib/include/common.h"

#include "FreeRTOS.h"

#include <stdbool.h>
#include <stdint.h>

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1) // subtract 1 for the pre-gain
#define ROLL_STATE_NUM (FEEDBACK_GAIN_NUM)
#define MIN_COOR_BOUND 0

/* Enums/Types */
typedef union {
	double roll_state_arr[ROLL_STATE_NUM];

	struct {
		double roll_angle;
		double roll_rate;
		double canard_angle;
	};
} roll_state_t;

// input from state estimation module
typedef struct {
	// Roll state
	roll_state_t roll_state;
	// Scheduling variables (flight condition)
	double pressure_dynamic;
	double canard_coeff;
} controller_input_t;

// Output of controller: latest commanded canard angle
typedef struct {
	double commanded_angle; // radians
	uint32_t timestamp; // ms
} controller_output_t;

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
	controller_output_t cmd_output;
	controller_input_t new_state;
	// both of the following bool should be revaluated if they should still exisits with the new
	// system design
	bool state_updated;
	bool cmd_updated;

	uint32_t last_run_ms; // last time the controller did a full loop. use for rate-limiting
} controller_ctx_t;

#include "application/flight_phase/flight_phase.h"

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void);

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state);

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output);

/**
 * @brief run 1 step of the controller
 * @param context pointer to controller global context
 * @param curr_flight_phase current flight phase state
 * @param act_allowed_timestamp_ms the timestamp at which act_allowed was set (only used when passed
 * actuation allowed)
 * @param curr_timestamp_ms the currrent timestamp
 */
w_status_t controller_step(controller_ctx_t *context, const flight_phase_state_t curr_flight_phase,
						   const uint32_t act_allowed_timestamp_ms,
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
uint32_t controller_get_status(void);

#endif // CONTROLLER_H_
