/**
 * Types from Controller used globally
 */
#ifndef CONTROLLER_TYPES_H
#define CONTROLLER_TYPES_H

#include <stdint.h>

#include "application/controller/gain_table.h"

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1) // subtract 1 for the pre-gain
#define ROLL_STATE_NUM (FEEDBACK_GAIN_NUM)
#define MIN_COOR_BOUND 0

/**
 * context/memory type structs
 */
typedef struct controller_ctx_t controller_ctx_t;

/**
 * data storage (format) structs
 */

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

#endif
