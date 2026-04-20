#include "application/state_machine/state_machine.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/imu_handler/imu_handler.h"

typedef struct {
	estimator_module_ctx_t *estimator_context; // global instance of estimator
	controller_ctx_t *controller_context; // global instance of controller
	const all_sensors_data_t *all_sensors_input; // imu data collected for this run
	uint32_t timestamp_ms; // curr timestamp
	flight_phase_state_t curr_state; // curr flight state
} state_machine_inputs_t;

void state_machine_exec(const state_machine_inputs_t *input) {
	switch (input->curr_state) {
		case STATE_IDLE:
			// do stuff
			break;
		case STATE_SE_INIT:
			estimator_step(input->estimator_context, 0); // (ignore loop_count var for now)
			break;
		case STATE_ACT_ALLOWED:
			estimator_step(input->estimator_context, 0); // (ignore loop_count var for now)
			controller_step(input->controller_context);
			// motor maybe
			break;

			// etc for more cases...
	}
}

void state_machine_task(void *args) {
	(void)args;
	while (1) {
		state_machine_inputs_t inputs = {0};

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		imu_handler_run(0, &inputs.all_sensors_input); // (ignore loop_count param for now)

		// do state machine transitions for a limited number of most recent events
		flight_phase_event_t event = EVENT_NONE;
		flight_phase_update_state(event, &inputs.curr_state);

		// run actions based on curr state
		state_machine_exec(&inputs);
	}
}
