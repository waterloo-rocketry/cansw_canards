#include "FreeRTOS.h"
#include "task.h"

#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/imu_handler/imu_handler.h"
#include "drivers/timer/timer.h"

static const uint8_t QUEUE_TIMEOUT_MS = 0;
// this is the max amount of flight phase events that will be processed from the flight phase
// queue (this does not count sensor base transitions)
#define MAX_PROCESS_FP_EVENTS 5
static const uint8_t FSM_PERIOD_MS = 2;

typedef struct {
	estimator_module_ctx_t *estimator_context; // global instance of estimator
	controller_ctx_t *p_controller_context; // global instance of controller
	const all_sensors_data_t *all_sensors_input; // imu data collected for this run
	uint32_t timestamp_ms; // curr timestamp
	fsm_state_t curr_state;
	flight_phase_ctx_t *p_flight_phase_context; // global instance of flight phase
} fsm_inputs_t;

// global
static fsm_inputs_t g_inputs = {0};

// create all of the global instances
static estimator_module_ctx_t g_estimator_context = {0};

// make sure controller_output_t is initalized to 0 and valid to read to match original design
static controller_ctx_t g_controller_context = {0};
static flight_phase_ctx_t g_flight_phase_context = {0};
static all_sensors_data_t g_all_sensors_input = {0};

w_status_t fsm_init() {
	// init estimator context
	// initialize ctx timestamp to current time
	uint32_t init_time_tenth_ms = 0;
	if (timer_get_tenth_ms(&init_time_tenth_ms) != W_SUCCESS) {
		// TODO how to deal with error
		return W_FAILURE;
	}
	g_estimator_context.t_sec =
		((float64_t)init_time_tenth_ms) / 10000.0; // convert 0.1ms to seconds
	g_estimator_context.x.attitude.w = 1.0;
	g_estimator_context.x.attitude.x = 0.0;
	g_estimator_context.x.attitude.y = 0.0;
	g_estimator_context.x.attitude.z = 0.0;
	g_estimator_context.x.altitude = 420;
	g_estimator_context.x.CL = 3;

	// init rest of input
	g_inputs.estimator_context = &g_estimator_context;
	g_inputs.p_controller_context = &g_controller_context;
	g_inputs.p_flight_phase_context = &g_flight_phase_context;
	g_inputs.all_sensors_input = &g_all_sensors_input;

	// initialize fsm state
	g_inputs.curr_state = STATE_IDLE;

	return W_SUCCESS;
}

void fsm_exec(const fsm_inputs_t *p_input) {
	navigator_input_t navigator_input = {};
	navigator_output_t navigator_output = {};
	controller_input_t controller_input = {0};
	controller_output_t controller_output = {0};

	// TODO: convert fsm_inputs into the nav/cntl inputs

	switch (p_input->curr_state) {
		case STATE_IDLE:
			// do stuff
			break;

		// both Pad filter and boost will only run estimator step
		case STATE_SE_INIT:
		// TODO: how to tell estimator it needs to pad filter
		case STATE_BOOST:
			estimator_step(p_input->estimator_context, &navigator_input, &navigator_output);
			break;

		// both act allowed and recovery will only run estimator and controller step
		case STATE_ACT_ALLOWED:
		case STATE_RECOVERY:
			estimator_step(p_input->estimator_context, &navigator_input, &navigator_output);

			controller_step(p_input->p_controller_context,
							&controller_input,
							&controller_output,
							p_input->p_flight_phase_context->act_allowed_timestamp_ms,
							p_input->timestamp_ms);
			// TODO: motor
			break;

			// etc for more cases...

		default:
			// TODO: how to deal with the other cases
			break;
	}
}

void fsm_do_transitions(fsm_inputs_t *p_input) {
	flight_phase_event_t event_array[MAX_PROCESS_FP_EVENTS] = {};

	// collect all of the events
	uint8_t current_index = 0;

	event_array[current_index] = flight_phase_timer_detection(
		p_input->p_flight_phase_context, p_input->curr_state, p_input->timestamp_ms);
	current_index++;

	event_array[current_index] = flight_phase_sensor_detection(
		p_input->p_flight_phase_context, p_input->curr_state, p_input->all_sensors_input);
	current_index++;

	// asnc events
	while (current_index < MAX_PROCESS_FP_EVENTS) {
		event_array[current_index] = flight_phase_get_queue_event(QUEUE_TIMEOUT_MS);
		current_index++;
	}

	// run the events and check for updates
	for (uint8_t i = 0; i < MAX_PROCESS_FP_EVENTS; i++) {
		p_input->curr_state = flight_phase_update_state(
			event_array[i], p_input->curr_state, p_input->p_flight_phase_context);
	}
}

void fsm_task(void *args) {
	(void)args;
	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {
		if (W_SUCCESS != timer_get_ms(&(g_inputs.timestamp_ms))) {
			// TODO: error handling
		}

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		// will type case bthe all sensor input pointer as this is the only function where sensor
		// data will be updated, while the rest should use const and therefore that should stay
		imu_handler_get_fresh_meas((all_sensors_data_t *)g_inputs.all_sensors_input);

		// do state machine transitions for a limited number of most recent events
		fsm_do_transitions(&g_inputs);

		// run actions based on curr state
		fsm_exec(&g_inputs);

		vTaskDelayUntil(&last_wake_time,
						pdMS_TO_TICKS(FSM_PERIOD_MS)); // state machine runs at 500 hz
	}
}
