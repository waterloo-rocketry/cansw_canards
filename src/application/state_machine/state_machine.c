#include "application/state_machine/state_machine.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/imu_handler/imu_handler.h"
#include "drivers/timer/timer.h"

#include "FreeRTOS.h"
#include "task.h"

static uint8_t TASK_TIMEOUT_MS = 0;
static uint8_t MAX_PROCESS_FP_QUEUE_EVENTS =
	3; // this is the max amount of flight phase events that will be processed from the flight phase
	   // queue (this does not count sensor base transitions)

typedef struct {
	estimator_module_ctx_t *estimator_context; // global instance of estimator
	controller_ctx_t *controller_context; // global instance of controller
	const all_sensors_data_t *all_sensors_input; // imu data collected for this run
	uint32_t timestamp_ms; // curr timestamp
	flight_phase_ctx_t *flight_phase_context; // global instance of flight phase
} state_machine_inputs_t;

void state_machine_exec(const state_machine_inputs_t *p_input) {
	switch (p_input->flight_phase_context->curr_state) {
		case STATE_IDLE:
			// do stuff
			break;
		case STATE_SE_INIT:
			estimator_step(p_input->estimator_context,
						   p_input->flight_phase_context->curr_state,
						   p_input->all_sensors_input,
						   p_input->controller_context,
						   0); // (ignore loop_count var for now)
			break;
		case STATE_ACT_ALLOWED:
			estimator_step(p_input->estimator_context,
						   p_input->flight_phase_context->curr_state,
						   p_input->all_sensors_input,
						   p_input->controller_context,
						   0); // (ignore loop_count var for now)
			controller_step(p_input->controller_context,
							p_input->flight_phase_context->curr_state,
							p_input->flight_phase_context->act_allowed_timestamp_ms,
							p_input->timestamp_ms);
			// motor maybe
			break;

			// etc for more cases...

		default:
			// TODO: how to deal with the other cases
			break;
	}
}

void state_machine_do_transitions(state_machine_inputs_t *p_input) {
	// TODO: consider where event queue should live and how to access it

	// perform timer based state transitions
	flight_phase_event_t timer_event = EVENT_NONE;
	if (W_SUCCESS == flight_phase_timer_detection(
						 p_input->flight_phase_context, p_input->timestamp_ms, &timer_event)) {
		if (EVENT_NONE != timer_event) {
			if (W_SUCCESS !=
				flight_phase_update_state(timer_event, p_input->flight_phase_context)) {
				// TODO: error handling
			}
		}
	} else {
		// TODO: error handling
	}

	// empty state queue (or as much as possible) and perfrom state transitions
	flight_phase_event_t queue_event = EVENT_NONE;
	uint8_t num_events = 0;
	// since queue receive fails when there are nothing in the queue so that can be used here
	while ((num_events < MAX_PROCESS_FP_QUEUE_EVENTS) &&
		   (pdPASS == xQueueReceive(p_input->flight_phase_context->event_queue,
									&queue_event,
									pdMS_TO_TICKS(TASK_TIMEOUT_MS)))) {
		if (W_SUCCESS != flight_phase_update_state(queue_event, p_input->flight_phase_context)) {
			// TODO: error handling
		}
		num_events++;
	}

	// TODO: what should be fone if there are still events remaining after reaching the
	// MAX_PROCESS_FP_QUEUE_EVENTS

	// perform sensor transitions
	// this is done last to make sure any of the new async or timer events can be processed first
	flight_phase_event_t sensor_event = EVENT_NONE;
	if (W_SUCCESS == flight_phase_sensor_detection(p_input->flight_phase_context,
												   p_input->all_sensors_input,
												   &sensor_event)) {
		if (EVENT_NONE != sensor_event) {
			if (W_SUCCESS !=
				flight_phase_update_state(sensor_event, p_input->flight_phase_context)) {
				// TODO: error handling
			}
		}
	} else {
		// TODO: error handling
	}
}

void state_machine_task(void *args) {
	(void)args;
	TickType_t last_wake_time = xTaskGetTickCount();

	state_machine_inputs_t inputs = {0};

	// create all of the global instances
	estimator_module_ctx_t g_estimator_context = {0};
	controller_ctx_t g_controller_context = {0};
	flight_phase_ctx_t g_flight_phase_context = {0};

	// update estimator context
	// initialize ctx timestamp to current time
	uint32_t init_time_tenth_ms = 0;
	if (timer_get_tenth_ms(&init_time_tenth_ms) != W_SUCCESS) {
		// TODO how to deal with error
	}
	g_estimator_context.t_sec =
		((float64_t)init_time_tenth_ms) / 10000.0; // convert 0.1ms to seconds
	g_estimator_context.x.attitude.w = 1.0;
	g_estimator_context.x.attitude.x = 0.0;
	g_estimator_context.x.attitude.y = 0.0;
	g_estimator_context.x.attitude.z = 0.0;
	g_estimator_context.x.altitude = 420;
	g_estimator_context.x.CL = 3;

	inputs.estimator_context = &g_estimator_context;
	inputs.controller_context = &g_controller_context;
	inputs.flight_phase_context = &g_flight_phase_context;

	// consider how to establish

	while (1) {
		if (W_SUCCESS != timer_get_ms(&inputs.timestamp_ms)) {
			// TODO: error handling
		}

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		// will type case bthe all sensor input pointer as this is the only function where sensor
		// data will be updated, while the rest should use const and therefore that should stay
		imu_handler_get_fresh_meas(
			0,
			(all_sensors_data_t *)inputs.all_sensors_input); // (ignore loop_count param for now)

		// do state machine transitions for a limited number of most recent events
		state_machine_do_transitions(&inputs);

		// run actions based on curr state
		state_machine_exec(&inputs);

		vTaskDelayUntil(&last_wake_time, 2); // state machine runs at 500 hz
	}
}
