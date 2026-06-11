#include "FreeRTOS.h"
#include "task.h"

#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/imu_handler/imu_handler.h"
#include "drivers/timer/timer.h"

static const uint8_t FSM_PERIOD_MS = 2;

typedef struct {
	estimator_module_ctx_t *estimator_context; // global instance of estimator
	controller_ctx_t *p_controller_context; // global instance of controller
	uint32_t timestamp_ms; // curr timestamp
	fsm_state_t curr_state;
	flight_phase_ctx_t *p_flight_phase_context; // global instance of flight phase
} fsm_ctx_t;

// global
static fsm_ctx_t g_ctx = {0};

// create all of the global instances
static estimator_module_ctx_t g_estimator_context = {0};

// make sure controller_output_t is initalized to 0 and valid to read to match original design
static controller_ctx_t g_controller_context = {0};
// setting the launch and act_allowed time to MAX to make sure of no inadvertent actuation
static flight_phase_ctx_t g_flight_phase_context = {.launch_timestamp_ms = UINT32_MAX,
													.act_allowed_timestamp_ms = UINT32_MAX};

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
	g_estimator_context.x.attitude = (quaternion_t){.w = 1.0, .x = 0.0, .y = 0.0, .z = 0.0};
	g_estimator_context.x.altitude = 420;
	g_estimator_context.x.CL = 3;

	// init rest of input
	g_ctx.estimator_context = &g_estimator_context;
	g_ctx.p_controller_context = &g_controller_context;
	g_ctx.p_flight_phase_context = &g_flight_phase_context;

	// initialize fsm state
	g_ctx.curr_state = STATE_IDLE;

	return W_SUCCESS;
}

void fsm_exec(const fsm_ctx_t *p_ctx, const all_sensors_data_t *p_sensor_data) {
	(void)p_sensor_data;
	// can't init to {0} as don't have any fields
	// TODO: either init to {0} or with some value once implemented
	navigator_input_t navigator_input = {};
	navigator_output_t navigator_output = {};
	controller_input_t controller_input = {0};
	controller_output_t controller_output = {0};

	// TODO: convert fsm_inputs into the nav/cntl inputs

	switch (p_ctx->curr_state) {
		case STATE_IDLE:
			// do stuff
			break;

			// both Pad filter and boost will only run estimator step
		case STATE_PAD_FILTER:
			// TODO: how to tell estimator it needs to pad filter
			/* fall through */
		case STATE_PAD_NAV:
			// TODO: enter into flight filter
			/* fall through */
		case STATE_BOOST:
			estimator_step(p_ctx->estimator_context, &navigator_input, &navigator_output);
			break;

			// both act allowed and recovery will only run estimator and controller step
		case STATE_ACT_ALLOWED:
		case STATE_RECOVERY:
			estimator_step(p_ctx->estimator_context, &navigator_input, &navigator_output);

			controller_step(p_ctx->p_controller_context,
							&controller_input,
							&controller_output,
							p_ctx->p_flight_phase_context->act_allowed_timestamp_ms,
							p_ctx->timestamp_ms);
			// TODO: motor
			break;

			// etc for more cases...
		case STATE_SLEEPY:
			break;

		default:
			// TODO: how to deal with the other cases
			break;
	}
}

void fsm_task(void *args) {
	(void)args;
	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {
		if (W_SUCCESS != timer_get_ms(&(g_ctx.timestamp_ms))) {
			// TODO: error handling
		}

		all_sensors_data_t sensor_data = {0};

		// TODO: decide how to deal with a function returning an error

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		imu_handler_get_fresh_meas(&sensor_data);

		flight_phase_gen_sync_events(
			g_ctx.p_flight_phase_context, g_ctx.curr_state, g_ctx.timestamp_ms, &sensor_data);

		// run 1 cycle of state transition
		flight_phase_event_t next_event = flight_phase_get_next_event();
		fsm_state_t new_state =
			flight_phase_update_state(next_event, g_ctx.curr_state, g_ctx.p_flight_phase_context);
		g_ctx.curr_state = new_state;

		// run actions based on new curr state
		fsm_exec(&g_ctx, &sensor_data);

		// state machine runs at 500 hz
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FSM_PERIOD_MS));
	}
}
