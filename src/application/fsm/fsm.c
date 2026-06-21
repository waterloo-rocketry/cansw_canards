#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include "tim.h"

#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/imu_handler/imu_handler.h"
#include "drivers/timer/timer.h"
#include "rocketlib/include/common.h"

extern TaskHandle_t fsm_task_handle;

static const uint8_t MAX_FSM_DELAY_MS = 4;

typedef struct {
	estimator_module_ctx_t *estimator_context; // global instance of estimator
	controller_ctx_t *p_controller_context; // global instance of controller
	uint32_t timestamp_ms; // curr timestamp
	fsm_state_t curr_state;
	flight_phase_ctx_t *p_flight_phase_context; // global instance of flight phase
	imu_handler_ctx_t *p_imu_context; // global instance of flight phase
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
static imu_handler_ctx_t g_imu_context = {0};

static void unblock_fsm_loop(TIM_HandleTypeDef *htim) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// check if FSM task has started yet
	if ((&htim5 == htim) && (fsm_task_handle != NULL)) {
		vTaskNotifyGiveFromISR(fsm_task_handle, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
	g_ctx.p_imu_context = &g_imu_context;

	// initialize fsm state
	g_ctx.curr_state = STATE_IDLE;

	HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, &unblock_fsm_loop);

	// start tim
	if (HAL_OK != HAL_TIM_Base_Start_IT(&htim5)) {
		return W_FAILURE;
	}

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
		// Unblock once we recieve the notification to unblock fsm
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_FSM_DELAY_MS));

		if (W_SUCCESS != timer_get_ms(&(g_ctx.timestamp_ms))) {
			// TODO: error handling
		}

		all_sensors_data_t sensor_data = {0};

		// TODO: decide how to deal with a function returning an error

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		imu_handler_get_fresh_meas(g_ctx.p_imu_context, &sensor_data);

		flight_phase_gen_sync_events(
			g_ctx.p_flight_phase_context, g_ctx.curr_state, g_ctx.timestamp_ms, &sensor_data);

		// run 1 cycle of state transition
		flight_phase_event_t next_event = flight_phase_get_next_event();
		fsm_state_t new_state =
			flight_phase_update_state(next_event, g_ctx.curr_state, g_ctx.p_flight_phase_context);
		g_ctx.curr_state = new_state;

		// run actions based on new curr state
		fsm_exec(&g_ctx, &sensor_data);
	}
}
