#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include "tim.h"

#include "GNC_codegen.h"
#include "GNC_codegen_types.h"
#include "application/controller/controller.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "application/navigator/navigator.h"
#include "application/sensor_handler/sensor_handler.h"
#include "drivers/gpio/gpio.h"
#include "drivers/timer/timer.h"
#ifdef HIL
#include "application/hil/hil.h"
#endif

// TODO: remove after motor_handler implemented
/****************************************************************/
#include "common/math/math.h"
#include "drivers/ak45_driver/ak45_driver.h"
/****************************************************************/
#include "rocketlib/include/common.h"

extern TaskHandle_t fsm_task_handle;

#ifdef HIL
static const uint16_t MAX_FSM_DELAY_MS = 15000;
#else
static const uint8_t MAX_FSM_DELAY_MS = 4;
#endif

static const uint32_t MS_TO_TENTH_MS = 10;

// global
static fsm_ctx_t g_ctx = {0};

// create all of the global instances
static navigator_ctx_t g_navigator_context = {0};

// make sure controller_output_t is initalized to 0 and valid to read to match original design
static controller_ctx_t g_controller_context = {0};
// setting the launch and act_allowed time to MAX to make sure of no inadvertent actuation
static flight_phase_ctx_t g_flight_phase_context = {.launch_timestamp_ms = UINT32_MAX,
													.act_allowed_timestamp_ms = UINT32_MAX};
static sensor_handler_ctx_t g_imu_context = {0};

// gnc context
static GNC_codegenPersistentData g_gnc_code_persistent = {0};
static GNC_codegenStackData g_gnc_codegen_data = {.pd = &g_gnc_code_persistent};

static void unblock_fsm_loop(TIM_HandleTypeDef *htim) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// check if FSM task has started yet
	if ((&htim5 == htim) && (fsm_task_handle != NULL)) {
		vTaskNotifyGiveFromISR(fsm_task_handle, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#ifdef HIL
void unblock_fsm_hil() {
	unblock_fsm_loop(&htim5);
}
#endif

w_status_t fsm_init() {
	// init estimator context
	// initialize ctx timestamp to current time
	uint32_t init_time_tenth_ms = 0;
	if (timer_get_tenth_ms(&init_time_tenth_ms) != W_SUCCESS) {
		// TODO how to deal with error
		return W_FAILURE;
	}

	// initialize gnc
	GNC_codegen_initialize(&g_gnc_codegen_data);

	// init the stack data
	g_ctx.p_codegen_stack_data = &g_gnc_codegen_data;
	g_navigator_context.p_gnc_stack_data = &g_gnc_codegen_data;
	g_controller_context.p_gnc_stack_data = &g_gnc_codegen_data;

	// init rest of input
	g_ctx.p_navigator_context = &g_navigator_context;
	g_ctx.p_controller_context = &g_controller_context;
	g_ctx.p_flight_phase_context = &g_flight_phase_context;
	g_ctx.p_imu_context = &g_imu_context;

	// initialize fsm state
	g_ctx.curr_state = STATE_IDLE;
#ifndef HIL
	HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, &unblock_fsm_loop);

	// start tim
	if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK) {
		return W_FAILURE;
	}

#else
	// init hil here to keep all hil changes in fsm.c
	if (hil_init() != W_SUCCESS) {
		log_text(1, LOG_LVL_WARN, "HIL", "init fail");
		return W_FAILURE;
	}
#endif

	return W_SUCCESS;
}

fsm_state_t fsm_get_state() {
	return g_ctx.curr_state;
}

void fsm_exec(const fsm_input_t* p_fsm_input, const uint32_t timestamp_tenth_ms, const fsm_ctx_t *p_ctx) {
	gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_LOW, 0);

	// set the inputs
	navigator_input_t navigator_input = {.sensor_data = p_fsm_input->p_sensor_data,
										 .fsm_state = p_ctx->curr_state};
	controller_input_t controller_input = {.launch_timestamp_ms =
											   p_ctx->p_flight_phase_context->launch_timestamp_ms};

	// initialize the outputs
	navigator_output_t navigator_output = {0};
	controller_output_t controller_output = {0};

	// TODO: ask tristan how to get behaviour of first cycle
	switch (p_ctx->curr_state) {
		case STATE_IDLE:
			p_ctx->p_navigator_context->last_run_tenth_ms = timestamp_tenth_ms;
			break;

			// both Pad filter and boost will only run estimator step
		case STATE_PAD_FILTER:
			// Nav enters pad filter
			/* fall through */
		case STATE_PAD_NAV:
			// Nav enters flight filter
			/* fall through */
		case STATE_BOOST:
			gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_LOW, 0);
			navigator_step(&navigator_input,
						   timestamp_tenth_ms,
						   p_ctx->p_navigator_context,
						   &navigator_output);
			p_ctx->p_controller_context->last_run_tenth_ms = timestamp_tenth_ms;
			break;

			// both act allowed and recovery will only run estimator and controller step
		case STATE_ACT_ALLOWED:
		case STATE_RECOVERY:
			if (timestamp_tenth_ms % 50 == 0) {
				gpio_toggle(GPIO_PIN_GREEN_LED, 0);
			}
			navigator_step(&navigator_input,
						   timestamp_tenth_ms,
						   p_ctx->p_navigator_context,
						   &navigator_output);

			// input the navigator outputs into controller
			memcpy(controller_input.xR, navigator_output.roll_state, sizeof(controller_input.xR));
			controller_input.dynamic_pressure = navigator_output.dynamic_pressure;

			controller_input.canard_angle_rad = p_fsm_input->p_sensor_data->motor_encoder_meas.meas;
			/****************************************************************/

			if (p_fsm_input->p_sensor_data->motor_encoder_meas.is_new) {
				controller_step(&controller_input,
								timestamp_tenth_ms,
								p_ctx->p_controller_context,
								&controller_output);

				// TODO: switch to motor handler once exists
				/****************************************************************/
				float32_t motor_angle_deg =
					(float32_t)(controller_output.canard_command_angle_rad * DEG_PER_RAD);
				ak45_send_position_cmd(motor_angle_deg);
				/****************************************************************/
			}
			break;

			// etc for more cases...
		case STATE_SLEEPY:
			break;

		default:
			// TODO: how to deal with the other cases
			break;
	}

#ifdef HIL
	/******************************** HIL START ********************************/
	// send hil packet regardless of fsm state. In non-actuation states, we
	// still want to send telem to simulink (canard cmd gets ignored)
	w_status_t send_rc = hil_send_simulink_cmd(&navigator_input,
											   &navigator_output,
											   &p_ctx->p_navigator_context->gnc_navigator_ctx.x,
											   &controller_input,
											   &controller_output);
	gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 0);
	if (send_rc != W_SUCCESS) {
		log_text(1, LOG_LVL_WARN, "HIL", "Failed to send cmd to simulink: %d", send_rc);
	} else {
		log_text(1,
				 LOG_LVL_DEBUG,
				 "HIL",
				 "Sent cmd to simulink %f, %f, %f",
				 controller_output.canard_command_angle_rad,
				 controller_output.ref_roll[0],
				 controller_output.ref_roll[1]);
	}
	/******************************** HIL END ********************************/
#endif
}

void fsm_task(void *args) {
	(void)args;

#ifdef HIL
	// kickoff simulink by unblocking it with 1 msg. use dummy data to verify it started
	navigator_input_t navigator_input = {0};
	controller_input_t controller_input = {0};
	navigator_output_t navigator_output = {0};
	controller_output_t controller_output = {0};
	gnc_x_state_t x_state = {0};
	controller_input.canard_angle_rad = 123.456;
	navigator_output.roll_state[0] = 567.567;

	hil_send_simulink_cmd(
		&navigator_input, &navigator_output, &x_state, &controller_input, &controller_output);
#endif

	while (1) {
		// Unblock once we receive the notification to unblock fsm
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_FSM_DELAY_MS)) == 0) {
			log_text(0, LOG_LVL_WARN, "FSM", "FSM loop wait timed out");
		}

		uint32_t timestamp_tenth_ms = 0;

		if (W_SUCCESS != timer_get_tenth_ms(&timestamp_tenth_ms)) {
			// TODO: error handling
		}

		uint32_t timestamp_ms = timestamp_tenth_ms / MS_TO_TENTH_MS;

		all_sensors_data_t sensor_data = {0};

		// TODO: decide how to deal with a function returning an error

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		sensor_handler_get_fresh_meas(g_ctx.p_imu_context, &sensor_data);

		#ifdef HIL
			gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_LOW, 0);
			/******************************** HIL START ********************************/
			// override sensor data with simulink sensors in HIL mode.
			// expect first run of this loop to fail, as we havent unblocked hil 1st step yet.
			// but fsm_exec() unconditionally sends cmd+telem to simulink, so next loop will have data
			// ready.
			if (hil_wait_for_simulink_data(&sensor_data) != W_SUCCESS) {
				log_text(1, LOG_LVL_WARN, "HIL", "Failed to get latest sensor data");
			}
			/******************************** HIL END ********************************/
			gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 0);
		#endif

		flight_phase_gen_sync_events(
			g_ctx.p_flight_phase_context, g_ctx.curr_state, timestamp_ms, &sensor_data);

		// run 1 cycle of state transition
		flight_phase_event_t next_event = flight_phase_get_next_event();
		fsm_state_t new_state =
			flight_phase_update_state(next_event, g_ctx.curr_state, g_ctx.p_flight_phase_context);
		g_ctx.curr_state = new_state;

		// run actions based on new curr state
		fsm_input_t fsm_input = {.p_sensor_data = &sensor_data};
		fsm_exec(&fsm_input, timestamp_tenth_ms, &g_ctx);
	}
}
