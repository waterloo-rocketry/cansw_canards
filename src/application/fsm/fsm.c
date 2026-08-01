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
#include "drivers/timer/timer.h"
#ifdef HIL
#include "application/hil/hil.h"
#include "drivers/gpio/gpio.h"
#endif

// TODO: remove after motor_handler implemented
/****************************************************************/
#include "common/math/math.h"
#include "drivers/ak45_driver/ak45_driver.h"
/****************************************************************/
#include "rocketlib/include/common.h"

extern TaskHandle_t fsm_task_handle;

#ifdef HIL
static const uint16_t MAX_FSM_DELAY_MS = 60000;
#else
static const uint8_t MAX_FSM_DELAY_MS = 4;
#endif

static const uint32_t MS_TO_TENTH_MS = 10;
static const uint8_t CONTROLLER_PERIOD_TENTH_MS = 100;

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

void unblock_fsm_hil() {
	unblock_fsm_loop(&htim5);
}

typedef struct {
	bool is_init;
	uint32_t init_timer_failures;
	uint32_t init_timer_start_failures;
	uint32_t loop_timeouts;
	uint32_t get_timer_failures;
	uint32_t unknown_state_errors;

	bool loop_timer_failed;
	bool get_timer_failed;
	bool is_in_unknown_state;
} fsm_health_t;

static fsm_health_t fsm_health = {0};

w_status_t fsm_init() {
	// init estimator context
	// initialize ctx timestamp to current time
	uint32_t init_time_tenth_ms = 0;
	if (timer_get_tenth_ms(&init_time_tenth_ms) != W_SUCCESS) {
		// TODO how to deal with error
		fsm_health.init_timer_failures++;
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
		fsm_health.init_timer_start_failures++;
		return W_FAILURE;
	}
#else
	// init hil here to keep all hil changes in fsm.c
	if (hil_init() != W_SUCCESS) {
		log_text(1, LOG_LVL_WARN, "HIL", "init fail");
		return W_FAILURE;
	}
#endif

	controller_codegen_init(g_ctx.p_controller_context);
	fsm_health.is_init = true;

	return W_SUCCESS;
}

fsm_state_t fsm_get_state() {
	return g_ctx.curr_state;
}

void fsm_exec(const fsm_input_t *p_fsm_input, const uint32_t timestamp_tenth_ms,
			  const fsm_ctx_t *p_ctx) {
#ifdef HIL
	gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_LOW, 0);
#endif

	// set the inputs
	navigator_input_t navigator_input = {.sensor_data = p_fsm_input->p_sensor_data,
										 .fsm_state = p_ctx->curr_state};
	controller_input_t controller_input = {.launch_timestamp_ms =
											   p_ctx->p_flight_phase_context->launch_timestamp_ms};

	// initialize the outputs
	navigator_output_t navigator_output = {0};
#ifdef HIL
	static controller_output_t controller_output = {0};
	bool ran_ctrl = false;
#else
	controller_output_t controller_output = {0};
#endif

	// calculate time elapsed since last controller run
	uint32_t ctrl_call_time_elapsed_tenth_ms =
		timestamp_tenth_ms - p_ctx->p_controller_context->last_run_tenth_ms;

	// TODO: ask tristan how to get behaviour of first cycle
	switch (p_ctx->curr_state) {
		case STATE_IDLE:
			p_ctx->p_navigator_context->last_run_tenth_ms = timestamp_tenth_ms;
			p_ctx->p_controller_context->last_run_tenth_ms = timestamp_tenth_ms;

			if (pad_filter_init(p_ctx->p_navigator_context, p_fsm_input->p_sensor_data) !=
				W_SUCCESS) {
				// TODO: add error handling
				log_text(0, LOG_LVL_WARN, "FSM", "pad_filter_init failed");
			}
			p_ctx->p_controller_context->last_run_tenth_ms = timestamp_tenth_ms;
			break;

		// both Pad filter and boost will only run estimator step
		case STATE_PAD_FILTER:
			// Nav enters pad filter
			/* fall through */
		case STATE_PAD_NAV:
			// Nav enters flight filter
			/* fall through */
		case STATE_BOOST:
#ifdef HIL
			gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_LOW, 0);
#endif
			navigator_step(&navigator_input,
						   timestamp_tenth_ms,
						   p_ctx->p_navigator_context,
						   &navigator_output);

			// input the navigator outputs into controller
			memcpy(controller_input.xR, navigator_output.roll_state, sizeof(controller_input.xR));
			controller_input.dynamic_pressure = navigator_output.dynamic_pressure;

			controller_input.canard_angle_rad = p_fsm_input->p_sensor_data->motor_encoder_meas.meas;

			// run controller at 100 hz
			if (p_fsm_input->p_sensor_data->motor_encoder_meas.is_new &&
				ctrl_call_time_elapsed_tenth_ms >= CONTROLLER_PERIOD_TENTH_MS) {
				controller_step(&controller_input,
								timestamp_tenth_ms,
								p_ctx->p_controller_context,
								&controller_output);
#ifdef HIL
				ran_ctrl = true;
#endif
			}

			// set motor command to zero in non-actuation state
			ak45_send_position_cmd(0);

			break;

		// both act allowed and recovery will only run estimator and controller step
		case STATE_ACT_ALLOWED:
		case STATE_RECOVERY:
#ifdef HIL
			if (timestamp_tenth_ms % 50 == 0) {
				gpio_toggle(GPIO_PIN_GREEN_LED, 0);
			}
#endif
			navigator_step(&navigator_input,
						   timestamp_tenth_ms,
						   p_ctx->p_navigator_context,
						   &navigator_output);

			// input the navigator outputs into controller
			memcpy(controller_input.xR, navigator_output.roll_state, sizeof(controller_input.xR));
			controller_input.dynamic_pressure = navigator_output.dynamic_pressure;

			controller_input.canard_angle_rad = p_fsm_input->p_sensor_data->motor_encoder_meas.meas;
			/****************************************************************/

			// run controller at 100 hz
			if (p_fsm_input->p_sensor_data->motor_encoder_meas.is_new &&
				ctrl_call_time_elapsed_tenth_ms >= CONTROLLER_PERIOD_TENTH_MS) {
				controller_step(&controller_input,
								timestamp_tenth_ms,
								p_ctx->p_controller_context,
								&controller_output);
#ifdef HIL
				ran_ctrl = true;
#endif

				// TODO: switch to motor handler once exists
				/****************************************************************/
				float32_t motor_angle_deg =
					(float32_t)(controller_output.canard_command_angle_rad * DEG_PER_RAD);
				ak45_send_position_cmd(motor_angle_deg);
				/****************************************************************/
			}
			break;

		// TODO Implement
		case STATE_SLEEPY:
			break;

		default:
			// TODO: how to deal with the other cases
			fsm_health.unknown_state_errors++;
			fsm_health.is_in_unknown_state = true;
			break;
	}

#ifdef HIL
	/******************************** HIL START ********************************/
	// send hil packet regardless of fsm state. In non-actuation states, we
	// still want to send telem to simulink (canard cmd gets ignored)
	w_status_t send_rc = hil_send_simulink_cmd(&navigator_input,
											   &navigator_output,
											   &p_ctx->p_navigator_context->gnc_navigator_ctx.x,
											   &p_ctx->p_controller_context->gnc_controller_ctx,
											   &controller_input,
											   &controller_output,
											   ran_ctrl);
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

	while (1) {
		// Unblock once we receive the notification to unblock fsm
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_FSM_DELAY_MS)) == 0) {
			fsm_health.loop_timeouts++;
			fsm_health.loop_timer_failed = true;
		}

		uint32_t timestamp_tenth_ms = 0;

		if (W_SUCCESS != timer_get_tenth_ms(&timestamp_tenth_ms)) {
			fsm_health.get_timer_failures++;
			fsm_health.get_timer_failed = true;
		}

		uint32_t timestamp_ms = timestamp_tenth_ms / MS_TO_TENTH_MS;

		all_sensors_data_t sensor_data = {0};

		// get inputs needed for state machine:
		// - imu data
		// - etc (probably more later)
		sensor_handler_get_fresh_meas(g_ctx.p_imu_context, &sensor_data);

#ifdef HIL
		gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_LOW, 0);
		/******************************** HIL START ********************************/
		// override sensor data with simulink sensors in HIL mode.
		// expect first run of this loop to fail, until hil statrs.
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
		g_ctx.curr_state = 7;

		// run actions based on new curr state
		fsm_input_t fsm_input = {.p_sensor_data = &sensor_data};
		fsm_exec(&fsm_input, timestamp_tenth_ms, &g_ctx);
	}
}

health_status_t fsm_get_status(void) {
	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_FSM,
							  .error_bitfield = 0};

	if (fsm_health.loop_timer_failed) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_LOOP_TIMING_OFFSET;
		fsm_health.loop_timer_failed = false;
	}

	if (fsm_health.get_timer_failed) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_INTERNAL_OFFSET;
		fsm_health.get_timer_failed = false;
	}

	if (fsm_health.is_in_unknown_state) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET;
		fsm_health.is_in_unknown_state = false;
	}

	if (!fsm_health.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_NOT_INIT_OFFSET;
	}

	log_text(10,
			 LOG_LVL_INFO,
			 "fsm",
			 "init=%d, loop_timeouts=%d, get_timer_failures=%d, unknown_state_errors=%d",
			 fsm_health.is_init,
			 fsm_health.loop_timeouts,
			 fsm_health.get_timer_failures,
			 fsm_health.unknown_state_errors);

	return status;
}
