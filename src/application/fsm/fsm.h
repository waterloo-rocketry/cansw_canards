#ifndef FSM_H
#define FSM_H

#include "application/controller/controller.h"
#include "application/flight_phase/flight_phase.h"
#include "application/navigator/navigator.h"
#include "application/sensor_handler/sensor_handler.h"
#include "common/gnc/gnc_types.h"
#include "rocketlib/include/common.h"

typedef struct {
	navigator_ctx_t *p_navigator_context; // global instance of estimator
	controller_ctx_t *p_controller_context; // global instance of controller
	fsm_state_t curr_state;
	flight_phase_ctx_t *p_flight_phase_context; // global instance of flight phase
	sensor_handler_ctx_t *p_imu_context; // global instance of flight phase
	GNC_codegenStackData *p_codegen_stack_data;
} fsm_ctx_t;

typedef struct {
	all_sensors_data_t *p_sensor_data;
} fsm_input_t;

/**
 * @brief init fsm
 */
w_status_t fsm_init();

/**
 * @brief get current fsm state
 * @return the current fsm state
 */
fsm_state_t fsm_get_state();

/**
 * run in 500 hz freertos task
 */
void fsm_task(void *args);

#endif
