#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "third_party/rocketlib/include/common.h"

#include "GNC_codegen_types.h"
#include "application/health_checks/health_checks.h"
#include "common/gnc/gnc_types.h"

/**
 * persistent state updated by navigator and fsm
 */
typedef struct {
	uint32_t last_run_tenth_ms; // previous timestamp
	gnc_navigator_ctx_t gnc_navigator_ctx;
	GNC_codegenStackData *p_gnc_stack_data;
} navigator_ctx_t;

/**
 * @brief initialize navigator module
 */
w_status_t navigator_init(void);

// TODO: to be REVIVED
// /**
//  * @brief Sends the complete state estimation data over CAN.
//  *
//  * Iterates through each state ID, builds a CAN message for it using the
//  * current state data, and transmits it.
//  *
//  * @param current_state Pointer to the current state estimation data (x_state_t).
//  * @return W_SUCCESS if all messages were sent successfully, W_FAILURE otherwise.
//  */
// w_status_t navigator_log_state_to_can(const x_state_t *current_state);

/**
 * @brief Report navigator module health status
 *
 * Retrieves and reports navigator error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t navigator_get_status(void);

/**
 * @brief 1 step of navigator
 * @param p_input pointer to the new navigator input
 * @param timestamp_tenth_ms is the current timestamp in tenth of a ms
 * @param p_ctx pointer to navigator context
 * @param p_output pointer to navigator output to update with new results
 * update with new actuation info
 */
w_status_t navigator_step(const navigator_input_t *p_input, const uint32_t timestamp_tenth_ms,
						  navigator_ctx_t *p_ctx, navigator_output_t *p_output);

/**
 * @brief init pad filter with alive sensors. Must call before pad filter starts. Can call this
 * multiple times to overwrite with new sensor values. Dead sensors will persist previous values.
 * @param p_ctx pointer to navigator context
 * @param p_sensor_data sensor data
 */
w_status_t pad_filter_init(navigator_ctx_t *p_ctx, all_sensors_data_t *p_sensor_data);

#endif
