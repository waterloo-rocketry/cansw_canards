#ifndef HIL_H
#define HIL_H

#include <stdint.h>

#include "common/gnc/gnc_types.h"
#include "rocketlib/include/common.h"
#include "GNC_codegen_types.h"

#ifdef HIL
// This is to prevent unintentionally building in HIL mode. Devs must explicitly allow HIL!
// #warning "HIL mode enabled! Comment out this line while working on HIL then uncomment when done."

extern volatile uint32_t hil_timestamp_tenth_ms; // HIL timestamp in 0.1 ms units

extern void unblock_fsm_hil(void);

/**
 * @brief Initialize HIL module context.
 */
w_status_t hil_init(void);

/**
 * @brief Parse one USB RX byte into fixed-size HIL packet state
 */
void hil_parse_rx_bytes(uint8_t byte);

/**
 * @brief Copy latest parsed HIL sensor packet into GNC sensor struct
 */
w_status_t hil_wait_for_simulink_data(all_sensors_data_t *out);

/**
 * @brief Build and send cmd+telem packet to simulink over USB CDC
 */
w_status_t hil_send_simulink_cmd(navigator_input_t *p_nav_in, navigator_output_t *p_nav_out,
								 gnc_x_state_t *p_x_state, gnc_controller_ctx_t *p_controller_ctx, controller_input_t *p_cntl_in,
								 controller_output_t *p_cntl_out);

#endif // HIL

#endif // HIL_H
