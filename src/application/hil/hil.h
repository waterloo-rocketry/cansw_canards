#ifndef HIL_H
#define HIL_H

#include <stdint.h>

#include "common/gnc/gnc_types.h"
#include "rocketlib/include/common.h"

#ifdef HIL
// This is to prevent unintentionally building in HIL mode. Devs must explicitly allow HIL!
// #warning "HIL mode enabled! Comment out this line while working on HIL then uncomment when done."
#endif

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
w_status_t hil_get_latest_sensor_data(all_sensors_data_t *out);

/**
 * @brief Build and send cmd+telem packet to simulink over USB CDC
 */
w_status_t hil_send_simulink_cmd(float64_t canard_cmd_rad);

#endif // HIL_H
