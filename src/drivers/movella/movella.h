#ifndef MOVELLA_H
#define MOVELLA_H

#include <stdint.h>

#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"

typedef struct {
	vector3d_t acc; // (x, y, z) m/s^2
	vector3d_t gyr; // (x, y, z) rad/s
	vector3d_t euler; // (x, y, z) deg
	vector3d_t mag; // (x, y, z) "arbitrary units" - estimator doesnt need conversion so leave this

	uint32_t acc_timestamp_ms;
	uint32_t gyr_timestamp_ms;
	uint32_t euler_timestamp_ms;
	uint32_t mag_timestamp_ms;
	uint32_t pres_timestamp_ms;
	uint32_t temp_timestamp_ms;

	float pres; // Pa
	float temp; // °c
	bool is_dead; // true if detected dead - ie, no uart comms within UART_RX_TIMEOUT_MS
} movella_data_t;

typedef struct {
	uint32_t double_init;
	uint32_t init_null_mutex;
	uint32_t get_data_null_out_param;
	uint32_t get_data_not_init;
	uint32_t get_data_failed_take_mutex;
	uint32_t event_callback_timer_fail;
} movella_health_t;

// Initialize the xsens interface, pass the configuration to the sensor
w_status_t movella_init(void);

// Return a copy structure of the latest received movella_data_t
w_status_t movella_get_data(movella_data_t *out_data, uint32_t timeout_ms);

// FreeRTOS task function
void movella_task(void *parameters);
#endif
