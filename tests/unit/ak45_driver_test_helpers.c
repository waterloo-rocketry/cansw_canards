#include "ak45_driver_test_helpers.h"
#include "drivers/ak45_driver/ak45_driver.c" // Extension of the src file.
#include <stdint.h>

void ak45_test_reset(void) {
	g_ak45_hfdcan = NULL;
	g_feedback_queue = NULL;
	ak45_health = (ak45_health_t){0};
	received_can_msg = false;
}

void ak45_set_tx_errors(uint32_t count) {
	ak45_health.tx_errors = count;
}

w_status_t ak45_test_angle_telemetry(void) {
	return ak45_driver_angle_telemetry();
}

w_status_t ak45_test_temp_curr_telemetry(void) {
	return ak45_driver_temp_curr_telemetry();
}