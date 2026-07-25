#include "ak45_driver_test_helpers.h"
#include "drivers/ak45_driver/ak45_driver.c" // Extension of the src file.
#include <stdint.h>

void ak45_test_reset(void) {
	g_ak45_hfdcan = NULL;
	g_feedback_queue = NULL;
	g_tx_errors = 0;
	is_init = false;
	received_can_msg = false;
}

void ak45_set_tx_errors(uint32_t count) {
	g_tx_errors = count;
}

w_status_t ak45_test_angle_telemetry(void) {
	return ak45_driver_angle_telemetry();
}

w_status_t ak45_test_temperature_telemetry(void) {
	return ak45_driver_temperature_telemetry();
}

w_status_t ak45_test_current_telemetry(void) {
	return ak45_driver_current_telemetry();
}