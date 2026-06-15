#include "ak45_driver_test_helpers.h"
#include "drivers/ak45_driver/ak45_driver.c"
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