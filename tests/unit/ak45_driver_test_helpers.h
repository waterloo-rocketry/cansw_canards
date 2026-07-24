#ifndef AK45_DRIVER_TEST_HELPERS
#define AK45_DRIVER_TEST_HELPERS

#include "third_party/rocketlib/include/common.h"
#include <stdint.h>

// Resets all static variables for testing
void ak45_test_reset(void);

// Sets number of errors to confirm that initialization resets the value
void ak45_set_tx_errors(uint32_t count);

// Exposing the driver's static telemetry functions for testing
w_status_t ak45_test_angle_telemetry(void);
w_status_t ak45_test_temperature_telemetry(void);
w_status_t ak45_test_current_telemetry(void);

#endif // AK45_DRIVER_TEST_HELPERS