#ifndef AK45_DRIVER_TEST_HELPERS
#define AK45_DRIVER_TEST_HELPERS

#include <stdint.h>

// Resets all static variables for testing
void ak45_test_reset(void);

// Sets number of errors to confirm that initialization resets the value
void ak45_set_tx_errors(uint32_t count);

#endif // AK45_DRIVER_TEST_HELPERS