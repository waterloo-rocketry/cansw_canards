#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "hal_fdcan_mock.h"

DEFINE_FFF_GLOBALS; // this must be called within the extern c block
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
}

class AK45DriverTest : public ::testing::Test {
protected:
	void SetUp() override {
		// Reset all fakes before each test, for example:
		RESET_FAKE(HAL_FDCAN_Start);
		RESET_FAKE(HAL_FDCAN_Stop);
		RESET_FAKE(HAL_FDCAN_AddMessageToTxFifoQ);
		RESET_FAKE(HAL_FDCAN_ConfigFilter);
		RESET_FAKE(HAL_FDCAN_ActivateNotification);
		RESET_FAKE(HAL_FDCAN_GetRxMessage);
		RESET_FAKE(log_text);
		RESET_FAKE(timer_get_ms);

		FFF_RESET_HISTORY();
	}

	void TearDown() override {}
};

// Test example
TEST_F(AK45DriverTest, InitSucceeds) {
	// Arrange
	// Set up any necessary variables, mocks, etc

	// Act
	// Call the function to be tested

	// Assert
	// Verify the expected behavior of the above Act
	EXPECT_EQ(1, 1); // Example assertion
}
