// Shall enable or disable the 5V external payload rail in response to a CANARD_5V_OUTPUT actuator CAN command from the operator.
// Shall enable or disable low power mode in response to a CANARD_LIPO_ONactuator CAN command from the operator.
// Shall default to lipo on
// Shall default to ext 5v on
// Shall ensure 5V external is NOT on when board is powered by rocket CHG (ie, when rocket CHG is the highest voltage).
// Shall continuously read and report that power rail voltage are within threshold values to rocketCAN. (health checks) 
// Shall continuously read and report that battery voltages and currents are within threshold values to rocketCAN. 
// Shall confirm that LiPos have been successfully enabled/disabled. 
// Shall notify other systems via RocketCAN when power fault occurs
// Shall report rail voltage and current to RocketCAN health checks
// Shall trigger system reset when receives RESET_CMD from rocketCAN.
// Shall not allow external 5V when low power mode is enabled
// CHG_MUX_EN gpio must default to HIGH
// CHG_MUX_EN must be set LOW at all times when EN_EXT_5V is HIGH
// Shall turn off lipos when low power mode is active
// should do other power-saving things when low power mode is active... TODO
// should send a CAN msg informing us about which power source is highest voltage (ie the one being used)
// Shall log_text informing us about which power source is highest voltage (ie the one being used)
// Shall NOT report any FATAL errors from power health checks
// Shall use the PG_EXT_5V gpio pin to check if our 5v external supply is good or bad for health checks (see schematic)

#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
    #include "FreeRTOS.h"
    #include "queue.h"
    #include "task.h"
    #include "timers.h"
// add includes like freertos, hal, proc headers, etc
DEFINE_FFF_GLOBALS; // this must be called within the extern c block
}

class power_handler_test : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(power_handler_test, CANARD_5V_OUTPUT_ON) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

TEST_F(power_handler_test, CANARD_5V_OUTPUT_OFF) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Test example
TEST_F(power_handler_test, CANARD_LIPO_ON_CMD) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Test example
TEST_F(power_handler_test, DEFAULT_STATE_CORRECTNESS) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

