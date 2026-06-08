#include "fff.h"
#include <gtest/gtest.h>
#include "application/power_handler/power_handler.h"

extern "C" {
    #include "FreeRTOS.h"
    #include "queue.h"
    #include "task.h"
    #include "timers.h"

// GPIO
FAKE_VALUE_FUNC(w_status_t, gpio_write,
                gpio_pin_t, gpio_level_t, uint32_t);

FAKE_VALUE_FUNC(w_status_t, gpio_read,
                gpio_pin_t, gpio_level_t*, uint32_t);

// ADC
FAKE_VALUE_FUNC(w_status_t, adc_get_converted_val,
                adc_channel_t, uint32_t*);

// CAN
FAKE_VALUE_FUNC(w_status_t,
                can_handler_register_callback,
                can_msg_type_t,
                w_status_t (*)(const can_msg_t *));

FAKE_VALUE_FUNC(w_status_t,
                can_handler_transmit,
                const can_msg_t *);

FAKE_VALUE_FUNC(w_status_t,
                get_actuator_id,
                const can_msg_t *,
                can_actuator_id_t *);

FAKE_VALUE_FUNC(w_status_t,
                get_cmd_actuator_state,
                const can_msg_t *,
                can_actuator_state_t *);

FAKE_VALUE_FUNC(w_status_t,
                get_reset_board_id,
                const can_msg_t *,
                uint8_t *,
                uint8_t *);

FAKE_VALUE_FUNC(w_status_t,
                check_board_need_reset,
                const can_msg_t *,
                bool *);

FAKE_VALUE_FUNC(w_status_t,
                build_actuator_status_msg,
                can_priority_t,
                uint16_t,
                can_actuator_id_t,
                can_actuator_state_t,
                can_actuator_state_t,
                can_msg_t *);

FAKE_VALUE_FUNC(w_status_t,
                build_analog_sensor_16bit_msg,
                can_priority_t,
                uint16_t,
                can_sensor_id_t,
                uint32_t,
                can_msg_t *);

FAKE_VALUE_FUNC(w_status_t,
                build_general_board_status_msg,
                can_priority_t,
                uint16_t,
                uint32_t,
                can_msg_t *);

// Timer
FAKE_VALUE_FUNC(w_status_t,
                timer_get_ms,
                uint32_t *);

// Reset spy
FAKE_VOID_FUNC(NVIC_SystemReset);

DEFINE_FFF_GLOBALS;
}

class power_handler_test : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(gpio_write);
        RESET_FAKE(gpio_read);
        RESET_FAKE(adc_get_converted_val);
        RESET_FAKE(can_handler_register_callback);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(get_actuator_id);
        RESET_FAKE(get_cmd_actuator_state);
        RESET_FAKE(get_reset_board_id);
        RESET_FAKE(check_board_need_reset);
        RESET_FAKE(build_actuator_status_msg);
        RESET_FAKE(build_analog_sensor_16bit_msg);
        RESET_FAKE(build_general_board_status_msg);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(NVIC_SystemReset);

        FFF_RESET_HISTORY();

        gpio_write_fake.return_val = W_SUCCESS;
        gpio_read_fake.return_val = W_SUCCESS;
        adc_get_converted_val_fake.return_val = W_SUCCESS;
        can_handler_register_callback_fake.return_val = W_SUCCESS;
        can_handler_transmit_fake.return_val = W_SUCCESS;

        actuator_cb = nullptr;
        reset_cb = nullptr;

        can_handler_register_callback_fake.custom_fake =
            register_callback_fake;
    }

    void TearDown() override {}
};


/******** */
/*rocket can cmd*/
/******** */
// Shall enable or disable the 5V external payload rail in response to a CANARD_5V_OUTPUT actuator CAN command from the operator.
TEST_F(power_handler_test, EN_5V_EXTERNAL_CMD) {
    // Arrange


    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

TEST_F(power_handler_test, DIS_5V_EXTERNAL_CMD) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall enable or disable low power mode in response to a CANARD_LIPO_ONactuator CAN command from the operator.
TEST_F(power_handler_test, EN_LIPO_CMD) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall trigger system reset when receives RESET_CMD from rocketCAN.
TEST_F(power_handler_test, EN_LIPO_CMD) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

/******** */
/*gpio states*/
/******** */

// Shall default to lipo on
TEST_F(power_handler_test, DEFAULT_LIPO_ON) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall confirm that LiPos have been successfully enabled/disabled.
TEST_F(power_handler_test, DEFAULT_LIPO_ON) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall default to ext 5v on
TEST_F(power_handler_test, DEFAULT_5V_EXTERNAL_ON) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// CHG_MUX_EN gpio must default to HIGH
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall ensure 5V external is NOT on when board is powered by rocket CHG (ie, when rocket CHG is the highest voltage).
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}
// CHG_MUX_EN must be set LOW at all times when EN_EXT_5V is HIGH
// Shall not allow external 5V when low power mode is enabled
// Shall turn off lipos when low power mode is active

/******** */
/*status reports*/
/******** */

// Shall continuously read and report that power rail voltage are within threshold values to rocketCAN. (health checks) 
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall continuously read and report that battery voltages and currents are within threshold values to rocketCAN. (health checks)
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall use the PG_EXT_5V gpio pin to check if our 5v external supply is good or bad for health checks (see schematic)
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall NOT report any FATAL errors from power health checks
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall notify other systems via RocketCAN when power fault occurs
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall report rail voltage and current to RocketCAN health checks
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

// Shall log_text informing us about which power source is highest voltage (ie the one being used)
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}