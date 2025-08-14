#include "fff.h"
#include <gtest/gtest.h>

#include "utils/mock_helpers.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/estimator/pad_filter.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "task.h"
#include "third_party/rocketlib/include/common.h"

extern w_status_t controller_run_loop();

DEFINE_FFF_GLOBALS;
}

// Define Fakes
// flight_phase_state_t flight_phase_get_state(void);
FAKE_VALUE_FUNC(flight_phase_state_t, flight_phase_get_state);
// bool get_analog_data(const can_msg_t *msg, can_analog_sensor_id_t *sensor_id, uint16_t *value);
FAKE_VALUE_FUNC(bool, get_analog_data, const can_msg_t *, can_analog_sensor_id_t *, uint16_t *);
FAKE_VOID_FUNC(proc_handle_fatal_error, const char *);
// w_status_t can_handler_transmit(const can_msg_t *msg);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
FAKE_VALUE_FUNC(w_status_t, flight_phase_get_act_allowed_ms, uint32_t *);
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
FAKE_VALUE_FUNC(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);
FAKE_VALUE_FUNC(
    bool, build_actuator_analog_cmd_msg, can_msg_prio_t, uint32_t, can_actuator_id_t, uint16_t,
    can_msg_t *
);

// Customizable fake for flight_phase_get_act_allowed_ms
static uint32_t test_act_allowed_ms_value = 0;
w_status_t flight_phase_get_act_allowed_custom(uint32_t *out_ms) {
    *out_ms = test_act_allowed_ms_value;
    return flight_phase_get_act_allowed_ms_fake.return_val;
}

static controller_input_t new_input_state = {0};
BaseType_t xQueueReceive_custom(QueueHandle_t arg0, void *arg1, TickType_t arg2) {
    *(controller_input_t *)arg1 = new_input_state;
    return xQueueReceive_fake.return_val;
}

uint16_t rad_to_can_cmd(float rad) {
    // Convert radians to millidegrees and shift to unsigned 16-bit
    return (uint16_t)(rad / M_PI * 180.0 * 1000.0) + 32768;
}

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(flight_phase_get_state);
        RESET_FAKE(get_analog_data);
        RESET_FAKE(proc_handle_fatal_error);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(log_text);
        RESET_FAKE(log_data);
        RESET_FAKE(build_actuator_analog_cmd_msg);
        RESET_FAKE(flight_phase_get_act_allowed_ms);
        RESET_FAKE(xQueueReceive);
        FFF_RESET_HISTORY();
        // default to everything passing
        xQueueReceive_fake.return_val = pdTRUE;
        xQueueReceive_fake.custom_fake = xQueueReceive_custom;
        can_handler_transmit_fake.return_val = W_SUCCESS;
        // Register the custom fake for flight_phase_get_act_allowed_ms
        flight_phase_get_act_allowed_ms_fake.custom_fake = flight_phase_get_act_allowed_custom;
        flight_phase_get_act_allowed_ms_fake.return_val = W_SUCCESS;
        test_act_allowed_ms_value = 0;
    }

    void TearDown() override {}
};

TEST_F(ControllerTest, Init) {
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    w_status_t res = controller_init();
    EXPECT_EQ(res, W_SUCCESS);
}

TEST_F(ControllerTest, RunLoopPadPhase) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_IDLE;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_SUCCESS);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0);
    EXPECT_EQ(log_text_fake.call_count, 0);
}

TEST_F(ControllerTest, RunLoopPadFilterPhase) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_SE_INIT;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_SUCCESS);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0);
    EXPECT_EQ(log_text_fake.call_count, 0);
}

TEST_F(ControllerTest, RunLoopDataMiss) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;
    xQueueReceive_fake.return_val = pdFALSE; // simulate data miss

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_FAILURE);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0);
}

TEST_F(ControllerTest, RunLoopBoostButNotActAllowed) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_BOOST;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_SUCCESS);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0);
    EXPECT_EQ(log_text_fake.call_count, 0);
}

TEST_F(ControllerTest, RunLoopActAllowedButTimeWrong) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;
    test_act_allowed_ms_value = 2000;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_FAILURE);
    // expect no cmd cuz failure
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0);
}

/**
 * for the successful actuation test cases, numbers come from controller_module_test.cpp
 */

TEST_F(ControllerTest, RunLoopActAllowedStep1) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;
    test_act_allowed_ms_value = 7005;
    new_input_state.roll_state.roll_angle = 0.16345;
    new_input_state.roll_state.roll_rate = 0.154534;
    new_input_state.roll_state.canard_angle = 0.000134;
    new_input_state.canard_coeff = 0.55234;
    new_input_state.pressure_dynamic = 12093;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_SUCCESS);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 1);
    EXPECT_EQ(build_actuator_analog_cmd_msg_fake.arg3_val, rad_to_can_cmd(0.02005049));
    EXPECT_EQ(build_actuator_analog_cmd_msg_fake.call_count, 1);
}

TEST_F(ControllerTest, RunLoopActAllowedStep2) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;
    test_act_allowed_ms_value = 12034;
    new_input_state.roll_state.roll_angle = 0.16345;
    new_input_state.roll_state.roll_rate = 0.154534;
    new_input_state.roll_state.canard_angle = 0.000134;
    new_input_state.canard_coeff = 0.55234;
    new_input_state.pressure_dynamic = 12093;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_SUCCESS);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 1);
    EXPECT_EQ(build_actuator_analog_cmd_msg_fake.arg3_val, rad_to_can_cmd(-0.14380301));
    EXPECT_EQ(build_actuator_analog_cmd_msg_fake.call_count, 1);
}

TEST_F(ControllerTest, RunLoopActAllowedOutOfBounds) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;
    test_act_allowed_ms_value = 15034;
    new_input_state.roll_state.roll_angle = 0.16345;
    new_input_state.roll_state.roll_rate = 0.154534;
    new_input_state.roll_state.canard_angle = 0.000134;
    new_input_state.canard_coeff = 0.55234;
    new_input_state.pressure_dynamic = 709097;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_FAILURE);
    // expect no cmd if interp fails
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0);
}

// expect controller to compute and run during recovery phase too
TEST_F(ControllerTest, RunLoopRecovery) {
    // Arrange
    flight_phase_get_state_fake.return_val = STATE_RECOVERY;
    test_act_allowed_ms_value = 12034; // step 2
    new_input_state.roll_state.roll_angle = 0.16345;
    new_input_state.roll_state.roll_rate = 0.154534;
    new_input_state.roll_state.canard_angle = 0.000134;
    new_input_state.canard_coeff = 0.55234;
    new_input_state.pressure_dynamic = 12093;

    // Act
    w_status_t actual_res = controller_run_loop();

    // Assert
    EXPECT_EQ(actual_res, W_SUCCESS);
    EXPECT_EQ(can_handler_transmit_fake.call_count, 1);
    EXPECT_EQ(build_actuator_analog_cmd_msg_fake.arg3_val, rad_to_can_cmd(-0.14380301));
    EXPECT_EQ(build_actuator_analog_cmd_msg_fake.call_count, 1);
}
