#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "application/controller/controller.h"
#include "application/controller/controller_module.h"
#include "application/controller/gain_table.h"
#include "application/flight_phase/flight_phase.h"
#include "math.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "third_party/canlib/message/msg_actuator.h"

FAKE_VALUE_FUNC(w_status_t, log_text, const char *, const char *);
}

#define TOL 1e-4 // tolerance for float comparisons

DEFINE_FFF_GLOBALS;

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        RESET_FAKE(log_text);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// NOTE: to get the test outputs, edit matlab controller_module.m:
// time_start = 0; % pad delay time
// (Set it to 0 to make it avoid simulating pad filter time)

// Test step 0 (initial reference signal)
/**
clear all; clc;

roll_angle = 0.16345;
roll_rate = 0.154534;
canard_angle = 0.000134;
pressure_dynamic = 12093;
canard_coeff = 0.55234;

input = [roll_angle; roll_rate; canard_angle; pressure_dynamic; canard_coeff];

timestamp = 10+12.5; % sec

[u, r] = controller_module(timestamp, input);

fprintf('Control Output (u): %.8f rad (%.4f deg)\n', u, rad2deg(u));
fprintf('Reference Signal (r): %.8f rad (%.4f deg)\n', r, rad2deg(r));
 */

TEST_F(ControllerTest, Step1) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 0.154534;
    input.roll_state.canard_angle = 0.000134;
    input.canard_coeff = 0.55234;
    input.pressure_dynamic = 12093;

    uint32_t act_allowed_ms = 7005; // step 1
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, act_allowed_ms, &output_angle, &ref_signal);

    // Assert
    double expect_output_angle = 0.02005049;
    float expect_ref_signal = 0.5f;

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_NEAR(ref_signal, expect_ref_signal, fabs(expect_ref_signal * TOL));
    EXPECT_NEAR(output_angle, expect_output_angle, fabs(expect_output_angle * TOL));
}

TEST_F(ControllerTest, Step2) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 0.154534;
    input.roll_state.canard_angle = 0.000134;
    input.canard_coeff = 0.55234;
    input.pressure_dynamic = 12093;

    uint32_t act_allowed_ms = 12034; // step 2
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, act_allowed_ms, &output_angle, &ref_signal);

    // Assert
    double expect_output_angle = -0.14380301;
    float expect_ref_signal = -0.5f;

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_NEAR(ref_signal, expect_ref_signal, fabs(expect_ref_signal * TOL));
    EXPECT_NEAR(output_angle, expect_output_angle, fabs(expect_output_angle * TOL));
}

TEST_F(ControllerTest, Step3) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 0.154534;
    input.roll_state.canard_angle = 0.000134;
    input.canard_coeff = 0.55234;
    input.pressure_dynamic = 12093;

    uint32_t act_allowed_ms = 17098; // step 3
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, act_allowed_ms, &output_angle, &ref_signal);

    // Assert
    double expect_output_angle = 0.02005049;
    float expect_ref_signal = 0.5f;

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_NEAR(ref_signal, expect_ref_signal, fabs(expect_ref_signal * TOL));
    EXPECT_NEAR(output_angle, expect_output_angle, fabs(expect_output_angle * TOL));
}

TEST_F(ControllerTest, Step4) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 0.154534;
    input.roll_state.canard_angle = 0.000134;
    input.canard_coeff = 0.55234;
    input.pressure_dynamic = 12093;

    uint32_t act_allowed_ms = 24098; // step 4
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, act_allowed_ms, &output_angle, &ref_signal);

    // Assert
    double expect_output_angle = -0.06187626;
    float expect_ref_signal = 0.0f;

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_NEAR(ref_signal, expect_ref_signal, fabs(expect_ref_signal * TOL));
    EXPECT_NEAR(output_angle, expect_output_angle, fabs(expect_output_angle * TOL));
}

// test various flight conditions
/**
clear all; clc;

roll_angle = 0.16345;
roll_rate = 2.54534;
canard_angle = -1.00134;
pressure_dynamic = 82093;
canard_coeff = 0.85234;

input = [roll_angle; roll_rate; canard_angle; pressure_dynamic; canard_coeff];

timestamp = 10+15.034; % sec

[u, r] = controller_module(timestamp, input);

fprintf('Control Output (u): %.8f rad (%.4f deg)\n', u, rad2deg(u));
fprintf('Reference Signal (r): %.8f rad (%.4f deg)\n', r, rad2deg(r));
 */
TEST_F(ControllerTest, Step2MoreConds) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 2.54534;
    input.roll_state.canard_angle = -1.00134;
    input.pressure_dynamic = 82093;
    input.canard_coeff = 0.85234;

    uint32_t act_allowed_ms = 15034; // step 2
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, act_allowed_ms, &output_angle, &ref_signal);

    // Assert
    double expect_output_angle = -0.08977158;
    float expect_ref_signal = -0.5f;

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_NEAR(ref_signal, expect_ref_signal, fabs(expect_ref_signal * TOL));
    // TOL 0.001 is ok. interpolation is slightly inaccurate to an acceptable degree (src: finn)
    EXPECT_NEAR(output_angle, expect_output_angle, fabs(expect_output_angle * 0.001));
}

TEST_F(ControllerTest, CondsOutOfBounds) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 2.54534;
    input.roll_state.canard_angle = -1.00134;
    input.pressure_dynamic = 820093;
    input.canard_coeff = 0.85234;

    uint32_t act_allowed_ms = 15034; // step 2
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, act_allowed_ms, &output_angle, &ref_signal);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}
