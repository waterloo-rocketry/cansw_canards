#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "controller_codegen_entry.h"
#include "GNC_codegen_data.h"
#include "GNC_codegen_initialize.h"
#include "diag.h"
#include "eye.h"
#include "common/gnc/gnc_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>
}


class ControllerCodegenTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        GNC_codegen_initialize();
    }

    void TearDown() override {}
};

TEST_F(ControllerCodegenTest, Test1) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {0.16345, 0.154534};
    const double coeffs[2] = {0.0, 0.0};
    const double P_minus[4] = {1.0, 0.0, 0.0, 1.0};

    test_input.b_time = 22.5;
    test_input.dt_ctrl = 0.01;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 12093.0;
    test_input.delta = 0.000134;
    test_input.w_old = 0.154534;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = 0.0;
    test_input.w_dot_old = 0.0;

    // Nominal post-launch controller state based on closedrocket's MATLAB model.

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, 0.5000, 1e-4);
    EXPECT_NEAR(test_output.u, 0.02770, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], 0.0000, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], 0.0000, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, 0.1545, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 1.000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], 0.0000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], 0.0000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.01780, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, 0.0000, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, 0.0000, 1e-4);
}



TEST_F(ControllerCodegenTest, Test2) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {0.16345, 2.54534};
    const double coeffs[2] = {0.0, 0.0};
    const double P_minus[4] = {1.0, 0.0, 0.0, 1.0};

    test_input.b_time = 29.0;
    test_input.dt_ctrl = 0.01;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 82093.0;
    test_input.delta = -1.00134;
    test_input.w_old = 2.54534;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = 0.0;
    test_input.w_dot_old = 0.0;

    // Nominal post-launch controller state based on closedrocket's MATLAB model.

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, -0.5000, 1e-4);
    EXPECT_NEAR(test_output.u, -0.3491, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], 0.0000, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], 0.0000, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, 2.5453, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 0.9846, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], 0.1232, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], 0.1232, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.0158, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, -0.1252, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, 0.0000, 1e-4);
}

TEST_F(ControllerCodegenTest, Test3) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {0.25, 0.4};
    const double coeffs[2] = {0.12, -0.03};
    const double P_minus[4] = {0.8, 0.02, 0.02, 0.6};

    test_input.b_time = 23.2;
    test_input.dt_ctrl = 0.008;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 450.0;
    test_input.delta = 0.35;
    test_input.w_old = 0.3;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = 0.05;
    test_input.w_dot_old = 0.2;

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, 0.5000, 1e-4);
    EXPECT_NEAR(test_output.u, 0.0, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], 0.1936, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], 0.4909, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, 0.4000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 0.7995, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], 0.0163, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], 0.0163, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.5736, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, 0.0813, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, 3.2750, 1e-4);
}

TEST_F(ControllerCodegenTest, Test4) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {-0.4, -2.8};
    const double coeffs[2] = {-0.08, 0.05};
    const double P_minus[4] = {1.2, -0.05, -0.05, 0.9};

    test_input.b_time = 27.8;
    test_input.dt_ctrl = 0.012;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 55000.0;
    test_input.delta = -0.6;
    test_input.w_old = -2.5;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = -0.2;
    test_input.w_dot_old = -0.6;

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, -0.5000, 1e-4);
    EXPECT_NEAR(test_output.u, 0.3491, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], 0.0067, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], -0.1968, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, -2.8000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 1.0960, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], 0.2463, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], 0.2463, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.0562, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, -0.2250, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, -6.7000, 1e-4);
}

TEST_F(ControllerCodegenTest, Test5) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {0.6, 1.5};
    const double coeffs[2] = {0.05, 0.02};
    const double P_minus[4] = {0.6, 0.1, 0.1, 0.4};

    test_input.b_time = 33.3;
    test_input.dt_ctrl = 0.015;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 25000.0;
    test_input.delta = 0.15;
    test_input.w_old = 1.3;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = 0.1;
    test_input.w_dot_old = 0.3;

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, 0.5000, 1e-4);
    EXPECT_NEAR(test_output.u, -0.3491, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], 0.1256, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], 0.2181, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, 1.5000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 0.5430, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], -0.0494, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], -0.0494, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.0087, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, 0.0938, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, 3.5583, 1e-4);
}

TEST_F(ControllerCodegenTest, Test6) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {-0.7, 3.6};
    const double coeffs[2] = {-0.15, -0.04};
    const double P_minus[4] = {1.5, 0.12, 0.12, 0.3};

    test_input.b_time = 41.2;
    test_input.dt_ctrl = 0.02;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 90000.0;
    test_input.delta = -1.1;
    test_input.w_old = 3.2;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = -0.35;
    test_input.w_dot_old = 0.8;

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, 0.0000, 1e-4);
    EXPECT_NEAR(test_output.u, 0.3491, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], -0.2379, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], 0.0062, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, 3.6000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 0.9815, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], 0.3922, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], 0.3922, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.1571, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, -0.4000, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, 5.6000, 1e-4);
}

TEST_F(ControllerCodegenTest, Test7) {
    controller_codegen_input_t test_input = {0};
    controller_codegen_output_t test_output = {0};
    const double xR[2] = {0.05, -0.9};
    const double coeffs[2] = {0.2, 0.1};
    const double P_minus[4] = {0.4, -0.08, -0.08, 0.2};

    test_input.b_time = 21.2;
    test_input.dt_ctrl = 0.009;
    memcpy(test_input.xR, xR, sizeof(xR));
    test_input.pdyn = 1200.0;
    test_input.delta = 0.45;
    test_input.w_old = -0.7;
    memcpy(test_input.coeffs, coeffs, sizeof(coeffs));
    memcpy(test_input.P_minus, P_minus, sizeof(P_minus));
    test_input.d_old = 0.25;
    test_input.w_dot_old = -0.4;

    controller_codegen_entry(test_input.b_time, test_input.dt_ctrl, test_input.xR,
                              test_input.pdyn, test_input.delta, test_input.w_old,
                              test_input.coeffs, test_input.P_minus,
                              test_input.d_old, test_input.w_dot_old, &test_output.u,
                              &test_output.b_r, test_output.coeffs_ret,
                              &test_output.w_old_ret, test_output.P_minus_ret,
                              &test_output.d_old_ret, &test_output.w_dot_old_ret);

    // check the outputs (MATLAB generated values, 4 significant figures)
    EXPECT_NEAR(test_output.b_r, 0.0000, 1e-4);
    EXPECT_NEAR(test_output.u, 0.3491, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[0], 0.1301, 1e-4);
    EXPECT_NEAR(test_output.coeffs_ret[1], -0.6207, 1e-4);
    EXPECT_NEAR(test_output.w_old_ret, -0.9000, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[0], 0.3999, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[1], -0.0816, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[2], -0.0816, 1e-4);
    EXPECT_NEAR(test_output.P_minus_ret[3], 0.1839, 1e-4);
    EXPECT_NEAR(test_output.d_old_ret, 0.2437, 1e-4);
    EXPECT_NEAR(test_output.w_dot_old_ret, -5.8556, 1e-4);
}