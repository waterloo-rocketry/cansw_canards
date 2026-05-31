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

    // check the outputs
    EXPECT_NEAR(test_output.b_r, 0.5, 1e-4);
    EXPECT_NEAR(test_output.u, 0.02768, 1e-4);


}