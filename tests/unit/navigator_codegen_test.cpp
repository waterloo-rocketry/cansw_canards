#include "fff.h"
#include <gtest/gtest.h>
#include <iostream>
using namespace std;

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "navigation_codegen_entry.h"
#include "GNC_codegen_data.h"
#include "GNC_codegen_initialize.h"
#include "GNC_codegen_types.h"
#include "common/gnc/gnc_types.h"
#include "airdata_atmos.h"
#include "atan2.h"
#include "ekf_correct.h"
#include "eye.h"
#include "inv.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>
}



class NavigationCodegenTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        GNC_codegen_initialize();
    }

    void TearDown() override {}
};

TEST_F(NavigationCodegenTest, Test1) {
    navigator_codegen_input_t test_input = {0};
    navigator_codegen_output_t test_output = {0};

    test_input.dt = 0.01;
    test_input.flight_phase = true; // Powered flight/coast
    
    // Initial state: [p, v, q, b_a, b_w]
    // Position (m), Velocity (m/s), Quaternion, Accel bias, Gyro bias
    const double initial_x[11] = {0.7071, 0.7071, 0, 0, 1.5, 0.05, 0.02, 350, 20, 10, 4000};
    memcpy(test_input.x, initial_x, sizeof(test_input.x));
    
    for(int i=0; i<121; i++) test_input.P[i] = 0.0;
    for(int i=0; i<11; i++) test_input.P[i*12] = 0.01; // diag elements

    // Sensor measurements with some noise/variation
    test_input.board_accel.meas[0] = 0.1;
    test_input.board_accel.meas[1] = -0.2;
    test_input.board_accel.meas[2] = 9.81 + 20.5; // Upward acceleration
    test_input.board_accel.status = true;

    test_input.mti_gyro.meas[0] = 0.01;
    test_input.mti_gyro.meas[1] = 0.05;
    test_input.mti_gyro.meas[2] = 0.3;
    test_input.mti_gyro.status = true;

    test_input.board_baro.meas = 101325.0 - 12000.0; // Higher altitude pressure
    test_input.board_baro.status = true;

    test_input.mti_mag.meas[0] = 0.22;
    test_input.mti_mag.meas[1] = -0.01;
    test_input.mti_mag.meas[2] = 0.45;
    test_input.mti_mag.status = true;

    // Nominal post-launch navigation state update
    navigation_codegen_entry(
        test_input.dt, test_input.flight_phase, test_input.x, test_input.P,
        &test_input.b, &test_input.sf, &test_input.board_accel,
        &test_input.board_gyro, &test_input.mti_accel,
        &test_input.mti_gyro, &test_input.ad_accel,
        &test_input.ad_gyro, &test_input.board_baro,
        &test_input.board_mag, &test_input.mti_baro,
        &test_input.mti_mag, test_output.x_ret, test_output.P_ret,
        &test_output.b_ret, &test_output.sf_ret);

    const double expected_x_ret[11] = {
        0.7071,
        0.7071,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        98100.0,
        0.0,
        0.0,
        999.0,
    };

    const double expected_P_ret[121] = {
        0.0100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.3335, 0.0, -1.4360, 0.0,
        0.0, 0.0100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0089, -1.4360, 0.0, 0.0,
        0.0, 0.0, 0.0100, 0.0, 0.0, 0.0, 0.0, 1.4360, 0.0089, -1.3335, 0.0,
        0.0, 0.0, 0.0, 0.0100, 0.0, 0.0, 0.0, 0.0, -1.3335, -0.0089, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -1.3335, 0.0089, 1.4360, 0.0, 0.0, 0.0, 0.0, 384.9545, 0.0, -0.0007, 0.0,
        0.0, -1.4360, 0.0089, -1.3335, 0.0, 0.0, 0.0, 0.0, 384.9545, 0.0001, 0.0,
        -1.4360, 0.0, -1.3335, -0.0089, 0.0, 0.0, 0.0, -0.0007, 0.0001, 384.9446, -0.0001,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0001, 0.6925,
    };

    for (int i = 0; i < 11; ++i) {
        cout << test_output.x_ret[i] << endl;
        // EXPECT_NEAR(test_output.x_ret[i], expected_x_ret[i], 1);
    }

    for (int i = 0; i < 121; ++i) {
        // EXPECT_NEAR(test_output.P_ret[i], expe
        cout << test_output.P_ret[i] << endl;
    }

    EXPECT_NEAR(test_output.b_ret.board_gyro[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.board_gyro[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.board_gyro[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_gyro[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_gyro[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_gyro[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.ad_gyro[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.ad_gyro[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.ad_gyro[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.board_mag_earth[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.board_mag_earth[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.board_mag_earth[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_mag_earth[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_mag_earth[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_mag_earth[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.board_baro, 0.0, 1e-12);
    EXPECT_NEAR(test_output.b_ret.mti_baro, 0.0, 1e-12);

    EXPECT_NEAR(test_output.sf_ret.board_accel_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_accel_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_accel_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_gyro_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_gyro_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_gyro_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_accel_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_accel_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_accel_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_gyro_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_gyro_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_gyro_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.ad_accel_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.ad_accel_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.ad_accel_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.ad_gyro_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.ad_gyro_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.ad_gyro_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_baro_f, 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_mag_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_mag_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.board_mag_f[2], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_baro_f, 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_mag_f[0], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_mag_f[1], 0.0, 1e-12);
    EXPECT_NEAR(test_output.sf_ret.mti_mag_f[2], 0.0, 1e-12);
}