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
    navigator_codegen_ctx_t test_ctx = {0};
    navigator_codegen_sensor_input_t sensor_input = {0};
    navigator_codegen_bias_t bias_in = {0};
    navigator_codegen_sensor_filter_t filter_in = {0};
    navigator_codegen_bias_t bias_out = {0};
    navigator_codegen_sensor_filter_t filter_out = {0};

    test_input.dt = 0.01;
    test_input.flight_phase = true; // Powered flight/coast
    test_input.p_sensor_input = &sensor_input;

    test_ctx.bias = &bias_in;
    test_ctx.filter = &filter_in;
    
    // Initial state: [p, v, q, b_a, b_w]
    // Position (m), Velocity (m/s), Quaternion, Accel bias, Gyro bias
    const double initial_x[11] = {0.7071, 0.7071, 0, 0, 1.5, 0.05, 0.02, 350, 20, 10, 4000};
    memcpy(test_ctx.x, initial_x, sizeof(test_ctx.x));
    
    for(int i=0; i<121; i++) test_ctx.P[i] = 0.0;
    for(int i=0; i<11; i++) test_ctx.P[i*12] = 0.01; // diag elements

    // Sensor measurements with some noise/variation
    sensor_input.board_accel.meas[0] = 0.1;
    sensor_input.board_accel.meas[1] = -0.2;
    sensor_input.board_accel.meas[2] = 9.81 + 20.5; // Upward acceleration
    sensor_input.board_accel.status = true;

    sensor_input.mti_gyro.meas[0] = 0.01;
    sensor_input.mti_gyro.meas[1] = 0.05;
    sensor_input.mti_gyro.meas[2] = 0.3;
    sensor_input.mti_gyro.status = true;

    sensor_input.board_baro.meas = 101325.0 - 12000.0; // Higher altitude pressure
    sensor_input.board_baro.status = true;

    sensor_input.mti_mag.meas[0] = 0.22;
    sensor_input.mti_mag.meas[1] = -0.01;
    sensor_input.mti_mag.meas[2] = 0.45;
    sensor_input.mti_mag.status = true;

    // Nominal post-launch navigation state update
    navigation_codegen_entry(
        test_input.dt, test_input.flight_phase, test_ctx.x, test_ctx.P,
        test_ctx.bias, test_ctx.filter, test_input.p_sensor_input,
        test_output.x_ret, test_output.P_ret, &bias_out, &filter_out);

    const double expected_x_ret[11] = {
        -0.7,
        0.7,
        0.1,
        0.0,
        0.0,
        0.0,
        0.3,
        349.8,
        18.3,
        9.7,
        3156.9,
    };

    const double expected_P_ret[121] = {
        0.0088, 0.0012, 0.0001, -0.0000, -0.0000, 0.0000, -0.0000, -0.0014, -0.0000, -0.0000, 0.0376,
        0.0012, 0.0088, -0.0001, 0.0000, 0.0000, -0.0000, 0.0000, -0.0014, 0.0000, 0.0000, -0.0371,
        0.0001, -0.0001, 0.0100, 0.0000, 0.0000, 0.0000, -0.0000, -0.0000, -0.0013, -0.0013, -0.0032,
        -0.0000, 0.0000, 0.0000, 0.0100, -0.0000, 0.0000, 0.0000, 0.0000, 0.0013, -0.0013, 0.0011,
        -0.0000, 0.0000, 0.0000, -0.0000, 0.0000, 0.0000, -0.0000, -0.0000, 0.0000, -0.0000, 0.0000,
        0.0000, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000, -0.0000, 0.0000, -0.0000,
        -0.0000, 0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0000, -0.0000, -0.0000, 0.0000,
        -0.0014, -0.0014, -0.0000, 0.0000, -0.0000, -0.0000, 0.0000, 0.0107, -0.0035, -0.0017, 0.0001,
        -0.0000, 0.0000, -0.0013, 0.0013, 0.0000, -0.0000, -0.0000, -0.0035, 0.0718, -0.0000, 0.0006,
        -0.0000, 0.0000, -0.0013, -0.0013, -0.0000, 0.0000, -0.0000, -0.0017, -0.0000, 0.0722, 0.0003,
        0.0376, -0.0371, -0.0032, 0.0011, 0.0000, -0.0000, 0.0000, 0.0001, 0.0006, 0.0003, 0.3796,
    };

    for (int i = 0; i < 11; ++i) {
        EXPECT_NEAR(test_output.x_ret[i], expected_x_ret[i], 1e-1);
    }

    for (int i = 0; i < 121; ++i) {
        EXPECT_NEAR(test_output.P_ret[i], expected_P_ret[i], 1e-3);
    }

    EXPECT_NEAR(bias_out.board_gyro[0], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.board_gyro[1], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.board_gyro[2], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_gyro[0], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_gyro[1], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_gyro[2], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.ad_gyro[0], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.ad_gyro[1], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.ad_gyro[2], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.board_mag_earth[0], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.board_mag_earth[1], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.board_mag_earth[2], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_mag_earth[0], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_mag_earth[1], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_mag_earth[2], 0.0, 1e-12);
    EXPECT_NEAR(bias_out.board_baro, 0.0, 1e-12);
    EXPECT_NEAR(bias_out.mti_baro, 0.0, 1e-12);

    EXPECT_NEAR(filter_out.board_accel_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_accel_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_accel_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_gyro_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_gyro_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_gyro_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_accel_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_accel_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_accel_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_gyro_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_gyro_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_gyro_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.ad_accel_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.ad_accel_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.ad_accel_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.ad_gyro_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.ad_gyro_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.ad_gyro_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_baro_f, 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_mag_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_mag_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.board_mag_f[2], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_baro_f, 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_mag_f[0], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_mag_f[1], 0.0, 1e-12);
    EXPECT_NEAR(filter_out.mti_mag_f[2], 0.0, 1e-12);
}