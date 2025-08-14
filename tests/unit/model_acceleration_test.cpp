#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_acceleration.h"
}

/**
 * all test expected vals generated from matlab v2.2.3
% Define inputs exactly as in the C test
x = zeros(10, 1);
x(5:7) = [0.1; -0.2; 0.3];  % angular rates

IMU_1 = [1.0; 2.0; 3.0];
IMU_2 = [4.0; 5.0; 6.0];

sensor_select = [1 1 1];

% --- Call the function ---
a = model_acceleration(x, IMU_1, IMU_2, sensor_select);

% --- Print in C-style double format ---
fprintf('double expected_x = %.15f;\n', a(1));
fprintf('double expected_y = %.15f;\n', a(2));
fprintf('double expected_z = %.15f;\n', a(3));

 */

TEST(ModelAccelerationTest, BasicTestTwoAlive) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){0.1, -0.2, 0.3};

    input_imu_1.accelerometer = (vector3d_t){1.0, 2.0, 3.0};
    input_imu_2.accelerometer = (vector3d_t){4.0, 5.0, 6.0};

    bool is_dead_1 = false;
    bool is_dead_2 = false;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-6;

    double expected_x = 2.6570900000;
    double expected_y = 3.5315500000;
    double expected_z = 4.4686699999;

    EXPECT_NEAR(actual_result.x, expected_x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_z, tolerance);
}

/**
 % Clear environment
clear; clc;

% Define input state: only angular rates matter (5:7)
x = zeros(10, 1);
x(5:7) = [0.1; 0.2; -0.1];

% Define IMU inputs
IMU_1 = [-1.0; 0.0; 1.0];       % IMU 1 is alive
IMU_2 = [100.0; 200.0; 300.0];  % IMU 2 is dead (ignored)

% Only IMU 1 alive
sensor_select = [1 0 1];

% Call the model_acceleration function
a = model_acceleration(x, IMU_1, IMU_2, sensor_select);

% Print result in C-style format for unit test
fprintf('double expected_x = %.15f;\n', a(1));
fprintf('double expected_y = %.15f;\n', a(2));
fprintf('double expected_z = %.15f;\n', a(3));

 */
TEST(ModelAccelerationTest, OnlyOneAlive_IMU1Alive) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){0.1, 0.2, -0.1};

    input_imu_1.accelerometer = (vector3d_t){-1.0, 0.0, 1.0};
    input_imu_2.accelerometer = (vector3d_t){100.0, 200.0, 300.0}; // Should be ignored

    bool is_dead_1 = false;
    bool is_dead_2 = true;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-5;

    double expected_x = -0.941750000;
    double expected_y = -0.023060000;
    double expected_z = 1.0121300000;

    EXPECT_NEAR(actual_result.x, expected_x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_z, tolerance);
}

/**
 % Clear environment
clear; clc;

% Define input state (angular rates only)
x = zeros(10, 1);
x(5:7) = [-0.2; 0.1; 0.4];

% IMU data
IMU_1 = [50.0; -50.0; 0.0];     % ignored
IMU_2 = [0.5; -0.5; 1.5];       % used

% Only IMU 2 alive
sensor_select = [0 1 1];

% Call the model_acceleration function (param already loaded internally)
a = model_acceleration(x, IMU_1, IMU_2, sensor_select);

% Print output in C-style format
fprintf('double expected_x = %.15f;\n', a(1));
fprintf('double expected_y = %.15f;\n', a(2));
fprintf('double expected_z = %.15f;\n', a(3));

 */
TEST(ModelAccelerationTest, OnlyOneAlive_IMU2Alive) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){-0.2, 0.1, 0.4};

    input_imu_1.accelerometer = (vector3d_t){50.0, -50.0, 0.0}; // Should be ignored
    input_imu_2.accelerometer = (vector3d_t){0.5, -0.5, 1.5};

    bool is_dead_1 = true;
    bool is_dead_2 = false;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-6;

    double expected_x = 0.7090600000;
    double expected_y = -0.464880000000;
    double expected_z = 1.59575000;

    EXPECT_NEAR(actual_result.x, expected_x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_z, tolerance);
}

TEST(ModelAccelerationTest, AllDead) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){0.0, 0.0, 0.0};

    input_imu_1.accelerometer = (vector3d_t){1.0, 2.0, 3.0};
    input_imu_2.accelerometer = (vector3d_t){4.0, 5.0, 6.0};

    bool is_dead_1 = true;
    bool is_dead_2 = true;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-5;

    EXPECT_NEAR(actual_result.x, 0.0, tolerance);
    EXPECT_NEAR(actual_result.y, 0.0, tolerance);
    EXPECT_NEAR(actual_result.z, 0.0, tolerance);
}
