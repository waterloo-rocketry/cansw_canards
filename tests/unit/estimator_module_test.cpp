/**
 * test outputs from simulink-canards v2.2.3
 */

#include "fff.h"
#include <gtest/gtest.h>

#include "utils/mock_helpers.hpp"

extern "C" {
#include "application/estimator/ekf.h"
#include "application/estimator/estimator_module.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/quaternion.h"
#include "arm_math.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdlib.h>

DEFINE_FFF_GLOBALS;

FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *);
}

#define TOLERANCE 0.001 // tolerance for float comparisons

class EstimatorModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_init();
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// clang-format off
/**
clear estimator_module;
clear pad_filter;
% pad filter phase

IMU_select = [1 1 1];

% --- Non-zero test inputs (example from your data) ---
timestamp = 0.005;
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
cmd = [0; 0.005; 0; 0];   % Commanded angle and dummy timestamp
encoder = 0.03;             % radians

[xhat, Phat, controller_input, bias_1, bias_2] = estimator_module( ...
    timestamp, IMU_1, IMU_2, cmd, encoder, IMU_select);

% xhat
fprintf('{');
fprintf('%.9f, ', xhat(1:end-1));
fprintf('%.9f}\n', xhat(end));

% bias_1
fprintf('{');
fprintf('%.9f, ', bias_1(1:end-1));
fprintf('%.9f}\n', bias_1(end));

% bias_2
fprintf('{');
fprintf('%.9f, ', bias_2(1:end-1));
fprintf('%.9f}\n', bias_2(end));
 */
// clang-format on
TEST_F(EstimatorModuleTest, BothImusAlivePadFilterPhaseOnce) {
    // Arrange
    estimator_module_input_t input = {0};
    input.timestamp = 0.005;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    // Commanded angle and timestamp for the controller
    controller_output_t cmd = {0, 5}; // commanded_angle in radians, timestamp in ms
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_SE_INIT;

    // Initializing the context
    estimator_module_ctx_t ctx = {0};
    controller_input_t controller_input = {0}; // Initialize controller_input_t to zero

    // Act
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // Expected output values for x_state_t
    x_state_t expect_x_init = {
        .array = {
            0.706926282,
            0.000000000,
            -0.707286405,
            -0.001083134,
            0.000000000,
            0.000000000,
            0.000000000,
            0.000000000,
            0.000000000,
            0.000000000,
            420.000000000,
            3.482826488,
            0.000000000
        }
    };

    // Expected biases for Movella and Pololu IMUs
    y_imu_t expect_bias_1 = {
        .array = {
            0.010000000,
            0.020000000,
            -9.810000000,
            0.001000000,
            -0.002000000,
            0.000500000,
            -0.400152618,
            0.000153451,
            0.299796362,
            -95365.025888203
        }
    };

    y_imu_t expect_bias_2 = {
        .array = {
            -0.020000000,
            0.010000000,
            -9.780000000,
            0.000500000,
            0.001000000,
            -0.001000000,
            -0.390173050,
            -0.009877161,
            0.309786109,
            -95364.975888203
        }
    };

    // ignore controller output for pad filter phase

    // Assert: Verify x_state_t (for state estimation module)
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            ctx.x.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * TOLERANCE)
        );
    }

    // Assert: Verify bias_1 and bias_2
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1.array[i],
            std::abs(expect_bias_1.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2.array[i],
            std::abs(expect_bias_2.array[i] * TOLERANCE)
        );
    }

    // Optionally, check Phat shouldnt change from 0
    double expected_Phat[169] = {0};
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}

// clang-format off
/**
clear all;
clear pad_filter; clear estimator_module;
format long g

IMU_select = [1 1 1];

% --- Non-zero test inputs (example from your data) ---
timestamp = 0.005;
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
cmd = [0; 0.005; 0; 0];   % Commanded angle and dummy timestamp
encoder = 0.03;             % radians

% --- Call estimator_module ---
estimator_module( ...
    timestamp, IMU_1, IMU_2, cmd, encoder, IMU_select);

% call again
timestamp = 0.01
cmd = [0; 0.1; 0; 0];   % Commanded angle and dummy timestamp
IMU_1 = [0.06; 0.1; -9.99; 0.001; -0.002; 0.0005; 0.5; 0.4; 0.4; 1010.25];
IMU_2 = [-0.03; 0.08; -9.1; 0.005; 0.01; -0.001; 0.31; -0.01; 0.99; 1014.30];
[xhat, Phat, controller_input, bias_1, bias_2] = estimator_module( ...
    timestamp, IMU_1, IMU_2, cmd, encoder, IMU_select);

% xhat
fprintf('{');
fprintf('%.9f, ', xhat(1:end-1));
fprintf('%.9f}\n', xhat(end));

% bias_1
fprintf('{');
fprintf('%.9f, ', bias_1(1:end-1));
fprintf('%.9f}\n', bias_1(end));

% bias_2
fprintf('{');
fprintf('%.9f, ', bias_2(1:end-1));
fprintf('%.9f}\n', bias_2(end));
 */
// clang-format on
TEST_F(EstimatorModuleTest, BothImusAlivePadFilterPhaseTwice) {
    // Arrange - First iteration
    estimator_module_input_t input = {0};
    input.timestamp = 0.005;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    controller_output_t cmd = {0.1, 5}; // Commanded angle in radians, timestamp in ms
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_SE_INIT;

    estimator_module_ctx_t ctx = {0};
    controller_input_t controller_input = {0};

    // Act - First call
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // --- Expected outputs from first iteration ---
    x_state_t expect_x_init = {
        .array = {
            0.571031,
            -0.417133,
            0.570745,
            0.417342,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            250.000000,
            4.481753,
            0.000000
        }
    };

    y_imu_t expect_bias_1 = {
        .array = {
            0.000000,
            0.000000,
            0.000000,
            0.001000,
            -0.002000,
            0.000500,
            0.119601,
            0.381864,
            -0.300800,
            -97343.236211
        }
    };

    y_imu_t expect_bias_2 = {
        .array = {
            0.000000,
            0.000000,
            0.000000,
            0.000522,
            0.001045,
            -0.001000,
            0.128910,
            0.371555,
            -0.309803,
            -97343.166211
        }
    };

    // Assert - First iteration
    // for (int i = 0; i < 13; ++i) {
    //     EXPECT_NEAR(
    //         ctx.x.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * TOLERANCE)
    //     );
    // }

    // for (int i = 0; i < 10; ++i) {
    //     EXPECT_NEAR(
    //         ctx.bias_movella.array[i],
    //         expect_bias_1.array[i],
    //         abs(expect_bias_1.array[i] * TOLERANCE)
    //     );
    //     EXPECT_NEAR(
    //         ctx.bias_pololu.array[i],
    //         expect_bias_2.array[i],
    //         abs(expect_bias_2.array[i] * TOLERANCE)
    //     );
    // }

    // --- Second iteration setup ---
    input.timestamp = 0.01;
    input.movella = {0.06, 0.1, -9.99, 0.001, -0.002, 0.0005, 0.5, 0.4, 0.4, 1010.25};
    input.pololu = {-0.03, 0.08, -9.1, 0.005, 0.01, -0.001, 0.31, -0.01, 0.99, 1014.30};
    input.cmd = {0.0, 10}; // Dummy cmd.angle = 0, timestamp = 100ms

    // Act - Second call
    status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // --- Expected outputs from second iteration ---
    x_state_t expect_x_2 = {
        .array = {
            0.706926425,
            0.000000000,
            -0.707286260,
            -0.001084222,
            0.000000000,
            0.000000000,
            0.000000000,
            0.000000000,
            0.000000000,
            0.000000000,
            420.000000000,
            3.482826488,
            0.000000000
        }
    };

    y_imu_t expect_bias_1_2 = {
        .array = {
            0.010010000,
            0.020016000,
            -9.810036000,
            0.001000000,
            -0.002000000,
            0.000500000,
            -0.400152393,
            0.000233544,
            0.299836648,
            -95365.026488203
        }
    };

    y_imu_t expect_bias_2_2 = {
        .array = {
            -0.020002000,
            0.010014000,
            -9.779864000,
            0.000500900,
            0.001001800,
            -0.001000000,
            -0.390292938,
            -0.009876854,
            0.309786192,
            -95364.975688203
        }
    };

    double expected_Phat[169] = {0}; // All zeros from MATLAB output

    // Assert - Second iteration
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(ctx.x.array[i], expect_x_2.array[i], abs(expect_x_2.array[i] * TOLERANCE));
    }

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1_2.array[i],
            abs(expect_bias_1_2.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2_2.array[i],
            abs(expect_bias_2_2.array[i] * TOLERANCE)
        );
    }

    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}

/** NOTE: to run this script, edit matlab estimator_module to replace the `persistent` variables
 * with `global` variables. Ie, delete 'persistent' and write 'global' in its place. ALSO, delete
the flight_phase calc in matlab and force it to be in flight
 // clang-format off
 clear all;
clear pad_filter; clear estimator_module;
format long g
global x
    global P
    global t
    global b; % remembers g_x, g_P, g_t from last iteration
    global flight_phase

P = zeros(13);
b.bias_1 = zeros(10, 1);
b.bias_2 = zeros(10, 1);
t = 0.005;

IMU_select = [1 1 1];

% --- Non-zero test inputs (example from your data) ---
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
cmd = 0.1;   % Commanded angle and dummy timestamp
encoder = 0.03;             % radians
flight_phase = 0;

timestamp = 0.1;
cmd = 0.1;
x = [...
    0.573781;
         -0.413347;
         0.573488;
         0.413558;
         0.000000;
         0.000000;
         0.000000;
         6.000000;
         0.000000;
         0.000000;
         25762.842421;
         3.627599;
         0.000000]

[xhat, Phat, controller_input, bias_1, bias_2] = estimator_module( ...
   timestamp, IMU_1, IMU_2, cmd, encoder, IMU_select);

% xhat
fprintf('{');
fprintf('%.9f, ', xhat(1:end-1));
fprintf('%.9f}\n', xhat(end));

% controller_input
fprintf('{');
fprintf('%.9f, ', controller_input(1:end-1));
fprintf('%.9f}\n', controller_input(end));

% bias_1
fprintf('{');
fprintf('%.9f, ', bias_1(1:end-1));
fprintf('%.9f}\n', bias_1(end));

% bias_2
fprintf('{');
fprintf('%.9f, ', bias_2(1:end-1));
fprintf('%.9f}\n', bias_2(end));

P_c = reshape(Phat.', 1, []);  % Transpose for row-major C-style output
fprintf('{ ');
fprintf('%.9f, ', P_c(1:end-1));
fprintf('%.9f };\n', P_c(end));
 */
// clang-format on
TEST_F(EstimatorModuleTest, BothImusAliveActAllowedPhaseOnce) {
    // Arrange
    estimator_module_input_t input = {0};
    input.timestamp = 0.1;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    controller_output_t cmd = {0.1, 5}; // commanded_angle, timestamp (ms)
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_ACT_ALLOWED;

    estimator_module_ctx_t ctx = {
        .x =
            {.array =
                 {0.573781,
                  -0.413347,
                  0.573488,
                  0.413558,
                  0.000000,
                  0.000000,
                  0.000000,
                  6.000000,
                  0.000000,
                  0.000000,
                  25762.842421,
                  3.627599,
                  0.000000}},
        .t = 0.005
    };
    controller_input_t controller_input = {0};

    // Act
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // Expected x_state

    x_state_t expect_x_init = {
        .array = {
            0.573781263,
            -0.413347189,
            0.573488263,
            0.413558189,
            0.000749625,
            -0.000499975,
            -0.000249988,
            5.999374329,
            0.885550498,
            -1.225233134,
            25763.647233782,
            3.627323932,
            0.032838284
        }
    };

    y_imu_t expect_bias_1 = {
        {0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000}
    };

    y_imu_t expect_bias_2 = {
        {0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000}
    };

    double expected_Phat[169] = {
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000500, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000500, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000500, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000001000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000001000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000001000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000999769, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.300000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000,
        0.000990099
    };

    // Assert: xhat
    for (int i = 0; i < 13; ++i) {
        if (expect_x_init.array[i] == 0) {
            EXPECT_NEAR(ctx.x.array[i], expect_x_init.array[i], TOLERANCE);
        } else {
            EXPECT_NEAR(
                ctx.x.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * TOLERANCE)
            );
        }
    }

    // Assert: bias_1 and bias_2
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1.array[i],
            abs(expect_bias_1.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2.array[i],
            abs(expect_bias_2.array[i] * TOLERANCE)
        );
    }

    // Expected controller input
    // TODO: use array. this is just conveniently copied as an array from matlab output vector
    double expected_controller_input[5] = {
        -0.000410096, 0.000749625, 0.032838284, 0.658087240, 3.627323932
    };

    // Assert: controller_input values
    EXPECT_NEAR(
        controller_input.roll_state.roll_angle,
        expected_controller_input[0],
        abs(expected_controller_input[0] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.roll_state.roll_rate,
        expected_controller_input[1],
        abs(expected_controller_input[1] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.roll_state.canard_angle,
        expected_controller_input[2],
        abs(expected_controller_input[2] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.pressure_dynamic,
        expected_controller_input[3],
        abs(expected_controller_input[3] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.canard_coeff,
        expected_controller_input[4],
        abs(expected_controller_input[4] * TOLERANCE)
    );

    // Assert: Phat matrix
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-6);
    }
}
