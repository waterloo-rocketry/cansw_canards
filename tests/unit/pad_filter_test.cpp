/**
 * numbers in these tests generated from matlab cmdline in simulink-canards commit 2c8c534
 */

#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/estimator_types.h"
#include "application/estimator/pad_filter.h"
DEFINE_FFF_GLOBALS; // this must be called within the extern c block
}

// IMPORTANT: when running the matlab pad_filter in cmdline, remember to
// clear the persistent variables `clear pad_filter`

class PadFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

/**
clc;
clear pad_filter;
IMU_select = [1 1 1];
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];

% Call filter
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2, IMU_select);

% C-style output helper
c_array_str = @(v) ['{', sprintf('%.9f, ', v(1:end-1)), sprintf('%.9f', v(end)), '}'];

% Print results in C-style format
fprintf("x_init  = %s\n", c_array_str(x_init));
fprintf("bias_1  = %s\n", c_array_str(bias_1));
fprintf("bias_2  = %s\n", c_array_str(bias_2));


 */
TEST(PadFilterTest, NewContextOneIteration) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {.array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}};

    y_imu_t imu2 = {
        .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
    };

    y_imu_t actual_bias_1 = {0};
    y_imu_t actual_bias_2 = {0};
    x_state_t actual_x_init = {0};

    // Initialize pad filter
    EXPECT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2), W_SUCCESS);

    // Act
    pad_filter(
        &test_ctx, &imu1, &imu2, false, false, &actual_x_init, &actual_bias_1, &actual_bias_2
    );

    // Assert
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

    // tolerance = 0.001% of the actual value
    // accounts for floats being ~6 significant digits of precision
    float tolerance = 0.00001;

    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            actual_x_init.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * tolerance)
        );
    }

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            actual_bias_1.array[i], expect_bias_1.array[i], abs(expect_bias_1.array[i] * tolerance)
        );
        EXPECT_NEAR(
            actual_bias_2.array[i], expect_bias_2.array[i], abs(expect_bias_2.array[i] * tolerance)
        );
    }
}

TEST(PadFilterTest, InitFailsOnSecondAttempt) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {.array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}};
    y_imu_t imu2 = {
        .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
    };

    // Act & Assert
    ASSERT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2), W_SUCCESS);
    ASSERT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2), W_FAILURE);
}

// clang-format off
/**
IMU_select = [1 1 1];
clear pad_filter;
c_array_str = @(v) ['{', sprintf('%.9f, ', v(1:end-1)), sprintf('%.9f', v(end)), '}'];

% --- Iteration 1 ---
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2, IMU_select);

% --- Iteration 2 ---
IMU_1 = [0.010628; 0.021093; -9.808891; 0.000136; -0.001923; -0.000714; 0.298886; -0.000007; 0.401533; 1013.249230];
IMU_2 = [-0.019629; 0.009774; -9.778883; -0.000589; 0.001033; -0.000447; 0.311101; -0.008456; 0.390086; 1013.298508];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2, IMU_select);

% --- Iteration 3 ---
IMU_1 = [0.009258; 0.018938; -9.807650; 0.000384; -0.001252; 0.000308; 0.300889; -0.000765; 0.398598; 1013.248578];
IMU_2 = [-0.019512; 0.009823; -9.780196; 0.001919; 0.001292; -0.000802; 0.311588; -0.010804; 0.390697; 1013.300835];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2, IMU_select);
fprintf("x_init  = %s\n", c_array_str(x_init));
fprintf("bias_1  = %s\n", c_array_str(bias_1));
fprintf("bias_2  = %s\n", c_array_str(bias_2));

 */
// clang-format on
TEST(PadFilterTest, RunsThreeSequentialIterationsCorrectly) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};
    // tolerance is evaled as 0.2% of the given expect value
    float tolerance = 0.002;

    // --- Iteration 1 ---
    {
        y_imu_t imu1 = {
            .array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}
        };
        y_imu_t imu2 = {
            .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
        };

        x_state_t expect_x = {
            .array = {0.70692628, 0, -0.7072864, -0.001083, 0, 0, 0, 0, 0, 0, 420, 3.482826, 0}
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0, 0.0, 0.0, 0.001888, -0.003147, -0.000569, 0.135194, 0.378179, -0.298964, 0.0
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0, 0.0, 0.0, 0.000398, 0.000759, -0.000681, 0.138892, 0.364747, -0.310093, 0.0
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        // Initialize pad filter on first iteration
        ASSERT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2), W_SUCCESS);

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );
        // only compare final result, dont check values between iterations
    }

    // --- Iteration 2 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.010628,
                0.021093,
                -9.808891,
                0.000136,
                -0.001923,
                -0.000714,
                0.298886,
                -0.000007,
                0.401533,
                1013.249230
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.019629,
                0.009774,
                -9.778883,
                -0.000589,
                0.001033,
                -0.000447,
                0.311101,
                -0.008456,
                0.390086,
                1013.298508
            }
        };

        x_state_t expect_x = {
            .array = {
                0.576677,
                -0.409313,
                0.576352,
                0.409544,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                25762.842166,
                3.62759872846844,
                0.0
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0, 0.0, 0.0, 0.001880, -0.003141, -0.000570, 0.135085, 0.378218, -0.298963, 0.0
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0, 0.0, 0.0, 0.000393, 0.000760, -0.000680, 0.138790, 0.364786, -0.310097, 0.0
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );
    }

    // --- Iteration 3 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.009258,
                0.018938,
                -9.807650,
                0.000384,
                -0.001252,
                0.000308,
                0.300889,
                -0.000765,
                0.398598,
                1013.248578
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.019512,
                0.009823,
                -9.780196,
                0.001919,
                0.001292,
                -0.000802,
                0.311588,
                -0.010804,
                0.390697,
                1013.300835
            }
        };

        x_state_t expect_x = {
            .array = {
                0.706926285,
                0.000000000,
                -0.707286402,
                -0.001083131,
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
        y_imu_t expect_bias_1 = {
            .array = {
                0.009999977,
                0.020000006,
                -9.809999308,
                0.000999704,
                -0.001999835,
                0.000499719,
                -0.400152642,
                0.000153297,
                0.299796320,
                -95365.025888641
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                -0.019999828,
                0.009999919,
                -9.779999816,
                0.000500066,
                0.001000065,
                -0.000999850,
                -0.390173204,
                -0.009877014,
                0.309786649,
                -95364.975888334
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );

        for (int i = 0; i < 13; ++i) {
            EXPECT_NEAR(actual_x.array[i], expect_x.array[i], abs(expect_x.array[i] * tolerance));
        }
        for (int i = 0; i < 10; ++i) {
            EXPECT_NEAR(
                actual_bias_1.array[i],
                expect_bias_1.array[i],
                abs(expect_bias_1.array[i] * tolerance)
            );
            EXPECT_NEAR(
                actual_bias_2.array[i],
                expect_bias_2.array[i],
                abs(expect_bias_2.array[i] * tolerance)
            );
        }
    }
}

TEST(PadFilterTest, RunFailsWithoutInit) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {.array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}};
    y_imu_t imu2 = {
        .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
    };

    bool is_dead_1 = false;
    bool is_dead_2 = false;

    x_state_t actual_x_init = {0};
    y_imu_t actual_bias_1 = {0};
    y_imu_t actual_bias_2 = {0};

    // Act & Assert
    ASSERT_EQ(
        pad_filter(
            &test_ctx,
            &imu1,
            &imu2,
            is_dead_1,
            is_dead_2,
            &actual_x_init,
            &actual_bias_1,
            &actual_bias_2
        ),
        W_FAILURE
    );
}
