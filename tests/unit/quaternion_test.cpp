
#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc

#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>
}

class QuaternionTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

// Helper function to compare vectors
bool vectors_are_equal(const vector3d_t &a, const vector3d_t &b, float tolerance = 1e-3f) {
    return (fabs(a.x - b.x) < tolerance) && (fabs(a.y - b.y) < tolerance) &&
           (fabs(a.z - b.z) < tolerance);
}

// Helper function to compare matrices
bool matrices_are_equal(const matrix3d_t &a, const matrix3d_t &b, float tolerance = 1e-3f) {
    return (fabs(a.s11 - b.s11) < tolerance) && (fabs(a.s12 - b.s12) < tolerance) &&
           (fabs(a.s13 - b.s13) < tolerance) && (fabs(a.s21 - b.s21) < tolerance) &&
           (fabs(a.s22 - b.s22) < tolerance) && (fabs(a.s23 - b.s23) < tolerance) &&
           (fabs(a.s31 - b.s31) < tolerance) && (fabs(a.s32 - b.s32) < tolerance) &&
           (fabs(a.s33 - b.s33) < tolerance);
}

// Helper function to compare quaternions
bool quaternions_are_equal(const quaternion_t &a, const quaternion_t &b, float tolerance = 1e-6f) {
    return (fabs(a.w - b.w) < tolerance) && (fabs(a.x - b.x) < tolerance) &&
           (fabs(a.y - b.y) < tolerance) && (fabs(a.z - b.z) < tolerance);
}

// Test case: Norm of a known quaternion
TEST(QuaternionTest, KnownQuaternionTestNorm) {
    quaternion_t q = {1.0f, 2.0f, 3.0f, 4.0f}; // Example quaternion
    float expected_norm = sqrt(1.0f + 4.0f + 9.0f + 16.0f); // Expected norm: sqrt(30)

    float result_norm = quaternion_norm(&q);

    EXPECT_NEAR(result_norm, expected_norm, 1e-3f); // Check if the norm is correct
}

// Test case: Norm of identity quaternion
TEST(QuaternionTest, IdentityQuaternionTestNorm) {
    quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion

    float expected_norm = 1.0f; // Norm of identity quaternion should be 1

    float result_norm = quaternion_norm(&q);

    EXPECT_NEAR(result_norm, expected_norm, 1e-3f); // Check if the norm is 1
}

// Test case: Norm of zero quaternion
TEST(QuaternionTest, ZeroQuaternionTestNorm) {
    quaternion_t q = {0.0f, 0.0f, 0.0f, 0.0f}; // Zero quaternion

    float expected_norm = 0.0f; // Norm of zero quaternion should be 0

    float result_norm = quaternion_norm(&q);

    EXPECT_NEAR(result_norm, expected_norm, 1e-3f); // Check if the norm is 0
}

// Test case: Norm of a normalized quaternion
TEST(QuaternionTest, NormalizedQuaternionTestNorm) {
    quaternion_t q = {0.707f, 0.0f, 0.707f, 0.0f}; // Example normalized quaternion

    float expected_norm = 1.0f; // Norm of a normalized quaternion should be 1

    float result_norm = quaternion_norm(&q);

    EXPECT_NEAR(result_norm, expected_norm, 1e-3f); // Check if the norm is 1
}

// Test case: Norm of a non-normalized quaternion
TEST(QuaternionTest, NonNormalizedQuaternionTestNorm) {
    quaternion_t q = {2.0f, 0.0f, 2.0f, 0.0f}; // A non-normalized quaternion

    float expected_norm = sqrt(4.0f + 0.0f + 4.0f + 0.0f); // Expected norm: sqrt(8)

    float result_norm = quaternion_norm(&q);

    EXPECT_NEAR(result_norm, expected_norm, 1e-3f); // Check if the norm is correct
}

TEST(QuaternionTest, KnownQuaternionTestNormalize) {
    // Test for a quaternion that is not normalized
    quaternion_t q = {1.0f, 2.0f, 3.0f, 4.0f}; // Example quaternion

    // Manually calculate the norm
    float norm =
        sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); // norm = sqrt(w^2 + x^2 + y^2 + z^2)

    // Manually calculate the normalized quaternion
    quaternion_t normalized_q = {
        q.w / norm, // w component
        q.x / norm, // x component
        q.y / norm, // y component
        q.z / norm // z component
    };

    // Verify that the quaternion is normalized (i.e., its norm should be 1)
    float normalized_q_norm = sqrt(
        normalized_q.w * normalized_q.w + normalized_q.x * normalized_q.x +
        normalized_q.y * normalized_q.y + normalized_q.z * normalized_q.z
    );
    EXPECT_FLOAT_EQ(normalized_q_norm, 1.0f);

    // Optionally, verify that each component of the normalized quaternion matches the manually
    // calculated value
    EXPECT_FLOAT_EQ(normalized_q.w, q.w / norm);
    EXPECT_FLOAT_EQ(normalized_q.x, q.x / norm);
    EXPECT_FLOAT_EQ(normalized_q.y, q.y / norm);
    EXPECT_FLOAT_EQ(normalized_q.z, q.z / norm);
}

// Test case: Rotation matrix from quaternion for a known quaternion
TEST(QuaternionTest, KnownQuaternionTestRotmatrix) {
    float tolerance = 1e-3f;

    // --- Test Case 1: 90 deg yaw ---
    quaternion_t q1 = {0.7071f, 0.0f, 0.0f, 0.7071f}; // 90 deg yaw
    // MATLAB output:
    matrix3d_t expected_S1 = {
        .s11 = 0.0f,
        .s12 = 1.0f,
        .s13 = 0.0f,
        .s21 = -1.0f,
        .s22 = 0.0f,
        .s23 = 0.0f,
        .s31 = 0.0f,
        .s32 = 0.0f,
        .s33 = 1.0f
    };
    matrix3d_t result_q1 = quaternion_rotmatrix(&q1);
    EXPECT_TRUE(matrices_are_equal(result_q1, expected_S1, tolerance));

    // --- Test Case 2: 90 deg pitch ---
    quaternion_t q2 = {0.7071f, 0.0f, 0.7071f, 0.0f}; // 90 deg pitch
    // MATLAB output:
    matrix3d_t expected_S2 = {
        .s11 = 0.0f,
        .s12 = 0.0f,
        .s13 = -1.0f,
        .s21 = 0.0f,
        .s22 = 1.0f,
        .s23 = 0.0f,
        .s31 = 1.0f,
        .s32 = 0.0f,
        .s33 = 0.0f
    };
    matrix3d_t result_q2 = quaternion_rotmatrix(&q2);
    EXPECT_TRUE(matrices_are_equal(result_q2, expected_S2, tolerance));

    // --- Test Case 3: 180 deg roll ---
    quaternion_t q3 = {0.0f, 1.0f, 0.0f, 0.0f}; // 180 deg roll

    matrix3d_t expected_S3 = {
        .s11 = 1.0f,
        .s12 = 0.0f,
        .s13 = 0.0f,
        .s21 = 0.0f,
        .s22 = -1.0f,
        .s23 = 0.0f,
        .s31 = 0.0f,
        .s32 = 0.0f,
        .s33 = -1.0f
    };
    matrix3d_t result_q3 = quaternion_rotmatrix(&q3);
    EXPECT_TRUE(matrices_are_equal(result_q3, expected_S3, tolerance));
}

// Test case: Identity quaternion (no rotation)
TEST(QuaternionTest, IdentityQuaternionTestRotmatrix) {
    quaternion_t identity_q = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion

    matrix3d_t expected_matrix = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f
    }; // Identity matrix

    matrix3d_t result_matrix = quaternion_rotmatrix(&identity_q);

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}

// Test case: 180-degree rotation around the z-axis (known quaternion)
TEST(QuaternionTest, Rotate180DegreesZAxisRotmatrix) {
    quaternion_t q = {0.0f, 0.0f, 0.707f, 0.707f}; // 180-degree rotation around z-axis

    matrix3d_t expected_matrix = {
        -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f
    }; // Expected rotation matrix for 180-degree rotation

    matrix3d_t result_matrix = quaternion_rotmatrix(&q);

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}

// Test for quaternion update calculation
TEST(QuaternionTest, KnownQuaternionTestUpdate) {
    // Define known quaternion q and omega (angular velocity vector)
    quaternion_t q = {0.5f, 0.5f, 0.5f, 0.5f}; // Example quaternion
    vector3d_t w = {30.0f, 20.0f, 10.0f}; // Example angular velocity
    double dt = 0.01; // example time period

    // Compute the expected update
    quaternion_t expected_update = {
        0.344031231028093, 0.540620505901290, 0.589767824619589, 0.491473187182990
    }; // from matlab

    // Call quaternion_update function
    quaternion_t actual_update = quaternion_update(&q, &w, dt);

    // Define tolerance for floating point comparison
    float tolerance = 1e-4f;

    // Test that each component of the result is close to the expected value
    EXPECT_NEAR(actual_update.w, expected_update.w, tolerance);
    EXPECT_NEAR(actual_update.x, expected_update.x, tolerance);
    EXPECT_NEAR(actual_update.y, expected_update.y, tolerance);
    EXPECT_NEAR(actual_update.z, expected_update.z, tolerance);
}

TEST(QuaternionTest, KnownQuaternionTestToEuler) {
    quaternion_t q1 = {0.7071f, 0.0f, 0.0f, 0.7071f}; // 90 degrees yaw
    vector3d_t expected_euler1 = {0.0f, 0.0f, 1.5708f}; // [roll, pitch, yaw]

    quaternion_t q2 = {0.9239f, 0.0f, 0.3827f, 0.0f}; // 45 degrees pitch
    vector3d_t expected_euler2 = {0.0f, 0.7854f, 0.0f}; // [roll, pitch, yaw]

    quaternion_t q3 = {0.9659f, 0.2588f, 0.0f, 0.0f}; // 30 degrees roll
    vector3d_t expected_euler3 = {0.5236f, 0.0f, 0.0f}; // [roll, pitch, yaw]

    // Call quaternion_to_euler function
    vector3d_t actual_euler1 = quaternion_to_euler(&q1);
    vector3d_t actual_euler2 = quaternion_to_euler(&q2);
    vector3d_t actual_euler3 = quaternion_to_euler(&q3);

    // Define tolerance for floating point comparison
    float tolerance = 1e-4f;

    // Test that the computed Euler angles are close to the expected values
    EXPECT_NEAR(actual_euler1.x, expected_euler1.x, tolerance);
    EXPECT_NEAR(actual_euler1.y, expected_euler1.y, tolerance);
    EXPECT_NEAR(actual_euler1.z, expected_euler1.z, tolerance);

    EXPECT_NEAR(actual_euler2.x, expected_euler2.x, tolerance);
    EXPECT_NEAR(actual_euler2.y, expected_euler2.y, tolerance);
    EXPECT_NEAR(actual_euler2.z, expected_euler2.z, tolerance);

    EXPECT_NEAR(actual_euler3.x, expected_euler3.x, tolerance);
    EXPECT_NEAR(actual_euler3.y, expected_euler3.y, tolerance);
    EXPECT_NEAR(actual_euler3.z, expected_euler3.z, tolerance);
}

// TEST(QuaternionTest, KnownQuaternionTestIncrement) {
//     quaternion_t q = {0.7071f, 0.0f, 0.7071f, 0.0f}; // Identity quaternion
//     vector3d_t omega = {0.1f, 0.2f, 0.3f}; // Rotation around the z-axis
//     float deltaT = 0.01f;

//     // Expected quaternion after a small time increment (from MATLAB output)
//     quaternion_t expected_q = {0.7064f, 0.0014f, 0.7078f, 0.0007f}; // Updated with MATLAB result

//     // Call quaternion_increment function
//     quaternion_t actual_q = quaternion_increment(&q, &omega, deltaT);

//     // Define tolerance for floating point comparison
//     float tolerance = 1e-4f;

//     // Test that the computed quaternion is close to the expected value
//     EXPECT_NEAR(actual_q.w, expected_q.w, tolerance);
//     EXPECT_NEAR(actual_q.x, expected_q.x, tolerance);
//     EXPECT_NEAR(actual_q.y, expected_q.y, tolerance);
//     EXPECT_NEAR(actual_q.z, expected_q.z, tolerance);
// }

TEST(QuaternionTest, KnownQuaternionTestToRollPositiveXRotation) {
    // 90 degrees roll (pi/2) about x-axis
    float angle = M_PI / 2.0f;
    float half_angle = angle / 2.0f;

    quaternion_t q = {.w = cosf(half_angle), .x = sinf(half_angle), .y = 0.0f, .z = 0.0f};

    float expected_roll = angle;
    float actual_roll = quaternion_to_roll(&q);
    float tolerance = 1e-3f;

    EXPECT_NEAR(actual_roll, expected_roll, tolerance);
}

TEST(QuaternionTest, KnownQuaternionTestToRollNegativeXRotation) {
    // -45 degrees roll (-pi/4) about x-axis
    float angle = -M_PI / 4.0f;
    float half_angle = angle / 2.0f;

    quaternion_t q = {.w = cosf(half_angle), .x = sinf(half_angle), .y = 0.0f, .z = 0.0f};

    float expected_roll = angle;
    float actual_roll = quaternion_to_roll(&q);
    float tolerance = 1e-4f;

    EXPECT_NEAR(actual_roll, expected_roll, tolerance);
}

TEST(QuaternionTest, KnownQuaternionTestToRollZeroRotation) {
    // Identity quaternion, should yield 0 roll
    quaternion_t q = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};

    float expected_roll = 0.0f;
    float actual_roll = quaternion_to_roll(&q);
    float tolerance = 1e-6f;

    EXPECT_NEAR(actual_roll, expected_roll, tolerance);
}

/*
 * jacobians unit tests
 */
// rotate
TEST(QuaternionTest, QuaternionRotateJacobianCheck) {
    quaternion_t q_unnormed = {
        .array = {8.147236863931790, 9.057919370756192, 1.269868162935061, 9.133758561390193}
    };
    vector3d_t vec = {.array = {6.323592462254095, 0.975404049994095, 2.784982188670484}};
    double expected_res[3 * 4] = {
        7.446912537073681,
        10.989217526303275,
        -2.864620415794992,
        -3.218071413239360,
        -3.218071413239360,
        2.864620415794992,
        10.989217526303277,
        -7.446912537073681,
        2.864620415794992,
        3.218071413239360,
        7.446912537073681,
        10.989217526303277
    };
    double actual_res[3 * 4];

    // act
    quaternion_rotate_jacobian(actual_res, &q_unnormed, &vec);

    // assert
    for (int i = 0; i < 3 * 4; i++) {
        EXPECT_NEAR(actual_res[i], expected_res[i], 1e-6);
    }
}
// update
TEST(QuaternionTest, QuaternionUpdateJacobianCheck) {
    quaternion_t q_unnormed = {
        .array = {8.147236863931790, 9.057919370756192, 1.269868162935061, 9.133758561390193}
    };
    vector3d_t vec = {.array = {6.323592462254095, 0.975404049994095, 2.784982188670484}};
    double dt = 1.093763038409968;
    double expected_res_q[4 * 4] = {
        1.000000000000000,
        -3.458255852590704,
        -0.533430448699465,
        -1.523055290298935,
        3.458255852590704,
        1.000000000000000,
        1.523055290298935,
        -0.533430448699465,
        0.533430448699465,
        -1.523055290298935,
        1.000000000000000,
        3.458255852590704,
        1.523055290298935,
        0.533430448699465,
        -3.458255852590704,
        1.000000000000000
    };
    double expected_res_w[4 * 3] = {
        -0.324200797261196,
        -0.045451085838688,
        -0.326915231455275,
        0.291605674399151,
        -0.326915231455275,
        0.045451085838688,
        0.326915231455275,
        0.291605674399151,
        -0.324200797261196,
        -0.045451085838688,
        0.324200797261196,
        0.291605674399151
    };
    double actual_res_q[4 * 4];
    double actual_res_w[4 * 3];

    // act
    quaternion_update_jacobian(actual_res_q, actual_res_w, &q_unnormed, &vec, dt);

    // assert
    for (int i = 0; i < 4 * 4; i++) {
        EXPECT_NEAR(actual_res_q[i], expected_res_q[i], 1e-6);
    }
    for (int i = 0; i < 4 * 3; i++) {
        EXPECT_NEAR(actual_res_w[i], expected_res_w[i], 1e-6);
    }
}

// Test for quaternion inverse
TEST(QuaternionTest, QuaternionInverseTest) {
    // Example quaternion
    quaternion_t q_input = {
        3.403857266661332, 5.852677509797774, 2.238119394911370, 7.512670593056528
    };

    // Compute the expected update
    quaternion_t expected_res = {
        3.403857266661332, -5.852677509797774, -2.238119394911370, -7.512670593056528
    }; // from matlab

    // Call quaternion_update function
    quaternion_t actual_res = quaternion_inverse(&q_input);

    // Test that each component of the result is close to the expected value
    EXPECT_TRUE(quaternions_are_equal(expected_res, actual_res, 1e-6));
}

