/**
 * helpful math-related helper functions to use in unit tests
 */
#pragma once

#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {

#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
}

/**
 * @brief Helper function to compare vectors
 * do EXPECT_NEAR on all 3 components of a vector
 * @param expected expected vector
 * @param actual actual vector
 * @param tolerance % of the expected value to use as the tolerance in EXPECT_NEAR
 */
inline void assert_vec_eq(const vector3d_t &expected, const vector3d_t &actual, double tolerance) {
    EXPECT_NEAR(expected.x, actual.x, abs(expected.x * tolerance));
    EXPECT_NEAR(expected.y, actual.y, abs(expected.y * tolerance));
    EXPECT_NEAR(expected.z, actual.z, abs(expected.z * tolerance));
}
