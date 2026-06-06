// #include <gtest/gtest.h>

// extern "C" {
// #include "application/navigator/navigator.h"
// #include "arm_math.h"
// }

// arm_matrix_instance_f32 testing_thing;

// // a math function for example
// uint32_t example_addition(uint32_t a, uint32_t b) {
//     // Handle overflow by returning UINT_MAX
//     if (a > UINT32_MAX - b) {
//         return UINT32_MAX;
//     }
//     return a + b;
// }

// // Test fixture for estimator lib test
// class EstimatorLibTest : public ::testing::Test {
// protected:
//     void SetUp() override {}

//     void TearDown() override {}
// };

// // example of testing addition
// TEST_F(EstimatorLibTest, NormalAdditionCalculatesCorrectly) {
//     // Arrange: set up the inputs and expected result
//     uint32_t a = 1;
//     uint32_t b = 2;
//     uint32_t expected_result = 3;

//     // Act: Call the addition function
//     uint32_t actual_result = example_addition(a, b);

//     // Assert: Verify that the expected result matches the actual result
//     EXPECT_EQ(actual_result, expected_result);
// }

// // example of testing addition with overflow
// TEST_F(EstimatorLibTest, IntegerOverflowAdditionReturnsMax) {
//     // Arrange: set up the inputs and expected result
//     uint32_t a = 1;
//     uint32_t b = UINT32_MAX;
//     uint32_t expected_result = UINT32_MAX;

//     // Act: Call the addition function with values that would overflow
//     uint32_t actual_result = example_addition(a, b);

//     // Assert: Verify that the function returns UINT_MAX when overflow would occur
//     EXPECT_EQ(actual_result, expected_result);
// }
