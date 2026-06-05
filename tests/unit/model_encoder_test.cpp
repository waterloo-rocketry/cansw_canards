// #include "fff.h"
// #include <gtest/gtest.h>

// extern "C" {
// #include "application/estimator/estimator_types.h"
// #include "application/estimator/model/model_encoder.h"
// DEFINE_FFF_GLOBALS;
// }

// class ModelEncoderTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//         FFF_RESET_HISTORY();
//     }

//     void TearDown() override {}
// };

// TEST_F(ModelEncoderTest, ModelEncoderNonJacobianCheck) {
//     // Arrange
//     x_state_t input = {
//         .array = {
//             14.362602531514463,
//             14.473328027989147,
//             2.364196225163224,
//             14.558891726409236,
//             14.357504223644183,
//             7.280634730842618,
//             12.004207033332001,
//             2.128295079408230,
//             6.326419239394125,
//             13.736032877836006,
//             11.883109943393316,
//             14.392386395893546,
//             9.836110487348803
//         }
//     };
//     double expected_res = 9.836110487348803;

//     // Act
//     double actual_res = model_meas_encoder(&input);

//     // Assert
//     EXPECT_NEAR(expected_res, actual_res, 1e-7);
// }
// TEST_F(ModelEncoderTest, ModelEncoderJacobianCheck) {
//     // Arrange
//     x_state_t expected_res = {.array = {0}};
//     expected_res.delta = 1;

//     // Act
//     x_state_t actual_res = model_meas_encoder_jacobian();

//     // Assert
//     for (int i = 0; i < 13; i++) {
//         EXPECT_EQ(expected_res.array[i], actual_res.array[i]);
//     }
// }