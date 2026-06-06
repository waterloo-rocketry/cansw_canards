// #include "fff.h"
// #include <gtest/gtest.h>

// extern "C" {
// #include "application/estimator/estimator_types.h"
// #include "application/estimator/model/jacobians.h"
// #include "application/estimator/model/model_imu.h"
// }

// class ModelImuTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//         FFF_RESET_HISTORY();
//     }

//     void TearDown() override {}
// };

// // clang-format off
// /**
// clear all;
// clc;

// % === SIZE DEFINITIONS ===
// SIZE_STATE = 13;

// % === STATE VECTOR ===
// x = [
//     1;0;3;8; 1;0;3; 0;0;0; 1000; 0; 0;
// ];

// bias = [1;2;3; 4;5;6; 7;8;9; 101325;];


// [y] = model_meas_imu(0, x, bias);
//  */
// // clang-format on
// TEST_F(ModelImuTest, ModelMeasurementIMUCheck) {
//     // ARRANGE:
//     x_state_t input_estimator_state;
//     y_imu_t input_imu_bias;
//     y_imu_t expectedResult;

//     // Initialize test input data
//     input_estimator_state.attitude = (quaternion_t){1.0, 0.0, 3.0, 8.0};
//     input_estimator_state.rates = (vector3d_t){1.0, 0.0, 3.0};
//     input_estimator_state.altitude = 1000.0;

//     input_imu_bias.accelerometer = (vector3d_t){1.0, 2.0, 3.0};
//     input_imu_bias.gyroscope = (vector3d_t){4.0, 5.0, 6.0};
//     input_imu_bias.magnetometer = (vector3d_t){7.0, 8.0, 9.0};
//     input_imu_bias.barometer = 101325.0;

//     // Initialize expected result
//     expectedResult.gyroscope = (vector3d_t){5.0, 5.0, 9.0}; // Expected accelerometer value
//     expectedResult.accelerometer = {0};
//     expectedResult.magnetometer = (vector3d_t
//     ){-5.81081081081081, -1.51351351351351, 12.5675675675676}; // Expected magnetometer value
//     expectedResult.barometer = 191194.354531258; // Expected barometer value

//     // ACT:
//     y_imu_t actualResult = model_measurement_imu(&input_estimator_state, &input_imu_bias);

//     // ASSERT:
//     double tolerance = 1e-5;

//     for (int i = 0; i < SIZE_IMU_ALL; i++) {
//         EXPECT_NEAR(actualResult.array[i], expectedResult.array[i], tolerance);
//     }
// }

// TEST_F(ModelImuTest, ModelMeasurementJacobianCheck) {
//     // ARRANGE:
//     // Initialize test input data
//     x_state_t input_estimator_state = {
//         .array = {
//             0.814723686393179,
//             0.905791937075619,
//             0.126986816293506,
//             0.913375856139019,
//             0.632359246225410,
//             0.097540404999410,
//             0.278498218867048,
//             0.546881519204984,
//             0.957506835434298,
//             0.964888535199277,
//             0.157613081677548,
//             0.970592781760616,
//             0.957166948242946
//         }
//     };
//     y_imu_t input_estimator_imu_data = {
//         .array = {
//             0.381558457093008,
//             0.765516788149002,
//             0.795199901137063,
//             0.186872604554379,
//             0.489764395788231,
//             0.445586200710899,
//             0.646313010111265,
//             0.709364830858073,
//             0.754686681982361,
//             0.276025076998578
//         }
//     };

//     // Initialize expected result
//     double expectedResultFlat[SIZE_IMU_MEAS * SIZE_STATE] = {
//         0,
//         0,
//         0,
//         0,
//         1,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         1,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         1,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         1.411894044,
//         1.786475312,
//         -0.071203369,
//         0.878564216,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0.878564216,
//         0.071203369,
//         1.786475312,
//         -1.411894044,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0.071203369,
//         -0.878564216,
//         1.411894044,
//         1.786475312,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         0,
//         -12.0168583,
//         0,
//         0
//     };

//     // ACT:
//     double actualResult[SIZE_IMU_MEAS * SIZE_STATE] = {0};
//     model_measurement_imu_jacobian(
//         &actualResult[0], &input_estimator_state, &input_estimator_imu_data
//     );

//     // ASSERT:
//     double tolerance = 1e-6;

//     for (int i = 0; i < SIZE_IMU_MEAS * SIZE_STATE; i++) {
//         EXPECT_NEAR(actualResult[i], expectedResultFlat[i], tolerance);
//     }
// }
