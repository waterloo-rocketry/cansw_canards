// #include "fff.h"
// #include <gtest/gtest.h>

// extern "C" {
// #include "application/estimator/estimator_types.h"
// #include "application/estimator/projector.h"
// }

// TEST(ProjectorTest, projector_test) {
//     // ARRANGE:
//     x_state_t input_estimator_state;
//     controller_input_t expectedResult;

//     // Initialize test input data (input_estimator_state)
//     input_estimator_state.attitude = (quaternion_t){1.0, 0.0, 3.0, 8.0};
//     input_estimator_state.rates = (vector3d_t){1.0, 0.0, 3.0};
//     input_estimator_state.velocity = (vector3d_t){1.0, 2.0, 3.0};
//     input_estimator_state.altitude = 1000.0;
//     input_estimator_state.CL = 0.5;
//     // input_estimator_state.delta = 0.1;

//     // Initialize expected result (expectedResult)
//     expectedResult.roll_state = (roll_state_t
//     ){0.708626272127670, 1.000000000000000}; // Expected roll state
//     expectedResult.pressure_dynamic = 7.780938460081654; // Expected dynamic pressure
//     expectedResult.canard_coeff = 0.5; // Expected canard coefficient

//     // ACT:
//     controller_input_t actualResult = estimator_controller_projector(&input_estimator_state);

//     // ASSERT:
//     double tolerance = 1e-5;

//     EXPECT_NEAR(
//         actualResult.roll_state.roll_angle, expectedResult.roll_state.roll_angle, tolerance
//     );
//     EXPECT_NEAR(actualResult.roll_state.roll_rate, expectedResult.roll_state.roll_rate, tolerance);
//     // EXPECT_NEAR(
//     //     actualResult.roll_state.canard_angle, expectedResult.roll_state.canard_angle, tolerance
//     // );
//     EXPECT_NEAR(actualResult.canard_coeff, expectedResult.canard_coeff, tolerance);
//     EXPECT_NEAR(actualResult.pressure_dynamic, expectedResult.pressure_dynamic, tolerance);
// }