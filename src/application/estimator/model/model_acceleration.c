#include <stdbool.h>
#include <stdint.h>

#include "application/estimator/model/model_acceleration.h"
#include "common/math/math-algebra3d.h"

// from params.m commit 6eb11c1 - for testflight
// positions of IMU centers relative to body frame
static const vector3d_t d1 = {{1.2, 0.074, -0.027}};
static const vector3d_t d2 = {{1.2, 0.065, 0.047}};

vector3d_t model_acceleration(
    const x_state_t *state, const y_imu_t *IMU_1, const bool is_dead_1, const y_imu_t *IMU_2,
    const bool is_dead_2
) {
    vector3d_t a = {{0.0, 0.0, 0.0}};
    const vector3d_t w = state->rates;

    if (!is_dead_1 && !is_dead_2) {
        // average if both IMUs are alive
        vector3d_t w_cross_d1 = math_vector3d_cross(&w, &d1);
        vector3d_t correction_1 = math_vector3d_cross(&w, &w_cross_d1);
        vector3d_t corrected_imu1 = math_vector3d_subt(&IMU_1->accelerometer, &correction_1);

        vector3d_t w_cross_d2 = math_vector3d_cross(&w, &d2);
        vector3d_t correction_2 = math_vector3d_cross(&w, &w_cross_d2);
        vector3d_t corrected_imu2 = math_vector3d_subt(&IMU_2->accelerometer, &correction_2);

        vector3d_t sum = math_vector3d_add(&corrected_imu1, &corrected_imu2);
        a = math_vector3d_scale(0.5, &sum);
    } else if (!is_dead_1) {
        vector3d_t w_cross_d1 = math_vector3d_cross(&w, &d1);
        vector3d_t correction_1 = math_vector3d_cross(&w, &w_cross_d1);
        a = math_vector3d_subt(&IMU_1->accelerometer, &correction_1);
    } else if (!is_dead_2) {
        vector3d_t w_cross_d2 = math_vector3d_cross(&w, &d2);
        vector3d_t correction_2 = math_vector3d_cross(&w, &w_cross_d2);
        a = math_vector3d_subt(&IMU_2->accelerometer, &correction_2);
    }

    return a;
}
