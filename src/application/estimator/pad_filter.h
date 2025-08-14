#ifndef PAD_FILTER_H
#define PAD_FILTER_H

#include "application/estimator/estimator.h"
#include "application/estimator/estimator_types.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * this holds persistent data for 1 instance of a pad_filter (ie, its context)
 */
typedef struct {
    y_imu_t filtered_1;
    y_imu_t filtered_2;
    bool is_initialized;
} pad_filter_ctx_t;

/**
 * @brief Initialize the pad filter. must only run once in the whole program
 * @param IMU_1 input to init with
 * @param IMU_2 input to init with
 * @note both imu values must be the last ALIVE values, even if its slightly stale. pad_filter_init
 * cannot handle dead imus, because it needs to init the filtered arrays with actual imu values. if
 * an imu is actually permanently dead, it will be all 0s anyway for the rest of the program
 */
w_status_t pad_filter_init(pad_filter_ctx_t *ctx, const y_imu_t *IMU_1, const y_imu_t *IMU_2);

/**
 * @param ctx input - object containing the persistent data for this instance of pad_filter
 * @param IMU_1 input
 * @param IMU_2 input
 * @param is_dead_1 input - true means dead
 * @param is_dead_2 input - true means dead
 * @param x_init output
 * @param bias output
 */
w_status_t pad_filter(
    pad_filter_ctx_t *ctx, const y_imu_t *IMU_1, const y_imu_t *IMU_2, const bool is_dead_1,
    const bool is_dead_2, x_state_t *x_init, y_imu_t *bias_1, y_imu_t *bias_2
);
#endif