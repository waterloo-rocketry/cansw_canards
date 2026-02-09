#ifndef MODEL_DYNAMICS_H
#define MODEL_DYNAMICS_H

#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "arm_math.h"
#include "third_party/rocketlib/include/common.h"

/**
 * Model dynamics update (non-jacobian)
 * Computes state derivative with predictive model
 * @param x_state_t current state of estimator
 * @param u_dynamics_t input signal
 * @param double THE dt time step in seconds
 * @return x_state_t new state of estimator
 */
x_state_t model_dynamics_update(const x_state_t* state, const u_dynamics_t* input, double dt);

/**
 * Model dynamics jacobian
 * Computes jacobian of state derivative with respect to state and input
 * @param pData_dynamic_jacobian  flat jacobian matrix to write to (13x13)
 * @param state current state of estimator
 * @param input input signal
 * @param dt the time step in seconds
 */
void model_dynamics_jacobian(
    double pData_dynamic_jacobian[SIZE_STATE * SIZE_STATE], const x_state_t* state,
    double dt
);

#endif