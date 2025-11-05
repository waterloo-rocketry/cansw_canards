#ifndef MODEL_AERODYNAMICS_H
#define MODEL_AERODYNAMICS_H
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "third_party/rocketlib/include/common.h"

static const double canard_sweep_angle = (61 * RAD_PER_DEG);

/**
 * @brief Computes the aerodynamic forces and torque
 * @param x_state estimator state
 * @param estimator_airdata_t airdata results from model_airdata
 * @param pointer to resulting torque vector
 */
void aerodynamics(const x_state_t *state, const estimator_airdata_t *airdata, vector3d_t *torque);

/**
 * @brief returns CL to expected value slowly, to force convergence in EKF
 * @param double mach_num, pre-computed in model_dynamics
 * @return double CL
 */
double airfoil(double mach_num);

/**
 * @brief jacobian for the aerodynamic function
 * @param x_state_t *state to compute the jacobian
 * @param airdata bcz density is needed
 * @param matrix3d_t *torque_v to write to
 * @param vector3d_t *torque_cl to write to
 * @param vector3d_t *torque_delta to write to
 */
void aerodynamics_jacobian(
    const x_state_t *state, const estimator_airdata_t *airdata, matrix3d_t *torque_v,
    vector3d_t *torque_cl, vector3d_t *torque_delta
);

#endif // MODEL_AERODYNAMICS_H