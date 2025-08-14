#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/jacobians.h"
#include "application/estimator/model/model_aerodynamics.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>

/*
 * Parameters
 * go here as static const, or as define
 */

// mass and inertia
static const matrix3d_t J = {
    .array = {{0.46, 0, 0}, {0, 49.5, 0}, {0, 0, 49.5}}
}; // inertia matrix of the rocket
static const matrix3d_t J_inv = {
    .array = {{1 / 0.46, 0, 0}, {0, 1 / 49.5, 0}, {0, 0, 1 / 49.5}}
}; // inverse of inertia matrix of the rocket (J^-1 in matlab)
static vector3d_t g = {
    .array = {-9.81, 0, 0}
}; // gravitational acceleration in the geographic inertial frame

// AIRFOIL
// time constant to converge Cl back to theoretical value in filter
static const double tau_cl_alpha = 50;
// time constant of first order actuator dynamics
static const double tau_est = 0.03;

static matrix3d_t tilde(const vector3d_t *vector) {
    // tilde operator for vector
    return (matrix3d_t){
        .array = {
            {0, -vector->z, vector->y}, {vector->z, 0, -vector->x}, {-vector->y, vector->x, 0}
        }
    };
}

/*
 * Dynamics update
 * returns the new integrated state
 */
x_state_t model_dynamics_update(const x_state_t *state, const u_dynamics_t *input, double dt) {
    x_state_t state_new = {0};

    // Compute rotation matrix from attitude quaternion

    const matrix3d_t S = quaternion_rotmatrix(&(state->attitude));

    const matrix3d_t ST = math_matrix3d_transp(&S); // S', used for altitude update

    /*
     * Aerodynamics
     */
    // get current air information (density is needed for aerodynamics)
    const estimator_airdata_t airdata = model_airdata(state->altitude);

    // forces and torque func -- see model_aerodynamics
    static vector3d_t torque = {0};
    aerodynamics(state, &airdata, &torque);

    // update attitude quaternion
    state_new.attitude = quaternion_update(&state->attitude, &state->rates, dt);

    // rate update
    const vector3d_t J_times_omega = math_vector3d_rotate(&J, &state->rates); // param.J*w
    const vector3d_t gyro_moment =
        math_vector3d_cross(&state->rates, &J_times_omega); // cross(w, param.J*w)
    const vector3d_t moment =
        math_vector3d_subt(&torque, &gyro_moment); // torque - cross(w, param.J*w)
    const vector3d_t omega_dot =
        math_vector3d_rotate(&J_inv, &moment); // inv(param.J) * (torque - cross(w, param.J*w))
    const vector3d_t domega =
        math_vector3d_scale(dt, &omega_dot); // dt * inv(param.J) * (torque - cross(w, param.J*w))

    const vector3d_t omega_new = math_vector3d_add(&state->rates, &domega);
    state_new.rates = omega_new;

    // velocity update
    const vector3d_t acceleration_transport =
        math_vector3d_cross(&state->rates, &state->velocity); // cross(w,v)
    const vector3d_t acceleration_body =
        math_vector3d_subt(&(input->acceleration), &acceleration_transport); // a - cross(w,v)
    const vector3d_t acceleration_gravity = math_vector3d_rotate(&S, &g); // S*param.g
    const vector3d_t acceleration_true =
        math_vector3d_add(&acceleration_body, &acceleration_gravity); // a - cross(w,v) + S*param.g
    const vector3d_t dvelocity =
        math_vector3d_scale(dt, &acceleration_true); // dt * (a - cross(w,v) + S*param.g)
    const vector3d_t v_new =
        math_vector3d_add(&(state->velocity), &dvelocity); // v + dt * (a - cross(w,v) + S*param.g)
    state_new.velocity = v_new;

    // altitude update
    const vector3d_t v_earth = math_vector3d_rotate(&ST, &(state->velocity)); // (S')*v
    state_new.altitude = state->altitude + dt * v_earth.x;

    // canard coeff derivative
    // returns Cl to expected value slowly, to force convergence in EKF
    if (float_equal(airdata.mach_local, 0.0)) {
        return state_new;
    }
    const double mach_num =
        math_vector3d_norm(&state->velocity) / airdata.mach_local; // norm(v) / mach_local
    const double sign_v_x = (state->velocity.x >= 0) ? 1.0 : -1.0; // sign of v.x

    // %%% * sign(v(1)) for easy flying backwards under chute
    const double Cl_theory = airfoil(mach_num) * sign_v_x;
    state_new.CL = state->CL + dt * (1 / tau_cl_alpha * (Cl_theory - state->CL));

    // actuator dynamics
    // linear 1st order
    const double delta_new = state->delta + dt * (1 / tau_est * (input->cmd - state->delta));
    state_new.delta = delta_new;

    return state_new;
}

void model_dynamics_jacobian(
    double pData_dynamic_jacobian[SIZE_STATE * SIZE_STATE], const x_state_t *state,
    const u_dynamics_t *input, double dt
) {
    /**
     * airdata calc
     */
    estimator_airdata_t airdata = model_airdata(state->altitude);

    /**
     * quaternion attitude rows (q, 1:4)
     */
    quaternion_update_matrix_q_t q_q = {0}; // 4x4
    quaternion_update_matrix_w_t q_w = {0}; // 4x3
    quaternion_update_jacobian(&q_q.flat[0], &q_w.flat[0], &(state->attitude), &(state->rates), dt);
    // write to pData: a matrix 4x4 and a matrix 4x3
    write_pData(
        pData_dynamic_jacobian, 0, 0, SIZE_QUAT, SIZE_QUAT, &q_q.flat[0]
    ); // J_x(1:4,1:4) = q_q; % column q (attitude)
    write_pData(
        pData_dynamic_jacobian, 0, 4, SIZE_QUAT, SIZE_VECTOR_3D, &q_w.flat[0]
    ); // J_x(1:4, 5:7) = q_w; % column w (rates)

    /**
     * angular rate rows (w, 5:7)
     */
    // aerodynamics jacobian
    matrix3d_t torque_v = {0}; // 3x3
    vector3d_t torque_cl = {0}; // 3x1
    vector3d_t torque_delta = {0}; // 3x1
    aerodynamics_jacobian(state, &airdata, &torque_v, &torque_cl, &torque_delta);

    const vector3d_t J_w = math_vector3d_rotate(&J, &state->rates); // param.J*w
    const matrix3d_t J_w_tilde = tilde(&J_w); // tilde(param.J*w)
    const matrix3d_t J_inv_scaled = {
        .array = {{1 / 0.46 * (dt), 0, 0}, {0, 1 / 49.5 * (dt), 0}, {0, 0, 1 / 49.5 * (dt)}}
    }; // dt * param.Jinv

    const matrix3d_t J_inv_torque =
        math_matrix3d_mult(&J_inv_scaled, &J_w_tilde); // dt * param.Jinv * tilde(param.J*w)
    const matrix3d_t idn = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; // identity matrix
    const matrix3d_t w_w =
        math_matrix3d_add(&idn, &J_inv_torque); // eye(3) + dt * param.Jinv * tilde(param.J*w)

    const matrix3d_t w_v =
        math_matrix3d_mult(&J_inv_scaled, &torque_v); // dt * param.Jinv * torque_v

    const vector3d_t w_cl =
        math_vector3d_rotate(&J_inv_scaled, &torque_cl); // dt * param.Jinv * torque_cl

    const vector3d_t w_delta =
        math_vector3d_rotate(&J_inv_scaled, &torque_delta); // dt * param.Jinv * torque_delta

    // write to pData: 2 matrices 2 vectors
    write_pData(
        pData_dynamic_jacobian, 4, 4, SIDE_MATRIX_3D, SIDE_MATRIX_3D, &w_w.flat[0]
    ); // J_x(5:7,5:7) = w_w; % column w
    write_pData(
        pData_dynamic_jacobian, 4, 7, SIDE_MATRIX_3D, SIDE_MATRIX_3D, &w_v.flat[0]
    ); // J_x(5:7,8:10) = w_v; % column v
    write_pData(
        pData_dynamic_jacobian, 4, 11, SIZE_VECTOR_3D, SIZE_1D, &w_cl.array[0]
    ); // J_x(5:7,12) = w_cl; % column Cl
    write_pData(
        pData_dynamic_jacobian, 4, 12, SIZE_VECTOR_3D, SIZE_1D, &w_delta.array[0]
    ); // J_x(5:7,13) = w_delta; % column delta

    /**
     * velocity rows (v, 8:10)
     */
    rotation_jacobian_t v_q = {0}; // 3x4
    quaternion_rotate_jacobian(&v_q.flat[0], &(state->attitude), &g);
    for (int i = 0; i < (SIZE_VECTOR_3D * SIZE_QUAT); i++) {
        v_q.flat[i] *= dt;
    } // dt * quaternion_rotate_jacobian(q, param.g)

    const vector3d_t v_scaled = math_vector3d_scale(dt, &state->velocity);

    const matrix3d_t v_w = tilde(&v_scaled); // dt * tilde(v)

    const vector3d_t w_scaled = math_vector3d_scale(-dt, &state->rates);
    const matrix3d_t w_scaled_tilde = tilde(&w_scaled); // - dt * tilde(w)

    const matrix3d_t v_v = math_matrix3d_add(&idn, &w_scaled_tilde); // eye(3) - dt * tilde(v)
    // write to pData: a 3x4 matrix, 2 regular matrix3d_t
    write_pData(
        pData_dynamic_jacobian, 7, 0, SIZE_VECTOR_3D, SIZE_QUAT, &v_q.flat[0]
    ); // J_x(8:10,1:4) = v_q; % column q
    write_pData(
        pData_dynamic_jacobian, 7, 4, SIDE_MATRIX_3D, SIDE_MATRIX_3D, &v_w.flat[0]
    ); // J_x(8:10,5:7) = v_w; % column w
    write_pData(
        pData_dynamic_jacobian, 7, 7, SIDE_MATRIX_3D, SIDE_MATRIX_3D, &v_v.flat[0]
    ); //  J_x(8:10,8:10) = v_v; % column v

    /**
     * altitude rows (alt, 11)
     */
    const quaternion_t q_inv = quaternion_inverse(&(state->attitude));
    rotation_jacobian_t r_q = {0}; // 3x4
    quaternion_rotate_jacobian(
        &r_q.flat[0], &q_inv, &(state->velocity)
    ); // quaternion_rotate_jacobian(quaternion_inv(q), v)
    for (int i = 0; i < (SIZE_VECTOR_3D * SIZE_QUAT); i++) {
        r_q.flat[i] *= dt;
    } // dt * quaternion_rotate_jacobian(q_inv, v)

    const double alt_q[SIZE_QUAT] = {r_q.s11, r_q.s12, r_q.s13, r_q.s14}; // alt_q = r_q(1,:)

    matrix3d_t r_v = quaternion_rotmatrix(&q_inv);
    for (int i = 0; i < (SIDE_MATRIX_3D * SIDE_MATRIX_3D);
         i++) { // we dont have matrix scale anymore :(
        r_v.flat[i] *= dt;
    } // dt * quaternion_rotmatrix(q_inv)

    const vector3d_t alt_v = {.array = {r_v.s11, r_v.s12, r_v.s13}}; // alt_v = r_v(1,:)

    const double alt_alt = 1;

    // write to pData: a 1x4 vector, a vector, a scalar
    write_pData(
        pData_dynamic_jacobian, 10, 0, SIZE_1D, SIZE_QUAT, &alt_q[0]
    ); // J_x(11,1:4) = alt_q; % column q
    write_pData(
        pData_dynamic_jacobian, 10, 7, SIZE_1D, SIZE_VECTOR_3D, &alt_v.array[0]
    ); // J_x(11,8:10) = alt_v; % column v
    write_pData(
        pData_dynamic_jacobian, 10, 10, SIZE_1D, SIZE_1D, &alt_alt
    ); // J_x(11, 11) = alt_alt; % column alt

    /**
     * coefficient row (Cl, 12)
     */
    const double Cl_Cl = 1 - dt * 1 / tau_cl_alpha;
    // write to pData: a scalar
    write_pData(
        pData_dynamic_jacobian, 11, 11, SIZE_1D, SIZE_1D, &Cl_Cl
    ); // J_x(12,12) = Cl_cl; % column Cl

    /**
     * canard angle row (delta, 13)
     */
    const double delta_delta = 1 - dt * 1 / tau_est; // delta_delta = 1 - dt * 1/param.tau_est
    // write to pData: a scalar
    write_pData(
        pData_dynamic_jacobian, 12, 12, SIZE_1D, SIZE_1D, &delta_delta
    ); // J_x(13,13) = delta_delta; % column delta
}