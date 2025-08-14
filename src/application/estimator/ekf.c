#include "application/estimator/ekf.h"
#include "application/estimator/model/jacobians.h"
#include "application/estimator/model/model_acceleration.h"
#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/model_encoder.h"
#include "application/estimator/model/model_imu.h"
#include "application/estimator/model/quaternion.h"
#include "application/logger/log.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>
#include <stdint.h>

static double Q_arr[SIZE_STATE * SIZE_STATE] = {0};
static double R_MTI_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {0};
static double R_ALTIMU_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {0};

static arm_matrix_instance_f64 Q = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = Q_arr};
static arm_matrix_instance_f64 R_MTI = {
    .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = R_MTI_arr
};
static arm_matrix_instance_f64 R_ALTIMU = {
    .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = R_ALTIMU_arr
};

/*
 * Filter settings --------------------------------
 */

/**
 * @param matrix pointer to the matrix aka array
 * @param length number of items in matrix
 * @note assumes all items are doubles
 */
static inline void reset_temp_matrix(double *matrix, uint32_t length) {
    memset(matrix, 0, length * sizeof(double));
}

/**
 * EKF init -------------------------
 *
 * Flattened data of matrices initialized:
 * 1. Q
 * - square 13 matrix, tuning for prediction E(noise)
 * @see ekf_algorithm, prediction step
 *
 * 2. R_MTI
 * - Weighting, measurement model: MTi630
 * @see ekf_algorithm, correction step
 *
 * 3. R_ALTIMU
 * - Weighting, measurement model: Polulu AltIMU v6
 * @see ekf_algorithm, correction step
 */

w_status_t ekf_init(void) {
    // 1. matrix Q
    static const double Q_diag[SIZE_STATE] = {
        1e-10, 1e-10, 1e-10, 1e-10, 0.001, 0.01, 0.01, 1e-6, 1e-6, 1e-6, 0.001, 0.3, 0.1
    };
    // static double Q_arr[SIZE_STATE * SIZE_STATE] = {0};
    // reset_temp_matrix(Q_arr, SIZE_STATE * SIZE_STATE);

    math_init_matrix_diag(&Q, (uint16_t)SIZE_STATE, Q_diag);

    // 2. matrix R for MTI
    // static double R_MTI_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {0};

    static const double R_MTI_diag[SIZE_IMU_MEAS] = {1e-6, 1e-6, 1e-6, 0.01, 0.01, 0.01, 1};

    math_init_matrix_diag(&R_MTI, (uint16_t)SIZE_IMU_MEAS, R_MTI_diag);

    // 3. matrix R for ALTIMU
    // static double R_ALTIMU_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {};

    static const double R_ALTIMU_diag[SIZE_IMU_MEAS] = {1e-6, 1e-6, 1e-6, 0.01, 0.01, 0.01, 1};

    math_init_matrix_diag(&R_ALTIMU, (uint16_t)SIZE_IMU_MEAS, R_ALTIMU_diag);

    return W_SUCCESS;
}

/*
 * Algorithms --------------------------------
 */

void ekf_matrix_predict(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const u_dynamics_t *u_input,
    double Q_arr[SIZE_STATE * SIZE_STATE], double dt
) {
    x_state_t state_new = {0};

    // set up matrix instance for arm operations
    arm_matrix_instance_f64 P = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = P_flat};

    // DISCRETE DYNAMICS UPDATE
    state_new = model_dynamics_update(x_state, u_input, dt);

    // discrete jacobian: F = df/dx
    static double F_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(F_flat, SIZE_STATE * SIZE_STATE);
    model_dynamics_jacobian(F_flat, x_state, u_input, dt);
    arm_matrix_instance_f64 F = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = F_flat};

    // DISCRETE COVARIANCE
    // F'
    static double F_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(F_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 F_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = F_transp_flat
    };
    arm_mat_trans_f64(&F, &F_transp);

    // FP = F*P
    static double FP_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(FP_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 FP = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = FP_flat};
    arm_mat_mult_f64(&F, &P, &FP);

    // F * P * F'
    static double FPF_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(FPF_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 FPF_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = FPF_transp_flat
    };
    arm_mat_mult_f64(&FP, &F_transp, &FPF_transp);

    // P_pred = FPF' + Q
    arm_mat_add_f64(&FPF_transp, &Q, &P);

    *x_state = state_new;
}

void ekf_matrix_correct_imu(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const arm_matrix_instance_f64 *R,
    const y_imu_t *y_meas_full, const y_imu_t *bias
) {
    // size 10 of y_imu_t to size 7
    static double y_meas[SIZE_IMU_MEAS] = {0};
    memcpy(y_meas, &y_meas_full->array[3], SIZE_IMU_MEAS * sizeof(double));

    // set up matrix instance for arm operations
    // old P
    arm_matrix_instance_f64 P = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = P_flat};

    // new P: to update old pointer at the end
    static double P_corr_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(P_corr_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 P_corr = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = P_corr_flat
    };

    // CORRECTION
    // compute expected measurement and difference to measured values

    // y = IMU_1(4:end)

    const y_imu_t y_expected_full = model_measurement_imu(x_state, bias);
    static double y_expected[SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(y_expected, SIZE_IMU_MEAS);
    memcpy(y_expected, &y_expected_full.array[3], SIZE_IMU_MEAS * sizeof(double));

    static double innovation[SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(innovation, SIZE_IMU_MEAS);
    arm_sub_f64(y_meas, y_expected, innovation, SIZE_IMU_MEAS);

    static double H_flat[SIZE_IMU_MEAS * SIZE_STATE] = {0};
    reset_temp_matrix(H_flat, SIZE_IMU_MEAS * SIZE_STATE);
    model_measurement_imu_jacobian(H_flat, x_state, (y_imu_t *)bias);
    const arm_matrix_instance_f64 H = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_STATE, .pData = H_flat
    };

    // compute Kalman gain (and helper matrices)
    // H' = trans(H) // b1
    static double H_transp_flat[SIZE_IMU_MEAS * SIZE_STATE] = {0};
    reset_temp_matrix(H_transp_flat, SIZE_IMU_MEAS * SIZE_STATE);
    arm_matrix_instance_f64 H_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = H_transp_flat
    };
    arm_mat_trans_f64(&H, &H_transp);

    // PH' = P * H' // b2
    static double PH_transp_flat[SIZE_STATE * SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(PH_transp_flat, SIZE_STATE * SIZE_IMU_MEAS);
    arm_matrix_instance_f64 PH_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = PH_transp_flat
    };
    arm_mat_mult_f64(&P, &H_transp, &PH_transp);

    // HPH' = H * PH' // b3
    static double HPH_transp_flat[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(HPH_transp_flat, SIZE_IMU_MEAS * SIZE_IMU_MEAS);
    arm_matrix_instance_f64 HPH_transp = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = HPH_transp_flat
    };
    arm_mat_mult_f64(&H, &PH_transp, &HPH_transp);

    // L = HPH' + R // b1
    static double L_flat[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(L_flat, SIZE_IMU_MEAS * SIZE_IMU_MEAS);
    arm_matrix_instance_f64 L = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = L_flat
    };
    arm_mat_add_f64(&HPH_transp, R, &L);

    // Linv = inv(L) // b3
    static double L_inv_flat[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(L_inv_flat, SIZE_IMU_MEAS * SIZE_IMU_MEAS);
    arm_matrix_instance_f64 L_inv = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = L_inv_flat
    };
    // this line clears L after inversing
    if (arm_mat_inverse_f64(&L, &L_inv) != ARM_MATH_SUCCESS) {
        log_text(5, "ekf", "L inv fail");
        return;
    }

    // Kalman gain
    // K =  PH' * inv(L) // K_data
    static double K_flat[SIZE_STATE * SIZE_IMU_MEAS] = {0};
    reset_temp_matrix(K_flat, SIZE_STATE * SIZE_IMU_MEAS);
    arm_matrix_instance_f64 K = {.numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = K_flat};
    arm_mat_mult_f64(&PH_transp, &L_inv, &K);

    // KH = K*H // b3
    static double KH_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(KH_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 KH = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = KH_flat};
    arm_mat_mult_f64(&K, &H, &KH);

    // // I = eye // I_data
    static double identity_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(identity_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 I = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = identity_flat
    };
    math_init_matrix_identity(&I, SIZE_STATE);

    // E = I - KH // b2
    static double E_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(E_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 E = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = E_flat};
    arm_mat_sub_f64(&I, &KH, &E);

    // correct covariance estimate

    // E' = trans(E) // b1
    static double E_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(E_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 E_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = E_transp_flat
    };
    arm_mat_trans_f64(&E, &E_transp);

    // PE' = P*E' // b3
    static double PE_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(PE_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 PE_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = PE_transp_flat
    };
    arm_mat_mult_f64(&P, &E_transp, &PE_transp);

    // EPE' = E*PE' // b1
    static double EPE_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(EPE_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 EPE_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = EPE_transp_flat
    };
    arm_mat_mult_f64(&E, &PE_transp, &EPE_transp);

    // K_transp = trans(K) // b2
    static double K_transp_flat[SIZE_IMU_MEAS * SIZE_STATE] = {0};
    reset_temp_matrix(K_transp_flat, SIZE_IMU_MEAS * SIZE_STATE);
    arm_matrix_instance_f64 K_transp = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_STATE, .pData = K_transp_flat
    };
    arm_mat_trans_f64(&K, &K_transp);

    // RK' = R*K' // b3
    static double RK_transp_flat[SIZE_IMU_MEAS * SIZE_STATE] = {0};
    reset_temp_matrix(RK_transp_flat, SIZE_IMU_MEAS * SIZE_STATE);
    arm_matrix_instance_f64 RK_transp = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_STATE, .pData = RK_transp_flat
    };
    arm_mat_mult_f64(R, &K_transp, &RK_transp);

    // KRK' = K*RK' // b2
    static double KRK_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(KRK_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 KRK_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = KRK_transp_flat
    };
    arm_mat_mult_f64(&K, &RK_transp, &KRK_transp);

    // P_new = EPE' + KRK' // b3
    arm_mat_add_f64(&EPE_transp, &KRK_transp, &P_corr);

    // update P_flat sheet
    write_pData(P_flat, 0, 0, SIZE_STATE, SIZE_STATE, &P_corr_flat[0]);

    // K * innovation
    x_state_t k_innovation = {0};
    arm_mat_vec_mult_f64(&K, &innovation[0], k_innovation.array);
    // x_corr = x + K * innovation;
    x_state_t x_corr = {0};
    arm_add_f64(k_innovation.array, x_state->array, x_corr.array, SIZE_STATE);

    for (int i = 0; i < SIZE_STATE; i++) {
        x_state->array[i] = x_corr.array[i];
    }
    const quaternion_t state_q = x_state->attitude; // normalize attitude quaternion
    x_state->attitude = quaternion_normalize(&state_q);
}

void ekf_matrix_correct_encoder(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const double R, double encoder
) {
    // set up matrix instance for arm operations
    // old P
    arm_matrix_instance_f64 P = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = P_flat};

    // new P: to update old pointer at the end
    static double P_corr_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(P_corr_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 P_corr = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = P_corr_flat
    };

    // CORRECTION
    // compute expected measurement and difference to measured values

    // y = IMU_1(4:end)

    const double y_expected = model_meas_encoder(x_state);
    const double innovation = encoder - y_expected;

    const x_state_t encoder_jacobian = model_meas_encoder_jacobian();
    static double H_flat[SIZE_STATE] = {0};
    memcpy(H_flat, encoder_jacobian.array, SIZE_STATE * sizeof(double));

    const arm_matrix_instance_f64 H = {
        .numRows = SIZE_1D, .numCols = SIZE_STATE, .pData = H_flat
    }; // H is 1x13 for encoder

    // compute Kalman gain (and helper matrices)
    // H' = trans(H) // b1

    double H_transp_flat[SIZE_STATE] = {0};
    arm_matrix_instance_f64 H_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_1D, .pData = H_transp_flat
    };

    arm_mat_trans_f64(&H, &H_transp);

    // PH' = P * H' // b2
    static double PH_transp_flat[SIZE_STATE] = {0};
    reset_temp_matrix(PH_transp_flat, SIZE_STATE);

    arm_matrix_instance_f64 PH_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_1D, .pData = PH_transp_flat
    };
    arm_mat_mult_f64(&P, &H_transp, &PH_transp);

    // HPH' = H * PH' // b3
    static double HPH_transp_flat = 0;
    arm_matrix_instance_f64 HPH_transp = {
        .numRows = SIZE_1D, .numCols = SIZE_1D, .pData = &HPH_transp_flat
    };
    arm_mat_mult_f64(&H, &PH_transp, &HPH_transp);

    // L = HPH' + R // b1
    static double L = 0;
    L = HPH_transp_flat + R;

    // L inv
    if (L == 0) {
        return;
    }
    double L_inv = 1.0 / L;

    // Kalman gain
    // K =  PH' * inv(L)
    static double K_flat[SIZE_STATE] = {0};
    memcpy(K_flat, PH_transp_flat, SIZE_STATE * sizeof(double));
    for (int i = 0; i < SIZE_STATE; i++) {
        K_flat[i] *= L_inv;
    }
    const arm_matrix_instance_f64 K = {.numRows = SIZE_STATE, .numCols = SIZE_1D, .pData = K_flat};

    // KH = K*H // b3
    static double KH_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(KH_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 KH = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = KH_flat};
    arm_mat_mult_f64(&K, &H, &KH);

    // // I = eye // I_data
    static double identity_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(identity_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 I = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = identity_flat
    };
    math_init_matrix_identity(&I, SIZE_STATE);

    // E = I - KH // b2
    static double E_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(E_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 E = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = E_flat};
    arm_mat_sub_f64(&I, &KH, &E);

    // correct covariance estimate

    // E' = trans(E) // b1
    static double E_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(E_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 E_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = E_transp_flat
    };
    arm_mat_trans_f64(&E, &E_transp); // this line fails to give right answer

    // PE' = P*E' // b3
    static double PE_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(PE_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 PE_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = PE_transp_flat
    };
    arm_mat_mult_f64(&P, &E_transp, &PE_transp);

    // EPE' = E*PE' // b1
    static double EPE_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(EPE_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 EPE_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = EPE_transp_flat
    };
    arm_mat_mult_f64(&E, &PE_transp, &EPE_transp);

    // K_transp = trans(K) // b2
    // do not need a matrix instance for K_transp

    // RK' = R*K' // b3
    static double RK_transp_flat[SIZE_STATE] = {0};
    memcpy(RK_transp_flat, K_flat, SIZE_STATE * sizeof(double));
    for (int i = 0; i < SIZE_STATE; i++) {
        RK_transp_flat[i] *= R;
    }
    const arm_matrix_instance_f64 RK_transp = {
        .numRows = SIZE_1D, .numCols = SIZE_STATE, .pData = RK_transp_flat
    };

    // KRK' = K*RK' // b2
    static double KRK_transp_flat[SIZE_STATE * SIZE_STATE] = {0};
    reset_temp_matrix(KRK_transp_flat, SIZE_STATE * SIZE_STATE);
    arm_matrix_instance_f64 KRK_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = KRK_transp_flat
    };
    arm_mat_mult_f64(&K, &RK_transp, &KRK_transp);

    // P_new = EPE' + KRK' // b3
    arm_mat_add_f64(&EPE_transp, &KRK_transp, &P_corr);

    // update P_flat sheet
    write_pData(P_flat, 0, 0, SIZE_STATE, SIZE_STATE, &P_corr_flat[0]);

    // K * innovation
    double k_innovation[SIZE_STATE] = {0};
    memcpy(k_innovation, K_flat, SIZE_STATE * sizeof(double));
    for (int i = 0; i < SIZE_STATE; i++) {
        k_innovation[i] *= innovation;
    }

    // x_corr = x + K * innovation;
    x_state_t x_corr = {0};
    arm_add_f64(k_innovation, x_state->array, x_corr.array, SIZE_STATE);

    // update x_state
    memcpy(x_state->array, x_corr.array, SIZE_STATE * sizeof(double));

    // normalize attitude quaternion
    const quaternion_t state_q = x_corr.attitude;
    x_state->attitude = quaternion_normalize(&state_q);
}

void ekf_algorithm(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const y_imu_t *imu_mti,
    const y_imu_t *bias_mti, const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, double cmd,
    double encoder, double dt, const bool is_dead_MTI, const bool is_dead_ALTIMU,
    bool is_dead_encoder
) {
    // Prediction step

    u_dynamics_t u_input = {0};
    u_input.acceleration =
        model_acceleration(x_state, imu_mti, is_dead_MTI, imu_altimu, is_dead_ALTIMU);
    u_input.cmd = cmd;

    // Predict
    ekf_matrix_predict(x_state, P_flat, &u_input, Q.pData, dt);

    // %% Correction step(s), sequential for each IMU
    // %%% R is a square matrix (size depending on amount of sensors), tuning for measurement
    // E(noise)

    // only correct with alive sensors
    if (!is_dead_encoder) {
        const double R = 0.001;
        ekf_matrix_correct_encoder(x_state, P_flat, R, encoder); // correct encoder measurement
    }

    if (!is_dead_MTI) {
        ekf_matrix_correct_imu(x_state, P_flat, &R_MTI, imu_mti, bias_mti);
    }

    if (!is_dead_ALTIMU) {
        ekf_matrix_correct_imu(x_state, P_flat, &R_ALTIMU, imu_altimu, bias_altimu);
    }
}

