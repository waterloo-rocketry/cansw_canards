#include "application/controller/controller_module.h"
#include "application/logger/log.h"
#include "common/math/math.h"

// roll program steps. ms from act-allowed timestamp (ie end of boost phase is 0ms)
#define STEP_0_START_MS 0
#define STEP_1_START_MS 7000
#define STEP_2_START_MS 12000
#define STEP_3_START_MS 17000
#define STEP_4_START_MS 24000

// reference signals corresponding to the step start times
static const float STEP_0_ANGLE_DEG = 0.0f;
static const float STEP_1_ANGLE_DEG = 0.5f;
static const float STEP_2_ANGLE_DEG = -0.5f;
static const float STEP_3_ANGLE_DEG = 0.5f;
static const float STEP_4_ANGLE_DEG = 0.0f;

typedef union {
    double gain_arr[GAIN_NUM];

    struct {
        double gain_k[ROLL_STATE_NUM];
        double gain_k_pre;
    };

} controller_gain_t;

static const double max_commanded_angle = 10 * RAD_PER_DEG; // 10 degrees in radians

// gain instances
static arm_bilinear_interp_instance_f32 gain_instance_arr[GAIN_NUM] = {
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[0][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[1][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[2][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[3][0]},
};

// helper function
static w_status_t
interpolate_gain(double p_dyn, double canard_coeff, controller_gain_t *gain_output) {
    // Normalize coordinates
    double p_norm = (p_dyn - PRESSURE_DYNAMIC_OFFSET) / PRESSURE_DYNAMIC_SCALE;
    double c_norm = (canard_coeff - CANARD_COEFF_OFFSET) / CANARD_COEFF_SCALE;

    // check bounds for p and c: for debugging
    if ((MIN_COOR_BOUND > p_norm) || (GAIN_P_SIZE - 1 < p_norm) || (MIN_COOR_BOUND > c_norm) ||
        (GAIN_C_SIZE - 1 < c_norm)) {
        return W_FAILURE;
    }

    // Interpolate
    for (int i = 0; i < GAIN_NUM; i++) {
        gain_output->gain_arr[i] = arm_bilinear_interp_f32(&gain_instance_arr[i], c_norm, p_norm);
    }

    return W_SUCCESS;
}

// helper function
static double
get_commanded_angle(controller_gain_t gain, roll_state_t roll_state, float ref_signal) {
    // compute commanded angle
    double dot_prod = 0.0;
    double output = 0.0;
    for (int i = 0; i < ROLL_STATE_NUM; i++) {
        dot_prod += gain.gain_k[i] * roll_state.roll_state_arr[i];
    }

    output = dot_prod + gain.gain_k_pre * ref_signal;

    output = fmin(fmax(output, -max_commanded_angle), max_commanded_angle);

    return output;
}

w_status_t controller_module(
    controller_input_t input, uint32_t act_allowed_ms, double *output_angle, float *ref_signal
) {
    // ref_signal
    float r = 0.0f;
    controller_gain_t controller_gain = {0};

    // validate inputs
    if ((NULL == output_angle) || (NULL == ref_signal)) {
        log_text(10, "cntlmodule", "nullptrs");
        return W_INVALID_PARAM;
    }

    // %% Reference signal
    // % Generates reference signal for roll program
    // % includes multiple roll angle steps. Reference r [rad].
    if (act_allowed_ms < STEP_1_START_MS) {
        r = STEP_0_ANGLE_DEG;
    } else if (act_allowed_ms >= STEP_1_START_MS && act_allowed_ms < STEP_2_START_MS) {
        r = STEP_1_ANGLE_DEG;
    } else if (act_allowed_ms >= STEP_2_START_MS && act_allowed_ms < STEP_3_START_MS) {
        r = STEP_2_ANGLE_DEG;
    } else if (act_allowed_ms >= STEP_3_START_MS && act_allowed_ms < STEP_4_START_MS) {
        r = STEP_3_ANGLE_DEG;
    } else if (act_allowed_ms >= STEP_4_START_MS) {
        r = STEP_4_ANGLE_DEG;
    } else {
        // this case should never occur unless timer gone wrong somehow. it must be an err
        log_text(10, "cntlmodule", "invalid flight ms %d", act_allowed_ms);
        return W_FAILURE;
    }

    // %% controller algorithm
    // % Computes control output. Uses gain schedule table and simplified roll model
    // % Inputs: roll state input(1:3), flight conditions input(4:5), reference signal r
    // % Outputs: control input u

    // %%% Gain scheduling
    if (interpolate_gain(input.pressure_dynamic, input.canard_coeff, &controller_gain) !=
        W_SUCCESS) {
        log_text(10, "cntlmodule", "interp fail");
        return W_FAILURE;
    }

    // %%% Feedback law
    *output_angle = get_commanded_angle(controller_gain, input.roll_state, r);

    *ref_signal = r;

    return W_SUCCESS;
}
