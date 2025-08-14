#include "application/estimator/estimator_module.h"
#include "application/estimator/ekf.h"
#include "application/estimator/projector.h"
#include "application/logger/log.h"

w_status_t estimator_module(
    const estimator_module_input_t *input, flight_phase_state_t flight_phase,
    estimator_module_ctx_t *ctx, controller_input_t *output_to_controller
) {
    w_status_t status = W_SUCCESS;

    // calculate dt regardless of flight phase
    double dt = input->timestamp - ctx->t;
    // record current time
    ctx->t = input->timestamp;

    // store persistent alive values in case an imu happens to die the moment pad_filter_init
    // runs, it can still use the older value. better than init to 0.
    static y_imu_t last_movella = {0};
    static y_imu_t last_pololu = {0};

    if (!input->pololu_is_dead) {
        last_pololu = input->pololu;
    }
    if (!input->movella_is_dead) {
        last_movella = input->movella;
    }

    switch (flight_phase) {
        // ------- if idle state: do nothing -------
        case STATE_IDLE:
            // do nothing.
            break;

        // ------- after SE-init CAN msg received: run pad filter -------
        case STATE_SE_INIT:
            // initialize the pad filter if it hasn't been done yet
            if (false == ctx->pad_filter_ctx.is_initialized) {
                status |= pad_filter_init(&ctx->pad_filter_ctx, &last_movella, &last_pololu);

                if (W_SUCCESS == status) {
                    log_text(5, "Estimator", "Pad filter init!");
                } else {
                    log_text(10, "Estimator", "Pad filter init fail");
                }
            }

            // run pad filter
            status |= pad_filter(
                &ctx->pad_filter_ctx,
                &input->movella,
                &input->pololu,
                input->movella_is_dead,
                input->pololu_is_dead,
                &ctx->x,
                &ctx->bias_movella,
                &ctx->bias_pololu
            );

            if (status != W_SUCCESS) {
                log_text(10, "Estimator", "ERROR: Pad filter run failed.");
            }
            break;

        // ------- flight!! run ekf -------
        case STATE_BOOST:
        case STATE_ACT_ALLOWED:
        case STATE_RECOVERY:
            ekf_algorithm(
                &ctx->x,
                ctx->P_flat,
                &input->movella,
                &ctx->bias_movella,
                &input->pololu,
                &ctx->bias_pololu,
                input->cmd.commanded_angle,
                input->encoder,
                dt,
                input->movella_is_dead,
                input->pololu_is_dead,
                input->encoder_is_dead
            );

            // controller post-processing
            *output_to_controller = estimator_controller_projector(&ctx->x);
            break;

        default:
            log_text(10, "Estimator", "invalid flight phase: %d", flight_phase);
            return W_FAILURE;
            break;
    }

    return status;
}
