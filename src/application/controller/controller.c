#include "application/controller/controller.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "task.h"
#include "third_party/canlib/message/msg_actuator.h"
#include <math.h>

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define DATA_WAIT_MS 10
#define LOG_WAIT_MS 10
#define RECOVERY_PERIOD_MS 1000
#define IDLE_PERIOD_MS 1

static controller_t controller_state = {0};
static controller_error_data_t controller_error_stats = {0};

// Send `canard_angle`, the desired canard angle (radians) to CAN
static w_status_t controller_send_can(float canard_angle) {
    // convert canard angle from radians to millidegrees
    int16_t canard_cmd_signed = (int16_t)(canard_angle / M_PI * 180.0 * 1000.0);
    uint16_t canard_cmd_shifted = canard_cmd_signed + 32768;

    // get timestamp for can msg
    float time_ms;
    if (W_SUCCESS != timer_get_ms(&time_ms)) {
        time_ms = 0.0f;
        controller_error_stats.timestamp_errors++;
    }
    uint32_t can_timestamp = (uint32_t)time_ms;

    // Build the CAN msg
    can_msg_t msg;
    if (!build_actuator_analog_cmd_msg(
            PRIO_HIGHEST, can_timestamp, ACTUATOR_CANARD_ANGLE, canard_cmd_shifted, &msg
        )) {
        log_text(LOG_WAIT_MS, "controller", "actuator message build failure");
        controller_error_stats.can_send_errors++;
        log_text(LOG_WAIT_MS, "controller", "actuator message build failure");
    }

    // Send this to can handler module's tx
    w_status_t result = can_handler_transmit(&msg);
    if (result != W_SUCCESS) {
        controller_state.can_send_errors++;
    }
    return result;
}

/**
 * helper to handle a new canard cmd:
 * send canard cmd to CAN, update output queue, log cmd+ref_sig
 * ref_signal is only used for logging here
 * Return W_FAILURE if any of the steps fail, but still try to do everything regardless.
 */
static w_status_t send_cmd(double cmd) {
    w_status_t status = W_SUCCESS; // track status but still try to do everything regardless
    float timestamp_ms = 0.0f;
    // object to copy into the outputs queue (queue is pass by copy, not by reference)
    controller_output_t controller_output = {0};

    // get current timestamp
    if (W_SUCCESS != timer_get_ms(&timestamp_ms)) {
        timestamp_ms = 0.0f;
        log_text(LOG_WAIT_MS, "controller", "get_ms fail");
        status |= W_FAILURE;
    }

    // set controller output
    controller_output.commanded_angle = cmd;
    controller_output.timestamp = (uint32_t)timestamp_ms;

    // send command via CAN
    if (controller_send_can(controller_output.commanded_angle) != W_SUCCESS) {
        controller_state.can_send_errors++;
        log_text(LOG_WAIT_MS, "controller", "CAN send failure");
        status |= W_FAILURE;
    }

    // update output queue with latest cmd (for estimator)
    xQueueOverwrite(output_queue, &controller_output);

    return status;
}

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
    // Create internal state queue (length = 1)
    internal_state_queue =
        xQueueCreate(1, sizeof(controller_input_t)); // estimator function to controller task
    output_queue = xQueueCreate(1, sizeof(controller_output_t));

    // check queue creation
    if ((NULL == internal_state_queue) || (NULL == output_queue)) {
        log_text(LOG_WAIT_MS, "controller", "queue creation failed");
        return W_FAILURE;
    }

    // avoid controller/estimator deadlock
    controller_output_t init_output = {0};
    xQueueOverwrite(output_queue, &init_output);

    // Initialize error tracking
    controller_error_stats.is_init = true;
    controller_error_stats.can_send_errors = 0;
    controller_error_stats.data_miss_counter = 0;
    controller_error_stats.timestamp_errors = 0;
    controller_error_stats.gain_interpolation_errors = 0;
    controller_error_stats.angle_calculation_errors = 0;
    controller_error_stats.log_errors = 0;
    // return w_status_t state
    log_text(LOG_WAIT_MS, "controller", "initialization successful");
    return W_SUCCESS;
}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state) {
    xQueueOverwrite(internal_state_queue, new_state); // overwrite internal queue

    return W_SUCCESS;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in controller
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output) {
    if (pdPASS == xQueuePeek(output_queue, output, 0)) {
        return W_SUCCESS;
    }

    return W_FAILURE;
}

// helper to run 1 loop of the controller task, including delaying where needed.
// not declared static to allow unit testing access
w_status_t controller_run_loop() {
    log_data_container_t log_container = {0};
    w_status_t status = W_SUCCESS;
    flight_phase_state_t current_phase = flight_phase_get_state();

    // do the following steps which vary depending on curr flight phase:
    // 1. determine new canard angle cmd
    // 2. process new cmd (send to CAN, update output queue, log)
    // 3. do task delay
    switch (current_phase) {
            // recovery: actively cmd 0 deg at 1hz
            // status |= send_cmd(cmd_angle_zero);

        // // ref_signal ignored here so set to 0
        // log_container.controller.cmd_angle = (float)cmd_angle_zero;
        // log_container.controller.ref_signal = 0.0f;
        // if (W_SUCCESS != log_data(5, LOG_TYPE_CANARD_CMD, &log_container)) {
        //     log_text(LOG_WAIT_MS, "cntl recovery", "log cmd fail");
        //     status |= W_FAILURE;
        // }
        // // delay 1s per iteration. not as precise as taskdelayuntil but doesnt matter here.
        // // AVOID vtaskdelayuntil: it breaks cuz we dont use it in ACT_ALLOWED phase, so
        // // last_wake_time doesnt get updated consistently and causes this to break as it
        // // tries to catch up in time. TDOO: update freertos versions to where delayuntil has
        // // a return value...
        // vTaskDelay(pdMS_TO_TICKS(RECOVERY_PERIOD_MS));
        // // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS));
        // break;

        // actuation allowed: run controller module.
        // actuation continues through recovery phase too until the board shuts off
        case STATE_RECOVERY:
        case STATE_ACT_ALLOWED: {
            controller_input_t new_state_msg = {0};
            float ref_signal = 0.0f; // track latest reference signal for logging only
            uint32_t act_allowed_ms = 0; // elapsed flight time
            double new_cmd = 0.0;

            // wait for estimator data. >5ms timeout to avoid false fail if wait takes just over 5ms
            if (xQueueReceive(internal_state_queue, &new_state_msg, pdMS_TO_TICKS(DATA_WAIT_MS)) !=
                pdPASS) {
                controller_state.data_miss_counter++;
                log_text(LOG_WAIT_MS, "controller", "data miss");
                return W_FAILURE;
            }

            // get elapsed time since actuation-allowed started
            status |= flight_phase_get_act_allowed_ms(&act_allowed_ms);

            // run controller module
            status |= controller_module(new_state_msg, act_allowed_ms, &new_cmd, &ref_signal);

            // send cmd if we can
            if (W_SUCCESS == status) {
                status |= send_cmd(new_cmd);

                log_container.controller.cmd_angle = (float)new_cmd;
                log_container.controller.ref_signal = ref_signal;
                if (W_SUCCESS != log_data(5, LOG_TYPE_CANARD_CMD, &log_container)) {
                    log_text(LOG_WAIT_MS, "cntl act", "log cmd fail");
                    status |= W_FAILURE;
                }
            } else {
                // if anything fails, send no cmd. MCB failsafes to 0 after some ms of silence
                log_text(LOG_WAIT_MS, "cntl act", "fail; send no cmd");
            }

            break;
        }

        default: // do nothing / wait for new state
            vTaskDelay(pdMS_TO_TICKS(IDLE_PERIOD_MS));
            break;
    }

    return status;
}

/**
 * Controller freertos task
 */
void controller_task(void *argument) {
    (void)argument;

    while (true) {
        if (controller_run_loop() != W_SUCCESS) {
            log_text(LOG_WAIT_MS, "controller", "run loop fail");
        }
    }
}

uint32_t controller_get_status(void) {
    uint32_t status_bitfield = 0;

    // Log all error statistics
    log_text(
        0,
        "controller",
        "can_send=%lu, data_misses=%lu, timestamp=%lu, gain_interp=%lu, "
        "angle_calc=%lu, log=%lu",
        controller_error_stats.can_send_errors,
        controller_error_stats.data_miss_counter,
        controller_error_stats.timestamp_errors,
        controller_error_stats.gain_interpolation_errors,
        controller_error_stats.angle_calculation_errors,
        controller_error_stats.log_errors
    );

    // Also log the internal controller state error counters for comparison
    log_text(
        0,
        "controller",
        "%s can_send_errors=%lu, data_miss_counter=%lu",
        controller_error_stats.is_init ? "true" : "false",
        controller_state.can_send_errors,
        controller_state.data_miss_counter
    );

    return status_bitfield;
}

