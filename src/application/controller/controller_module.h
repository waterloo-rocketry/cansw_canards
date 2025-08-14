#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H

#include "application/controller/controller.h"
#include "third_party/rocketlib/include/common.h"

/**
 * Top-level controller module. Calls controller algorithm. Sets reference signal.
 * @param input inputs to controller
 * @param act_allowed_ms elapsed time since actuation-allowed started
 * @param output_angle pointer to store the output angle cmd (rad)
 * @param ref_signal pointer to store output reference signal (rad)
 * @return W_SUCCESS if successful
 */
w_status_t controller_module(
    controller_input_t input, uint32_t act_allowed_ms, double *output_angle, float *ref_signal
);

#endif // CONTROLLER_MODULE_H
