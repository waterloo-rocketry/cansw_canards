#ifndef MODEL_ENCODER_H
#define MODEL_ENCODER_H

#include "application/estimator/estimator_types.h"

/**
 * model measurement encoder function
 * @brief Computes measurement prediction using current state and sensor biases
 * @param x_state_t state
 * @return measurement prediction
 */

double model_meas_encoder(const x_state_t *state);

/**
 * model measurement encoder jacobian
 * @brief jacobian of model_meas_encoder
 * @param x_state_t pointer for returned prrediction
 * @return x_state_t all zeros except for delta = 1
 */

x_state_t model_meas_encoder_jacobian(void);

#endif
