#include "application/estimator/model/model_encoder.h"

double model_meas_encoder(const x_state_t *state) {
	return state->delta;
}

x_state_t model_meas_encoder_jacobian(void) {
	// fill state with zeros
	x_state_t result = {0};

	// "as D = delta, Jacobian is unity" -- ft. matlab comments
	result.delta = 1;

	// write to pointer
	return result;
}
