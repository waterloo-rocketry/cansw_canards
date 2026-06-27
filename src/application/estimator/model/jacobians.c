#include "application/estimator/model/jacobians.h"
#include "application/estimator/estimator_types.h"

void write_pData(double *jacobian_flat, int start_coor_x, int start_coor_y, int num_row,
				 int num_col, const double *flat_data) {
	for (int i = 0; i < num_row; i++) {
		for (int j = 0; j < num_col; j++) {
			jacobian_flat[(start_coor_x + i) * SIZE_STATE + (start_coor_y + j)] =
				flat_data[i * num_col + j];
		}
	}
}
