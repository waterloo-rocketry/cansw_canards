#ifndef AK45_CALIBRATION_H
#define AK45_CALIBRATION_H

#include "drivers/ak45_driver/ak45_driver.h"
#include "third_party/rocketlib/include/common.h"

typedef struct {
	float32_t seek_target_deg;
	float32_t backoff_deg;
	uint32_t backoff_settle_ms;
	float32_t stall_speed_erpm_max;
	float32_t stall_current_a_min;
	uint32_t stall_hold_ms;
	uint32_t stall_sample_count;
	float32_t max_tap_delta_deg;
	uint32_t seek_timeout_ms;
	uint32_t settle_timeout_ms;
	float32_t position_tolerance_deg;
	float32_t min_span_deg;
	float32_t max_span_deg;
} ak45_calibration_config_t;

extern const ak45_calibration_config_t ak45_calibration_config_default;

/**
 * @brief Drive to both hard stops (double-tap each), move to midpoint, and re-zero encoder.
 *
 * @param[in] config Calibration parameters; must not be NULL.
 * @return W_SUCCESS on success, W_FAILURE on error.
 */
w_status_t ak45_hard_stop_calibrate(const ak45_calibration_config_t *config);

#endif // AK45_CALIBRATION_H
