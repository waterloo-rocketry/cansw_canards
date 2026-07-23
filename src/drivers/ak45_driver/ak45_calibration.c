#include <math.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/logger/log.h"
#include "drivers/ak45_driver/ak45_calibration.h"
#include "drivers/timer/timer.h"

static const uint32_t LOG_WAIT_MS = 1;
static const uint32_t POLL_MS = 20;
// motor failsafes to 0 after a short period of CAN silence, so position commands
// must be resent periodically while waiting on a long-running condition
static const uint32_t CMD_RESEND_MS = 500;

// tracks elapsed time and resend/freshness bookkeeping for one wait loop
typedef struct {
	uint32_t start_ms;
	uint32_t last_cmd_ms;
	uint32_t last_timestamp_ms;
} cal_timer_t;

static bool config_is_valid(const ak45_calibration_config_t *config) {
	if (NULL == config) {
		return false;
	}

	return (config->seek_target_deg > 0.0f) && (config->backoff_deg > 0.0f) &&
		   (config->stall_speed_erpm_max > 0.0f) && (config->stall_current_a_min > 0.0f) &&
		   (config->stall_hold_ms > 0) && (config->stall_sample_count > 0) &&
		   (config->max_tap_delta_deg > 0.0f) && (config->seek_timeout_ms > 0) &&
		   (config->settle_timeout_ms > 0) && (config->position_tolerance_deg > 0.0f) &&
		   (config->min_span_deg > 0.0f) && (config->max_span_deg > config->min_span_deg);
}

static bool feedback_is_stalled(const ak45_feedback_t *fb,
								const ak45_calibration_config_t *config) {
	return (fabsf(fb->speed_erpm) < config->stall_speed_erpm_max) &&
		   (fabsf(fb->current_a) > config->stall_current_a_min);
}

static bool feedback_is_slow(const ak45_feedback_t *fb, const ak45_calibration_config_t *config) {
	return fabsf(fb->speed_erpm) < config->stall_speed_erpm_max;
}

/**
 * @brief Read feedback, rejecting motor faults and frames already seen.
 */
static w_status_t read_fresh_feedback(ak45_feedback_t *fb, uint32_t *last_timestamp_ms) {
	if (ak45_get_latest_feedback(fb) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (fb->fault_code != AK45_FAULT_NONE) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "motor fault %d", fb->fault_code);
		return W_FAILURE;
	}

	if (fb->timestamp_ms == *last_timestamp_ms) {
		return W_FAILURE;
	}

	*last_timestamp_ms = fb->timestamp_ms;
	return W_SUCCESS;
}

static w_status_t cal_timer_start(cal_timer_t *timer) {
	if (timer_get_ms(&timer->start_ms) != W_SUCCESS) {
		return W_FAILURE;
	}

	timer->last_cmd_ms = timer->start_ms - CMD_RESEND_MS; // force an immediate resend
	timer->last_timestamp_ms = 0;
	return W_SUCCESS;
}

/**
 * @brief Block until a fresh feedback frame arrives or the deadline passes.
 *
 * Resends @p target_deg every CMD_RESEND_MS while @p resend is true, so callers
 * can wait on motor state without the MCB failsafing on CAN silence.
 */
static w_status_t poll_feedback(cal_timer_t *timer, float32_t target_deg, bool resend,
								uint32_t timeout_ms, ak45_feedback_t *out_fb,
								uint32_t *out_now_ms) {
	while (true) {
		uint32_t now_ms = 0;
		if (timer_get_ms(&now_ms) != W_SUCCESS) {
			return W_FAILURE;
		}

		if ((now_ms - timer->start_ms) > timeout_ms) {
			return W_FAILURE;
		}

		if (resend && ((now_ms - timer->last_cmd_ms) >= CMD_RESEND_MS)) {
			if (ak45_send_position_cmd(target_deg) != W_SUCCESS) {
				return W_FAILURE;
			}
			timer->last_cmd_ms = now_ms;
		}

		if (read_fresh_feedback(out_fb, &timer->last_timestamp_ms) == W_SUCCESS) {
			*out_now_ms = now_ms;
			return W_SUCCESS;
		}

		vTaskDelay(pdMS_TO_TICKS(POLL_MS));
	}
}

/**
 * @brief Drive toward a hard stop and wait for a stall, sampling position while held.
 *
 * Stalled = low speed + high current, sustained for stall_hold_ms. The reported
 * position is the mean of stall_sample_count feedback frames taken during that hold,
 * filtering out single-frame noise.
 */
static w_status_t wait_for_stall(float32_t target_deg, const ak45_calibration_config_t *config,
								 float32_t *out_position_deg) {
	cal_timer_t timer;
	uint32_t hold_start_ms = 0;
	float32_t sample_sum = 0.0f;
	uint32_t sample_count = 0;

	if (cal_timer_start(&timer) != W_SUCCESS) {
		return W_FAILURE;
	}

	while (true) {
		ak45_feedback_t fb = {0};
		uint32_t now_ms = 0;
		if (poll_feedback(&timer, target_deg, true, config->seek_timeout_ms, &fb, &now_ms) !=
			W_SUCCESS) {
			log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "seek timeout");
			return W_FAILURE;
		}

		if (!feedback_is_stalled(&fb, config)) {
			sample_count = 0;
			continue;
		}

		if (sample_count == 0) {
			hold_start_ms = now_ms;
			sample_sum = 0.0f;
		}
		sample_sum += fb.position_deg;
		sample_count++;

		if (((now_ms - hold_start_ms) >= config->stall_hold_ms) &&
			(sample_count >= config->stall_sample_count)) {
			*out_position_deg = sample_sum / (float32_t)sample_count;
			return W_SUCCESS;
		}
	}
}

/**
 * @brief Drive toward target_deg and wait until speed stays low for hold_ms.
 */
static w_status_t wait_for_low_speed(float32_t target_deg, const ak45_calibration_config_t *config,
									 uint32_t hold_ms, uint32_t timeout_ms) {
	cal_timer_t timer;
	uint32_t hold_start_ms = 0;
	bool holding = false;

	if (cal_timer_start(&timer) != W_SUCCESS) {
		return W_FAILURE;
	}

	while (true) {
		ak45_feedback_t fb = {0};
		uint32_t now_ms = 0;
		if (poll_feedback(&timer, target_deg, true, timeout_ms, &fb, &now_ms) != W_SUCCESS) {
			return W_FAILURE;
		}

		if (!feedback_is_slow(&fb, config)) {
			holding = false;
			continue;
		}

		if (!holding) {
			holding = true;
			hold_start_ms = now_ms;
		}

		if ((now_ms - hold_start_ms) >= hold_ms) {
			return W_SUCCESS;
		}
	}
}

/**
 * @brief Confirm one hard stop with a double-tap: hit, back off, hit again.
 *
 * A single stall can be friction or a snag rather than the real mechanical
 * limit, so the stop is only accepted once two independent approaches agree
 * within max_tap_delta_deg. @p sign is +1 for the positive stop, -1 for the
 * negative stop.
 */
static w_status_t seek_stop_confirmed(float32_t sign, const ak45_calibration_config_t *config,
									  float32_t *out_stop_deg) {
	float32_t seek_target = sign * config->seek_target_deg;
	float32_t tap1 = 0.0f;
	float32_t tap2 = 0.0f;

	if (wait_for_stall(seek_target, config, &tap1) != W_SUCCESS) {
		return W_FAILURE;
	}

	float32_t backoff_target = tap1 - (sign * config->backoff_deg);
	if (wait_for_low_speed(
			backoff_target, config, config->backoff_settle_ms, config->seek_timeout_ms) !=
		W_SUCCESS) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "backoff settle timeout");
		return W_FAILURE;
	}

	if (wait_for_stall(seek_target, config, &tap2) != W_SUCCESS) {
		return W_FAILURE;
	}

	if (fabsf(tap1 - tap2) > config->max_tap_delta_deg) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "tap mismatch %.2f vs %.2f", tap1, tap2);
		return W_FAILURE;
	}

	*out_stop_deg = (tap1 + tap2) / 2.0f;
	log_text(LOG_WAIT_MS,
			 LOG_LVL_INFO,
			 "ak45_cal",
			 "stop %.2f (taps %.2f, %.2f)",
			 *out_stop_deg,
			 tap1,
			 tap2);
	return W_SUCCESS;
}

/**
 * @brief Drive to target_deg and wait until it's reached at low speed.
 */
static w_status_t wait_for_position(float32_t target_deg, const ak45_calibration_config_t *config) {
	cal_timer_t timer;

	if (cal_timer_start(&timer) != W_SUCCESS) {
		return W_FAILURE;
	}

	while (true) {
		ak45_feedback_t fb = {0};
		uint32_t now_ms = 0;
		if (poll_feedback(&timer, target_deg, true, config->settle_timeout_ms, &fb, &now_ms) !=
			W_SUCCESS) {
			log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "midpoint settle timeout");
			return W_FAILURE;
		}

		bool at_target = fabsf(fb.position_deg - target_deg) <= config->position_tolerance_deg;
		if (at_target && feedback_is_slow(&fb, config)) {
			return W_SUCCESS;
		}
	}
}

/**
 * @brief Confirm the new origin reads ~0 deg, averaging a few samples for noise rejection.
 */
static w_status_t verify_origin(const ak45_calibration_config_t *config) {
	cal_timer_t timer;
	float32_t sample_sum = 0.0f;

	if (cal_timer_start(&timer) != W_SUCCESS) {
		return W_FAILURE;
	}

	for (uint32_t i = 0; i < config->stall_sample_count; i++) {
		ak45_feedback_t fb = {0};
		uint32_t now_ms = 0;
		if (poll_feedback(&timer, 0.0f, false, config->settle_timeout_ms, &fb, &now_ms) !=
			W_SUCCESS) {
			return W_FAILURE;
		}
		sample_sum += fb.position_deg;
	}

	float32_t mean_position_deg = sample_sum / (float32_t)config->stall_sample_count;
	if (fabsf(mean_position_deg) > config->position_tolerance_deg) {
		log_text(
			LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "origin verify failed %.2f", mean_position_deg);
		return W_FAILURE;
	}

	return W_SUCCESS;
}

static w_status_t abort_calibration(void) {
	ak45_send_disable_cmd();
	return W_FAILURE;
}

w_status_t ak45_hard_stop_calibrate(const ak45_calibration_config_t *config) {
	if (!config_is_valid(config)) {
		return W_INVALID_PARAM;
	}

	float32_t pos_neg = 0.0f;
	if (seek_stop_confirmed(-1.0f, config, &pos_neg) != W_SUCCESS) {
		return abort_calibration();
	}

	float32_t pos_pos = 0.0f;
	if (seek_stop_confirmed(1.0f, config, &pos_pos) != W_SUCCESS) {
		return abort_calibration();
	}

	float32_t span = pos_pos - pos_neg;
	if ((span < config->min_span_deg) || (span > config->max_span_deg)) {
		log_text(LOG_WAIT_MS, LOG_LVL_WARN, "ak45_cal", "span %.2f out of range", span);
		return abort_calibration();
	}

	float32_t midpoint = (pos_neg + pos_pos) / 2.0f;
	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "ak45_cal", "span %.2f midpoint %.2f", span, midpoint);

	if (wait_for_position(midpoint, config) != W_SUCCESS) {
		return abort_calibration();
	}

	if (ak45_send_set_origin() != W_SUCCESS) {
		return abort_calibration();
	}

	if (verify_origin(config) != W_SUCCESS) {
		return abort_calibration();
	}

	log_text(LOG_WAIT_MS, LOG_LVL_INFO, "ak45_cal", "hard stop cal success");
	return W_SUCCESS;
}
