#include "fff.h"
#include <gtest/gtest.h>
#include <math.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/ak45_driver/ak45_calibration.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "task.h"

DEFINE_FFF_GLOBALS;
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *, ...);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
FAKE_VALUE_FUNC(w_status_t, ak45_get_latest_feedback, ak45_feedback_t *);
FAKE_VALUE_FUNC(w_status_t, ak45_send_position_cmd, float32_t);
FAKE_VALUE_FUNC(w_status_t, ak45_send_set_origin);
FAKE_VALUE_FUNC(w_status_t, ak45_send_disable_cmd);
}

typedef enum {
	SCENARIO_TIMEOUT,
	SCENARIO_TAP_MISMATCH,
	SCENARIO_BAD_SPAN,
	SCENARIO_SUCCESS,
} feedback_scenario_t;

static feedback_scenario_t g_scenario = SCENARIO_SUCCESS;
static uint32_t g_fake_time_ms = 0;
static uint32_t g_fb_timestamp = 0;
static float g_prev_cmd_deg = 0.0f;
static int g_neg_tap_index = 0;
static int g_pos_tap_index = 0;

static w_status_t timer_get_ms_increment(uint32_t *time) {
	g_fake_time_ms += 30;
	*time = g_fake_time_ms;
	return W_SUCCESS;
}

static w_status_t ak45_send_position_cmd_track(float32_t angle_deg) {
	if ((angle_deg < -11.0f) && (g_prev_cmd_deg > -11.0f) && (g_prev_cmd_deg < 0.0f)) {
		g_neg_tap_index++;
	}

	if ((angle_deg > 11.0f) && (g_prev_cmd_deg > 0.0f) && (g_prev_cmd_deg < 11.0f)) {
		g_pos_tap_index++;
	}

	g_prev_cmd_deg = angle_deg;
	return W_SUCCESS;
}

static void fill_stall(ak45_feedback_t *fb, float32_t position_deg) {
	fb->position_deg = position_deg;
	fb->speed_erpm = 0.0f;
	fb->current_a = 2.0f;
	fb->fault_code = AK45_FAULT_NONE;
	fb->timestamp_ms = ++g_fb_timestamp;
}

static void fill_moving(ak45_feedback_t *fb, float32_t position_deg) {
	fb->position_deg = position_deg;
	fb->speed_erpm = 200.0f;
	fb->current_a = 0.1f;
	fb->fault_code = AK45_FAULT_NONE;
	fb->timestamp_ms = ++g_fb_timestamp;
}

static w_status_t ak45_get_latest_feedback_scenario(ak45_feedback_t *fb) {
	switch (g_scenario) {
		case SCENARIO_TIMEOUT:
			fill_moving(fb, 0.0f);
			break;

		case SCENARIO_TAP_MISMATCH:
			if (g_prev_cmd_deg < -11.0f) {
				fill_stall(fb, (g_neg_tap_index == 0) ? -10.0f : -12.0f);
			} else if (g_prev_cmd_deg > 11.0f) {
				fill_stall(fb, 10.0f);
			} else {
				fill_stall(fb, -8.5f);
			}
			break;

		case SCENARIO_BAD_SPAN:
			if (g_prev_cmd_deg < -11.0f) {
				fill_stall(fb, -5.0f);
			} else if (g_prev_cmd_deg > 11.0f) {
				fill_stall(fb, 5.0f);
			} else {
				fill_stall(fb, 0.0f);
			}
			break;

		case SCENARIO_SUCCESS:
		default:
			if (g_prev_cmd_deg < -11.0f) {
				fill_stall(fb, (g_neg_tap_index == 0) ? -10.0f : -10.05f);
			} else if (g_prev_cmd_deg > 11.0f) {
				fill_stall(fb, (g_pos_tap_index == 0) ? 10.0f : 10.05f);
			} else if (fabsf(g_prev_cmd_deg) <= 1.0f) {
				fill_stall(fb, 0.0f);
			} else {
				fill_stall(fb, g_prev_cmd_deg);
			}
			break;
	}

	return W_SUCCESS;
}

static ak45_calibration_config_t fast_test_config(void) {
	ak45_calibration_config_t cfg = ak45_calibration_config_default;
	cfg.stall_hold_ms = 40;
	cfg.stall_sample_count = 2;
	cfg.backoff_settle_ms = 40;
	cfg.seek_timeout_ms = 5000;
	cfg.settle_timeout_ms = 2000;
	return cfg;
}

class AK45CalibrationTest : public ::testing::Test {
protected:
	void SetUp() override {
		RESET_FAKE(log_text);
		RESET_FAKE(timer_get_ms);
		RESET_FAKE(ak45_get_latest_feedback);
		RESET_FAKE(ak45_send_position_cmd);
		RESET_FAKE(ak45_send_set_origin);
		RESET_FAKE(ak45_send_disable_cmd);
		RESET_FAKE(vTaskDelay);
		FFF_RESET_HISTORY();

		g_scenario = SCENARIO_SUCCESS;
		g_fake_time_ms = 0;
		g_fb_timestamp = 0;
		g_prev_cmd_deg = 0.0f;
		g_neg_tap_index = 0;
		g_pos_tap_index = 0;

		timer_get_ms_fake.custom_fake = timer_get_ms_increment;
		ak45_get_latest_feedback_fake.custom_fake = ak45_get_latest_feedback_scenario;
		ak45_send_position_cmd_fake.custom_fake = ak45_send_position_cmd_track;
		ak45_send_set_origin_fake.return_val = W_SUCCESS;
		ak45_send_disable_cmd_fake.return_val = W_SUCCESS;
	}
};

TEST_F(AK45CalibrationTest, FailsOnNullConfig) {
	EXPECT_EQ(ak45_hard_stop_calibrate(NULL), W_INVALID_PARAM);
	EXPECT_EQ(ak45_send_disable_cmd_fake.call_count, 0);
}

TEST_F(AK45CalibrationTest, FailsOnInvalidConfig) {
	ak45_calibration_config_t cfg = ak45_calibration_config_default;
	cfg.min_span_deg = 25.0f;
	cfg.max_span_deg = 20.0f;

	EXPECT_EQ(ak45_hard_stop_calibrate(&cfg), W_INVALID_PARAM);
	EXPECT_EQ(ak45_send_disable_cmd_fake.call_count, 0);
}

TEST_F(AK45CalibrationTest, FailsOnSeekTimeout) {
	g_scenario = SCENARIO_TIMEOUT;
	ak45_calibration_config_t cfg = fast_test_config();
	cfg.seek_timeout_ms = 200;

	EXPECT_EQ(ak45_hard_stop_calibrate(&cfg), W_FAILURE);
	EXPECT_EQ(ak45_send_disable_cmd_fake.call_count, 1);
}

TEST_F(AK45CalibrationTest, FailsOnTapMismatch) {
	g_scenario = SCENARIO_TAP_MISMATCH;
	ak45_calibration_config_t cfg = fast_test_config();

	EXPECT_EQ(ak45_hard_stop_calibrate(&cfg), W_FAILURE);
	EXPECT_EQ(ak45_send_disable_cmd_fake.call_count, 1);
}

TEST_F(AK45CalibrationTest, FailsOnSpanOutOfRange) {
	g_scenario = SCENARIO_BAD_SPAN;
	ak45_calibration_config_t cfg = fast_test_config();

	EXPECT_EQ(ak45_hard_stop_calibrate(&cfg), W_FAILURE);
	EXPECT_GE(ak45_send_disable_cmd_fake.call_count, 1);
}

TEST_F(AK45CalibrationTest, SucceedsFullSequence) {
	g_scenario = SCENARIO_SUCCESS;
	ak45_calibration_config_t cfg = fast_test_config();

	EXPECT_EQ(ak45_hard_stop_calibrate(&cfg), W_SUCCESS);
	EXPECT_EQ(ak45_send_set_origin_fake.call_count, 1);
	EXPECT_EQ(ak45_send_disable_cmd_fake.call_count, 0);
	EXPECT_GT(ak45_send_position_cmd_fake.call_count, 0);
}
