#include "fff.h"
#include <gtest/gtest.h>

DEFINE_FFF_GLOBALS;

extern "C" {

#include "FreeRTOS.h"
#include "application/telemetry/telemetry.h"
#include "common/gnc/gnc_types.h"
#include "drivers/timer/timer.h"
#include "task.h"

// Dependencies of the telemetry module (declared inline; no mock files exist).
FAKE_VALUE_FUNC(fsm_state_t, fsm_get_state);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *, ...);

// Test doubles for telemetry source log_fn callbacks.
FAKE_VALUE_FUNC(w_status_t, source_log_ok);
FAKE_VALUE_FUNC(w_status_t, source_log_fail);

}

// Controllable "current time" returned by the timer_get_ms fake.
static uint32_t g_mock_now_ms = 0;
static w_status_t g_mock_timer_status = W_SUCCESS;

static w_status_t timer_get_ms_custom(uint32_t *p_ms) {
    if (p_ms != nullptr) {
        *p_ms = g_mock_now_ms;
    }
    return g_mock_timer_status;
}

class TelemetryModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        telemetry_init();

        RESET_FAKE(fsm_get_state);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(log_text);
        RESET_FAKE(source_log_ok);
        RESET_FAKE(source_log_fail);
        FFF_RESET_HISTORY();

        g_mock_now_ms = 0;
        g_mock_timer_status = W_SUCCESS;
        timer_get_ms_fake.custom_fake = timer_get_ms_custom;

        source_log_ok_fake.return_val = W_SUCCESS;
        source_log_fail_fake.return_val = W_FAILURE;
    }

    // Helper: build a minimal valid source config.
    static telemetry_source_config_t
    make_source(telemetry_log_fn_t log_fn, fsm_state_t phase, uint32_t period_ms) {
        telemetry_source_config_t cfg = {};
        cfg.name = "test_source";
        cfg.log_fn = log_fn;
        cfg.flight_phase_state = phase;
        cfg.period_ms = period_ms;
        cfg.last_logged_ms = 0;
        cfg.due_date_ms = 0;
        return cfg;
    }
};


TEST_F(TelemetryModuleTest, RegisterValidSourceSucceeds) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_ACT_ALLOWED, 100);
    EXPECT_EQ(telemetry_register(&cfg), W_SUCCESS);
}

TEST_F(TelemetryModuleTest, RegisterBeforeInitFails) {
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_ACT_ALLOWED, 100);
    EXPECT_EQ(telemetry_register(&cfg), W_FAILURE);
}

TEST_F(TelemetryModuleTest, RegisterNullNameIsInvalidParam) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_ACT_ALLOWED, 100);
    cfg.name = nullptr;
    EXPECT_EQ(telemetry_register(&cfg), W_INVALID_PARAM);
}

TEST_F(TelemetryModuleTest, RegisterNullLogFnIsInvalidParam) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(nullptr, STATE_ACT_ALLOWED, 100);
    EXPECT_EQ(telemetry_register(&cfg), W_INVALID_PARAM);
}

TEST_F(TelemetryModuleTest, RegisterZeroPeriodIsInvalidParam) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_ACT_ALLOWED, 0);
    EXPECT_EQ(telemetry_register(&cfg), W_INVALID_PARAM);
}

TEST_F(TelemetryModuleTest, RegisterWithPresetSchedulingFieldsWarnsButSucceeds) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_ACT_ALLOWED, 100);
    cfg.last_logged_ms = 42;
    cfg.due_date_ms = 99;
    EXPECT_EQ(telemetry_register(&cfg), W_SUCCESS);
    // A warning must have been logged for the misused scheduling fields.
    EXPECT_GE(log_text_fake.call_count, 1u);
}

TEST_F(TelemetryModuleTest, RegisterBeyondMaxSourcesFails) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    // TELEMETRY_MAX_SOURCES is a private macro (32) in telemetry.c.
    constexpr uint32_t kMaxSources = 32;
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_ACT_ALLOWED, 100);
    for (uint32_t i = 0; i < kMaxSources; i++) {
        EXPECT_EQ(telemetry_register(&cfg), W_SUCCESS) << "at source " << i;
    }
    EXPECT_EQ(telemetry_register(&cfg), W_FAILURE);
}

// ---------------------------------------------------------------------------
// get_status
// ---------------------------------------------------------------------------

TEST_F(TelemetryModuleTest, StatusHealthyByDefault) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    health_status_t status = telemetry_get_status();
    EXPECT_EQ(status.severity, HEALTH_OK);
    EXPECT_EQ(status.module_id, MODULE_TELEMETRY);
}

// ---------------------------------------------------------------------------
// scheduling: telemetry_run_once / telemetry_init_due_dates
// ---------------------------------------------------------------------------

TEST_F(TelemetryModuleTest, WrongPhaseDoesNotLog) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_BOOST, 100);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    telemetry_init_due_dates(0); // due at t=100
    fsm_get_state_fake.return_val = STATE_IDLE; // mismatched phase
    g_mock_now_ms = 500; // well past due

    telemetry_run_once();

    EXPECT_EQ(source_log_ok_fake.call_count, 0u);
}

TEST_F(TelemetryModuleTest, NotDueYetDoesNotLog) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_BOOST, 100);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    telemetry_init_due_dates(0); // due at t=100
    fsm_get_state_fake.return_val = STATE_BOOST;
    g_mock_now_ms = 50; // before due

    telemetry_run_once();

    EXPECT_EQ(source_log_ok_fake.call_count, 0u);
}

TEST_F(TelemetryModuleTest, DueExactlyLogsOnceAndAdvancesSchedule) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_BOOST, 100);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    telemetry_init_due_dates(0); // due at t=100
    fsm_get_state_fake.return_val = STATE_BOOST;

    g_mock_now_ms = 100; // exactly due
    telemetry_run_once();
    EXPECT_EQ(source_log_ok_fake.call_count, 1u);

    // Immediately re-running at the same time must NOT log again: the due date
    // was advanced to now + period (200).
    telemetry_run_once();
    EXPECT_EQ(source_log_ok_fake.call_count, 1u);

    // Advancing to the new due date logs again.
    g_mock_now_ms = 200;
    telemetry_run_once();
    EXPECT_EQ(source_log_ok_fake.call_count, 2u);

    // On-time logging keeps the module healthy.
    EXPECT_EQ(telemetry_get_status().severity, HEALTH_OK);
}

TEST_F(TelemetryModuleTest, OverdueLogsAndFlagsError) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_BOOST, 100);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    telemetry_init_due_dates(0); // due at t=100
    fsm_get_state_fake.return_val = STATE_BOOST;
    g_mock_now_ms = 150; // past due -> overdue

    telemetry_run_once();

    EXPECT_EQ(source_log_ok_fake.call_count, 1u);
    // An overdue log bumps overdue_count, which surfaces as HEALTH_ERROR.
    EXPECT_EQ(telemetry_get_status().severity, HEALTH_ERROR);
}

TEST_F(TelemetryModuleTest, FailedTransmissionFlagsError) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_fail, STATE_BOOST, 100);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    telemetry_init_due_dates(0);
    fsm_get_state_fake.return_val = STATE_BOOST;
    g_mock_now_ms = 100; // exactly due (not overdue)

    telemetry_run_once();

    EXPECT_EQ(source_log_fail_fake.call_count, 1u);
    // A failed transmission surfaces as HEALTH_ERROR.
    EXPECT_EQ(telemetry_get_status().severity, HEALTH_ERROR);
}

TEST_F(TelemetryModuleTest, TimerFailureSkipsProcessing) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_BOOST, 100);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    telemetry_init_due_dates(0);
    fsm_get_state_fake.return_val = STATE_BOOST;
    g_mock_now_ms = 500;
    g_mock_timer_status = W_FAILURE; // timer read fails

    telemetry_run_once();

    EXPECT_EQ(source_log_ok_fake.call_count, 0u);
    EXPECT_EQ(telemetry_get_status().severity, HEALTH_OK);
}

TEST_F(TelemetryModuleTest, DueDateSurvivesCounterWraparound) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t cfg = make_source(source_log_ok, STATE_BOOST, 50);
    ASSERT_EQ(telemetry_register(&cfg), W_SUCCESS);

    // Seed near the top of the 32-bit ms counter so the due date lands just
    // below UINT32_MAX; "now" then wraps around to a small value.
    telemetry_init_due_dates(UINT32_MAX - 100); // due = UINT32_MAX - 50
    fsm_get_state_fake.return_val = STATE_BOOST;
    g_mock_now_ms = 10; // wrapped past the due date

    telemetry_run_once();

    // The signed-difference compare treats this as due, not as "far in the future".
    EXPECT_EQ(source_log_ok_fake.call_count, 1u);
}

TEST_F(TelemetryModuleTest, OnlyMatchingPhaseSourceLogs) {
    ASSERT_EQ(telemetry_init(), W_SUCCESS);
    telemetry_source_config_t boost_src = make_source(source_log_ok, STATE_BOOST, 100);
    telemetry_source_config_t idle_src = make_source(source_log_fail, STATE_IDLE, 100);
    ASSERT_EQ(telemetry_register(&boost_src), W_SUCCESS);
    ASSERT_EQ(telemetry_register(&idle_src), W_SUCCESS);

    telemetry_init_due_dates(0); // both due at t=100
    fsm_get_state_fake.return_val = STATE_BOOST;
    g_mock_now_ms = 100;

    telemetry_run_once();

    EXPECT_EQ(source_log_ok_fake.call_count, 1u); // boost source logged
    EXPECT_EQ(source_log_fail_fake.call_count, 0u); // idle source skipped
}
