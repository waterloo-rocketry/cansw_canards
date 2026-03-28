/**
 * helpful mocking-related helper functions to use in unit tests
 */
#pragma once

#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include "application/estimator/estimator_types.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/timer/timer.h"
}

// helpers for the timer_get_ms function

// w_status_t timer_get_ms(uint32_t *time_ms);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);

// static makes this accessible to only the *_test.cpp file that includes this header.
// test file should modify this variable to set the timer value as needed
static uint32_t mock_timer_ms = 0;

// in test setups, set the custom_fake of timer_get_ms to this function
static w_status_t timer_get_ms_custom_fake(uint32_t *time_ms) {
    *time_ms = mock_timer_ms;
    return W_SUCCESS;
}