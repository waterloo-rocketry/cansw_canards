/**
 * helpful mocking-related helper functions to use in unit tests
 */
#pragma once

#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include "application/logger/log.h"
}

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
FAKE_VALUE_FUNC(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);