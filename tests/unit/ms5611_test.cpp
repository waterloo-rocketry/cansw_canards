/**
 * @file ms5611_test.cpp
 * @brief Unit tests for MS5611 barometric pressure sensor driver.
 *   TC-01 … TC-06  ms5611_init (reset/PROM/CRC paths, double-init guard)
 *   TC-07 … TC-11  ms5611_get_raw_pressure guard clauses and cache-status path
 */

#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
    #include "FreeRTOS.h"
    #include "../src/drivers/MS5611/MS5611.h"
    #include "../src/drivers/timer/timer.h"
    #include "../src/drivers/i2c/i2c.h"
    #include "../src/drivers/gpio/gpio.h"
    #include "../mocks/task.h"
    #include "../mocks/semphr.h"
    #include "../src/application/logger/log.h"

    // reset command
    #define MS5611_CMD_RESET 0x1E

    // ADC read command
    #define MS5611_CMD_ADC_READ 0x00
    #define MS5611_CMD_PROM_READ_BASE 0xA0 /* OR with (addr << 1) for addr 0..7 */

    FAKE_VALUE_FUNC(w_status_t, i2c_write_data,
        i2c_bus_t, uint8_t, const uint8_t *, uint8_t)
    FAKE_VALUE_FUNC(w_status_t, i2c_read_reg,
        i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)
    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *, ...)
    FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *)
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

#define MS5611_ADDR 0x77u /* CSB tied to GND → 0x77 */

/* Dummy non-NULL handle returned by the mocked xSemaphoreCreateMutex so that
 * ms5611_init believes the mutex was created successfully. */
static int g_dummy_mutex_storage = 0;
#define DUMMY_MUTEX ((SemaphoreHandle_t)&g_dummy_mutex_storage)

/* ═══════════════════════════════════════════════════════════════════════════
 *  calculates crc (CRC-4, per datasheet)
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint8_t ms5611_crc(const uint16_t n_prom[8]) {
    uint16_t prom[8];
    for (int i = 0; i < 8; i++) prom[i] = n_prom[i];
    prom[7] &= 0xFF00u;

    uint16_t crc_reg = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        crc_reg ^= (cnt % 2 == 1)
            ? (uint16_t)(prom[cnt >> 1] & 0x00FFu)
            : (uint16_t)(prom[cnt >> 1] >> 8);
        for (int n_bit = 8; n_bit > 0; n_bit--) {
            crc_reg = (crc_reg & 0x8000u)
                ? (crc_reg << 1) ^ 0x3000u
                : (crc_reg << 1);
        }
    }
    return (crc_reg >> 12) & 0x0Fu;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PROM custom fakes
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * Datasheet example coefficients (C1-C6).  prom[7] has the correct CRC
 * nibble spliced in each time so the driver's CRC check passes.
 */
static const uint16_t g_valid_prom[8] = {
    0,
    40127, 36924, 23317,
    23282, 33464, 28312,
    0   /* CRC nibble to be filled in by the fake at call time */
};

static w_status_t fake_prom_valid(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR) return W_FAILURE;

    uint16_t prom[8];
    for (int i = 0; i < 8; i++) prom[i] = g_valid_prom[i];
    prom[7] = (uint16_t)((prom[7] & 0xFF00u) | ms5611_crc(prom));

    int idx = (reg - MS5611_CMD_PROM_READ_BASE) / 2;
    if (idx < 0 || idx > 7) return W_FAILURE;

    data[0] = (uint8_t)(prom[idx] >> 8);
    data[1] = (uint8_t)(prom[idx] & 0xFFu);
    return W_SUCCESS;
}

/*
 * Same as fake_prom_valid but returns a corrupted byte for coeff[1],
 * so the CRC check in the driver will fail (TC-04).
 */
static w_status_t fake_prom_corrupt_crc(
    i2c_bus_t bus, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t len)
{
    w_status_t ret = fake_prom_valid(bus, device_addr, reg, data, len);
    /* Flip a bit in C1 (coeff index 1) to break the CRC */
    if (ret == W_SUCCESS && reg == (MS5611_CMD_PROM_READ_BASE + 2)) {
        data[0] ^= 0xFFu;
    }
    return ret;
}

/*
 * PROM read that fails on the 5th coefficient (idx 4 = C4/TCO).
 * Used by TC-03 to verify a mid-read failure aborts init.
 */
static int g_prom_mid_fail_call = 0;

static w_status_t fake_prom_mid_fail(
    i2c_bus_t bus, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t len)
{
    int n = g_prom_mid_fail_call++;
    if (n == 4) return W_FAILURE;
    return fake_prom_valid(bus, device_addr, reg, data, len);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helper: drive the driver to initialised state
 * ═══════════════════════════════════════════════════════════════════════════ */

static void init_driver() {
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_valid;
    xSemaphoreCreateMutex_fake.return_val = DUMMY_MUTEX;
    ASSERT_EQ(W_SUCCESS, ms5611_init());
    RESET_FAKE(i2c_write_data);
    RESET_FAKE(i2c_read_reg);
    RESET_FAKE(vTaskDelay);
    FFF_RESET_HISTORY();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test fixture
 * ═══════════════════════════════════════════════════════════════════════════ */

class MS5611Test : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(i2c_write_data);
        RESET_FAKE(i2c_read_reg);
        RESET_FAKE(log_text);
        RESET_FAKE(vTaskDelay);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        FFF_RESET_HISTORY();

        /* Mutex creation always succeeds so init isn't blocked on it. Note the
         * driver caches the mutex in a private static across tests, so once
         * created it is reused regardless of this return value. */
        xSemaphoreCreateMutex_fake.return_val = DUMMY_MUTEX;

        ms5611_deinit();
        g_prom_mid_fail_call = 0;
    }
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC-01 … TC-06   ms5611_init
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-01 — reset command write fails, init aborts immediately */
TEST_F(MS5611Test, TC01_InitFailsIfResetWriteFails) {
    i2c_write_data_fake.return_val = W_FAILURE;

    EXPECT_EQ(W_FAILURE, ms5611_init());
    /* The PROM must not have been touched */
    EXPECT_EQ(0u, i2c_read_reg_fake.call_count);
    /* An error must have been logged */
    EXPECT_GE(log_text_fake.call_count, 1u);
}

/* TC-02 — first PROM coefficient read fails */
TEST_F(MS5611Test, TC02_InitFailsIfFirstPromReadFails) {
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.return_val   = W_FAILURE;

    EXPECT_EQ(W_FAILURE, ms5611_init());
}

/* TC-03 — PROM read fails mid-way (coeff index 4); init aborts, driver stays uninitialised */
TEST_F(MS5611Test, TC03_InitFailsIfMidPromReadFails) {
    i2c_write_data_fake.return_val  = W_SUCCESS;
    i2c_read_reg_fake.custom_fake   = fake_prom_mid_fail;

    EXPECT_EQ(W_FAILURE, ms5611_init());

    /* Driver must still refuse reads — not initialised */
    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_FAILURE, ms5611_get_raw_pressure(&result, &timestamp_ms));
}

/* TC-04 — CRC mismatch: corrupted PROM byte produces wrong nibble */
TEST_F(MS5611Test, TC04_InitFailsOnCrcMismatch) {
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_corrupt_crc;

    EXPECT_EQ(W_FAILURE, ms5611_init());
    EXPECT_GE(log_text_fake.call_count, 1u);
}

/* TC-05 — happy path: reset + 8 valid PROM reads + correct CRC */
TEST_F(MS5611Test, TC05_InitSucceedsWithValidProm) {
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_valid;

    EXPECT_EQ(W_SUCCESS, ms5611_init());
    /* Init triggers exactly one vTaskDelay for the post-reset wait (AN520 datasheet) */
    EXPECT_EQ(1u, vTaskDelay_fake.call_count);
}

/* TC-06 — double init: second fails since already initialized */
TEST_F(MS5611Test, TC06_DoubleInitCauseFail) {
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_valid;

    ASSERT_EQ(W_SUCCESS, ms5611_init());

    RESET_FAKE(i2c_write_data);
    RESET_FAKE(i2c_read_reg);
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_valid;

    EXPECT_EQ(W_FAILURE, ms5611_init());
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC-07 … TC-11   ms5611_get_raw_pressure (non-blocking cache getter)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-07 — NULL result pointer rejected without touching hardware */
TEST_F(MS5611Test, TC07_GetPressureRejectsNullResult) {
    init_driver();
    uint32_t timestamp_ms;
    EXPECT_EQ(W_INVALID_PARAM, ms5611_get_raw_pressure(nullptr, &timestamp_ms));
    EXPECT_EQ(0u, i2c_write_data_fake.call_count);
    EXPECT_EQ(0u, i2c_read_reg_fake.call_count);
}

/* TC-08 — NULL timestamp pointer rejected without touching hardware */
TEST_F(MS5611Test, TC08_GetPressureRejectsNullTimestamp) {
    init_driver();
    ms5611_raw_result_t result{};
    EXPECT_EQ(W_INVALID_PARAM, ms5611_get_raw_pressure(&result, nullptr));
    EXPECT_EQ(0u, i2c_write_data_fake.call_count);
    EXPECT_EQ(0u, i2c_read_reg_fake.call_count);
}

/* TC-09 — called before ms5611_init: not-initialised guard fires */
TEST_F(MS5611Test, TC09_GetPressureFailsWhenNotInitialized) {
    uint32_t timestamp_ms;
    ms5611_raw_result_t result{};
    EXPECT_EQ(W_FAILURE, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_EQ(0u, i2c_write_data_fake.call_count);
    EXPECT_EQ(0u, i2c_read_reg_fake.call_count);
}

/* TC-10 — mutex currently held by the sampling task: getter backs off, returns failure */
TEST_F(MS5611Test, TC10_GetPressureFailsWhenMutexBusy) {
    init_driver();
    xSemaphoreTake_fake.return_val = pdFALSE; /* cannot acquire */

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_FAILURE, ms5611_get_raw_pressure(&result, &timestamp_ms));
    /* Never touches hardware, and must not give back a semaphore it didn't take */
    EXPECT_EQ(0u, i2c_write_data_fake.call_count);
    EXPECT_EQ(0u, xSemaphoreGive_fake.call_count);
}

/* TC-11 — mutex acquired but no successful sample cached yet: returns the cached
 * (failure) status and releases the mutex */
TEST_F(MS5611Test, TC11_GetPressureReturnsCachedFailureStatus) {
    init_driver();
    xSemaphoreTake_fake.return_val = pdTRUE; /* acquires cache mutex */

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    /* No successful read has been cached, so the latest status is a failure */
    EXPECT_EQ(W_FAILURE, ms5611_get_raw_pressure(&result, &timestamp_ms));
    /* Mutex must be released after reading the cache */
    EXPECT_EQ(1u, xSemaphoreTake_fake.call_count);
    EXPECT_EQ(1u, xSemaphoreGive_fake.call_count);
}
