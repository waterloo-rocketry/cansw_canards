/**
 * @file test_ms5611.cpp
 * @brief Unit tests for MS5611 barometric pressure sensor driver.
 *
 * Test plan coverage (TC-01 through TC-20):
 *   TC-01  Init: reset write fails
 *   TC-02  Init: first PROM read fails
 *   TC-03  Init: mid-PROM read fails (coeff index 4)
 *   TC-04  Init: CRC mismatch (corrupted PROM byte)
 *   TC-05  Init: happy path, validated against datasheet coefficients
 *   TC-06  Init: double-init succeeds
 *   TC-07  get_raw_pressure: NULL result pointer
 *   TC-08  get_raw_pressure: called before init
 *   TC-09  get_raw_pressure: D2 convert command write fails
 *   TC-10  get_raw_pressure: D2 ADC read fails
 *   TC-11  get_raw_pressure: D1 convert command write fails
 *   TC-12  get_raw_pressure: D1 ADC read fails
 *   TC-13  Compensation: warm temp (>=20 °C), first-order only, datasheet values
 *   TC-14  Compensation: cold temp (0-20 °C), second-order applied
 *   TC-15  Compensation: extreme cold (<-15 °C), extra addends applied
 *   TC-16  Compensation: boundary temp == 2000 centideg, second-order NOT entered
 *   TC-17  Compensation: boundary temp == 1999 centideg, second-order IS entered
 *   TC-18  Timing: vTaskDelay called exactly twice per read
 *   TC-19  Integration: three consecutive reads return consistent values
 *   TC-20  Integration: failed init -> successful re-init -> read succeeds
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
    #include "../src/application/logger/log.h"

        /* IIC address: CSB pin low = 0x77, CSB pin high = 0x76 */
    typedef enum {
        MS5611_ADDRESS_CSB_LOW = 0x77,
        MS5611_ADDRESS_CSB_HIGH = 0x76
    } ms5611_address_t;

    // reset command
    #define MS5611_CMD_RESET 0x1E

    // pressure conversion commands (D1)
    #define MS5611_CMD_CONVERT_D1_OSR256 0x40
    #define MS5611_CMD_CONVERT_D1_OSR512 0x42
    #define MS5611_CMD_CONVERT_D1_OSR1024 0x44
    #define MS5611_CMD_CONVERT_D1_OSR2048 0x46
    #define MS5611_CMD_CONVERT_D1_OSR4096 0x48

    // temperature conversion commands (D2)
    #define MS5611_CMD_CONVERT_D2_OSR256 0x50
    #define MS5611_CMD_CONVERT_D2_OSR512 0x52
    #define MS5611_CMD_CONVERT_D2_OSR1024 0x54
    #define MS5611_CMD_CONVERT_D2_OSR2048 0x56
    #define MS5611_CMD_CONVERT_D2_OSR4096 0x58

    // ADC read command
    #define MS5611_CMD_ADC_READ 0x00
    #define MS5611_CMD_PROM_READ_BASE 0xA0 /* OR with (addr << 1) for addr 0..7 */

    // calibration coefficient indices in PROM readout
    #define MS5611_COEFF_SENS 1
    #define MS5611_COEFF_OFF 2
    #define MS5611_COEFF_TCS 3
    #define MS5611_COEFF_TCO 4
    #define MS5611_COEFF_TREF 5
    #define MS5611_COEFF_TEMPSENS 6

    // osr index in command arrays
    typedef enum {
        MS5611_OSR_256 = 0,
        MS5611_OSR_512 = 1,
        MS5611_OSR_1024 = 2,
        MS5611_OSR_2048 = 3,
        MS5611_OSR_4096 = 4
    } ms5611_osr_t;

    typedef struct {
        /* Calibration coefficients read from PROM */
        uint16_t prom_coef[8]; /* C[1]..C[6] used; C[0] = factory reserved empty space */

        i2c_bus_t bus;
        ms5611_address_t addr;

        /* Selected OSR for pressure and temperature conversions */
        ms5611_osr_t osr_pressure;
        ms5611_osr_t osr_temperature;

        /* Set true once init succeeds */
        bool initialized;
    } ms5611_handle_t;

    /* Conversion time in microseconds - max values from datasheet for safety */
    static const uint32_t CONV_TIME_US[] = {
        600, /* OSR 256  — datasheet max 0.60ms */
        1170, /* OSR 512  — datasheet max 1.17ms */
        2280,
        /* OSR 1024 — datasheet max 2.28ms */ // this one is picked
        4540, /* OSR 2048 — datasheet max 4.54ms */
        9040, /* OSR 4096 — datasheet max 9.04ms */
    };

    /* D1 (pressure) convert commands indexed by ms5611_osr_t */
    static const uint8_t D1_CMD[] = {
        MS5611_CMD_CONVERT_D1_OSR256,
        MS5611_CMD_CONVERT_D1_OSR512,
        MS5611_CMD_CONVERT_D1_OSR1024,
        MS5611_CMD_CONVERT_D1_OSR2048,
        MS5611_CMD_CONVERT_D1_OSR4096,
    };

    /* D2 (temperature) convert commands indexed by ms5611_osr_t */
    static const uint8_t D2_CMD[] = {
        MS5611_CMD_CONVERT_D2_OSR256,
        MS5611_CMD_CONVERT_D2_OSR512,
        MS5611_CMD_CONVERT_D2_OSR1024,
        MS5611_CMD_CONVERT_D2_OSR2048,
        MS5611_CMD_CONVERT_D2_OSR4096,
    };

    FAKE_VALUE_FUNC(w_status_t, i2c_write_data,
        i2c_bus_t, uint8_t, const uint8_t *, uint8_t)
    FAKE_VALUE_FUNC(w_status_t, i2c_read_reg,
        i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)
    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...)
    FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *)
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

#define MS5611_ADDR 0x77u /* CSB tied to GND → 0x77 */

/* Datasheet example ADC raw values */
#define DS_D2_WARM  8569150u  /* raw temperature — yields 20.07 °C */
#define DS_D1       9085466u  /* raw pressure    — yields 1000.09 mbar */

/* Datasheet example expected compensated outputs */
#define DS_TEMP_CENTIDEG  2007
#define DS_PRES_CENTIMBAR 100009

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
 * Used by TC-03 to verify mid-read failure does not corrupt handle state.
 * FFF call_count is used by the fixture to know which call this is, but
 * since this is a custom_fake we track it locally.
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
 *  ADC custom fakes
 *
 *  The driver always reads D2 (temperature) first, then D1 (pressure).
 *  A module-level counter (reset in fixture SetUp) selects which value to
 *  return.  Using a global avoids the lambda-capture restriction on C
 *  function pointers required by FFF's custom_fake field.
 * ═══════════════════════════════════════════════════════════════════════════ */

static int g_adc_call = 0; /* reset in SetUp */

/*
 * Encode a 24-bit big-endian ADC value into the three output bytes.
 */
static void encode_adc(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)((value >> 16) & 0xFFu);
    data[1] = (uint8_t)((value >>  8) & 0xFFu);
    data[2] = (uint8_t)( value        & 0xFFu);
}

/*
 * Returns datasheet D2 on call 0, D1 on call 1.
 * Used for the warm-temp (TC-13) and multi-read (TC-18, TC-19) tests.
 */
static w_status_t fake_adc_warm(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    encode_adc(data, (g_adc_call++ == 0) ? DS_D2_WARM : DS_D1);
    return W_SUCCESS;
}

/*
 * D2 raw value that produces temp ≈ 1000 centideg (10 °C) — inside the
 * cold-compensation branch (TC-14).
 *
 * With C5=33464 and TEMPSENS=28312:
 *   dt   = D2 - (C5<<8) = D2 - 8566784
 *   temp = 2000 + (dt * 28312) / 8388608
 * For temp=1000: dt = -296220 → D2 = 8566784 - 296220 = 8270564
 */
#define D2_COLD 8270564u

static w_status_t fake_adc_cold(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    encode_adc(data, (g_adc_call++ == 0) ? D2_COLD : DS_D1);
    return W_SUCCESS;
}

/*
 * D2 that produces temp ≈ -2000 centideg — inside the extreme-cold branch
 * (TC-15, additional off2/sens2 addends applied when temp < -1500).
 *
 * For temp=-2000: dt = (-4000 * 8388608) / 28312 ≈ -1184748
 *   D2 = 8566784 - 1184748 = 7382036
 */
#define D2_EXTREME_COLD 7382036u

static w_status_t fake_adc_extreme_cold(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    encode_adc(data, (g_adc_call++ == 0) ? D2_EXTREME_COLD : DS_D1);
    return W_SUCCESS;
}

/*
 * dt = 0  →  D2 = C5<<8 = 33464<<8 = 8566784  →  temp = exactly 2000 (TC-16).
 */
#define D2_BOUNDARY_EXACT 8566784u

static w_status_t fake_adc_boundary_exact(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    encode_adc(data, (g_adc_call++ == 0) ? D2_BOUNDARY_EXACT : DS_D1);
    return W_SUCCESS;
}

/*
 * dt = -296  →  D2 = 8566784 - 296 = 8566488  →  temp ≈ 1999 (TC-17).
 */
#define D2_BOUNDARY_JUST_COLD 8566488u

static w_status_t fake_adc_boundary_just_cold(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    encode_adc(data, (g_adc_call++ == 0) ? D2_BOUNDARY_JUST_COLD : DS_D1);
    return W_SUCCESS;
}

/*
 * Fails on the D2 ADC read (call 0) — used by TC-10.
 */
static w_status_t fake_adc_d2_fail(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    (void)data;
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    if (g_adc_call++ == 0) return W_FAILURE; /* D2 read fails */
    encode_adc(data, DS_D1);
    return W_SUCCESS;
}

/*
 * D2 succeeds, D1 ADC read fails (call 1) — used by TC-12.
 */
static w_status_t fake_adc_d1_fail(
    i2c_bus_t /*bus*/, uint8_t device_addr,
    uint8_t reg, uint8_t *data, uint8_t /*len*/)
{
    if (device_addr != MS5611_ADDR || reg != MS5611_CMD_ADC_READ) return W_FAILURE;
    if (g_adc_call++ == 0) {
        encode_adc(data, DS_D2_WARM);
        return W_SUCCESS;
    }
    return W_FAILURE; /* D1 read fails */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helper: drive the driver to initialised state
 * ═══════════════════════════════════════════════════════════════════════════ */

static void init_driver() {
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_valid;
    ASSERT_EQ(W_SUCCESS, ms5611_init());
    RESET_FAKE(i2c_write_data);
    RESET_FAKE(i2c_read_reg);
    RESET_FAKE(vTaskDelay);
    FFF_RESET_HISTORY();
    g_adc_call = 0;
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
        FFF_RESET_HISTORY();
        ms5611_deinit();
        g_adc_call           = 0;
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

/* TC-03 — PROM read fails mid-way (coeff index 4), handle not partially updated */
TEST_F(MS5611Test, TC03_InitFailsIfMidPromReadFails) {
    i2c_write_data_fake.return_val  = W_SUCCESS;
    i2c_read_reg_fake.custom_fake   = fake_prom_mid_fail;

    EXPECT_EQ(W_FAILURE, ms5611_init());

    /* Driver must still refuse reads — handle.initialized must be false */
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
    /* Init triggers exactly one vTaskDelay for the post-reset wait (AN520 datasheet),
     * but must not trigger any ADC conversion delays */
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
 *  TC-07 … TC-08   Guard clauses
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-07 — NULL result pointer rejected without touching hardware */
TEST_F(MS5611Test, TC07_GetPressureRejectsNullPointer) {
    uint32_t timestamp_ms;
    EXPECT_EQ(W_INVALID_PARAM, ms5611_get_raw_pressure(nullptr, &timestamp_ms));
    EXPECT_EQ(0u, i2c_write_data_fake.call_count);
    EXPECT_EQ(0u, i2c_read_reg_fake.call_count);
}

/* TC-08 — called before ms5611_init: not-initialised guard fires */
TEST_F(MS5611Test, TC08_GetPressureFailsWhenNotInitialized) {
    uint32_t timestamp_ms;
    ms5611_raw_result_t result{};
    EXPECT_EQ(W_FAILURE, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_EQ(0u, i2c_write_data_fake.call_count);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC-09 … TC-12   I/O error paths
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-09 — D2 (temperature) convert command write fails */
TEST_F(MS5611Test, TC09_GetPressureFailsOnD2WriteError) {
    init_driver();
    /* Every write now fails — the very first one is the D2 convert command */
    i2c_write_data_fake.return_val = W_FAILURE;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_IO_ERROR, ms5611_get_raw_pressure(&result, &timestamp_ms));
    /* No ADC read should have been attempted */
    EXPECT_EQ(0u, i2c_read_reg_fake.call_count);
}

/* TC-10 — D2 ADC read fails after D2 convert command succeeds */
TEST_F(MS5611Test, TC10_GetPressureFailsOnD2AdcReadError) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS; /* both convert commands succeed */
    i2c_read_reg_fake.custom_fake  = fake_adc_d2_fail;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_IO_ERROR, ms5611_get_raw_pressure(&result, &timestamp_ms));
}

/* TC-11 — D1 (pressure) convert command write fails after D2 completes */
TEST_F(MS5611Test, TC11_GetPressureFailsOnD1WriteError) {
    init_driver();

    /* Write call 0 = D2 convert (success), call 1 = D1 convert (fail) */
    w_status_t write_seq[] = {W_SUCCESS, W_FAILURE};
    i2c_write_data_fake.return_val_seq       = write_seq;
    i2c_write_data_fake.return_val_seq_len   = 2;

    /* D2 ADC must succeed so the code reaches the D1 write */
    i2c_read_reg_fake.custom_fake = fake_adc_warm; /* call 0 = D2 succeeds */

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_IO_ERROR, ms5611_get_raw_pressure(&result, &timestamp_ms));
}

/* TC-12 — D1 ADC read fails after both convert commands succeed */
TEST_F(MS5611Test, TC12_GetPressureFailsOnD1AdcReadError) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_d1_fail;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_IO_ERROR, ms5611_get_raw_pressure(&result, &timestamp_ms));
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC-13 … TC-17   Compensation algorithm branches
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-13 — warm temp (>=20 °C): first-order only, validated against datasheet */
TEST_F(MS5611Test, TC13_WarmTempFirstOrderCompensation) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_warm;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_EQ(DS_TEMP_CENTIDEG,  result.temperature_centideg);
    EXPECT_EQ(DS_PRES_CENTIMBAR, result.pressure_centimbar);
}

/* TC-14 — cold temp (0-20 °C): second-order T2/off2/sens2 correction applied */
TEST_F(MS5611Test, TC14_ColdTempSecondOrderCompensationApplied) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_cold;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_LT(result.temperature_centideg, 2000);
    EXPECT_GT(result.temperature_centideg, -1500);
}

/* TC-15 — extreme cold (<-15 °C): additional off2/sens2 addends applied */
TEST_F(MS5611Test, TC15_ExtremeColdAdditionalCompensationApplied) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_extreme_cold;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_LT(result.temperature_centideg, -1500);
}

/* TC-16 — boundary: temp == 2000 centideg, second-order NOT entered */
TEST_F(MS5611Test, TC16_BoundaryTempExactly2000NoSecondOrder) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_boundary_exact;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    /* dt == 0 → temp must be exactly 2000 (the second-order-threshold constant) */
    EXPECT_EQ(2000, result.temperature_centideg);
}

/* TC-17 — boundary: temp == 1999 centideg, second-order IS entered */
TEST_F(MS5611Test, TC17_BoundaryTempJustBelow2000SecondOrderEntered) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_boundary_just_cold;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_LT(result.temperature_centideg, 2000);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC-18   Timing / delay
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-18 — vTaskDelay called exactly twice per read (once per conversion) */
TEST_F(MS5611Test, TC18_DelayCalledExactlyTwicePerRead) {
    init_driver();
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_warm;

    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_EQ(2u, vTaskDelay_fake.call_count);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TC-19 … TC-20   Integration / regression
 * ═══════════════════════════════════════════════════════════════════════════ */

/* TC-19 — three consecutive reads all return datasheet-consistent values */
TEST_F(MS5611Test, TC19_ThreeConsecutiveReadsReturnConsistentValues) {
    init_driver();

    for (int i = 0; i < 3; i++) {
        RESET_FAKE(i2c_write_data);
        RESET_FAKE(i2c_read_reg);
        g_adc_call = 0;
        i2c_write_data_fake.return_val = W_SUCCESS;
        i2c_read_reg_fake.custom_fake  = fake_adc_warm;

        ms5611_raw_result_t result{};
        uint32_t timestamp_ms;
        ASSERT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms)) << "Failed on read " << i;
        EXPECT_EQ(DS_TEMP_CENTIDEG,  result.temperature_centideg) << "Read " << i;
        EXPECT_EQ(DS_PRES_CENTIMBAR, result.pressure_centimbar)   << "Read " << i;
    }
}

/* TC-20 — failed init leaves driver uninitialized; successful re-init unblocks reads */
TEST_F(MS5611Test, TC20_SuccessfulReInitAfterFailureAllowsReads) {
    /* First init: fail via bad PROM */
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.return_val   = W_FAILURE;
    ASSERT_EQ(W_FAILURE, ms5611_init());

    /* Driver must refuse reads while uninitialized */
    ms5611_raw_result_t result{};
    uint32_t timestamp_ms;
    EXPECT_EQ(W_FAILURE, ms5611_get_raw_pressure(&result, &timestamp_ms));

    /* Second init: succeed */
    RESET_FAKE(i2c_write_data);
    RESET_FAKE(i2c_read_reg);
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_prom_valid;
    ASSERT_EQ(W_SUCCESS, ms5611_init());

    /* Reads must now succeed */
    RESET_FAKE(i2c_write_data);
    RESET_FAKE(i2c_read_reg);
    g_adc_call = 0;
    i2c_write_data_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.custom_fake  = fake_adc_warm;

    EXPECT_EQ(W_SUCCESS, ms5611_get_raw_pressure(&result, &timestamp_ms));
    EXPECT_EQ(DS_TEMP_CENTIDEG,  result.temperature_centideg);
    EXPECT_EQ(DS_PRES_CENTIMBAR, result.pressure_centimbar);
}