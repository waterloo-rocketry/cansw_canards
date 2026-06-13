#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/i2c/i2c.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"

// Mock definitions
FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *, uint8_t)
FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...)
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *)
}


static bool self_test_enabled = false;
static uint32_t current_time_ms = 0;
static uint8_t who_am_i_val = 0x40; //expected who_am_i register

// Simulates time passing to avoid timeouts
static w_status_t timer_get_ms_custom(uint32_t *ms) {
    if (ms == NULL) return W_FAILURE;
    *ms = current_time_ms;
    current_time_ms += 2; 
    return W_SUCCESS;
}


static w_status_t i2c_write_reg_custom(i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len) {
    (void)bus;
    (void)dev_addr;
    (void)len;
    if (reg == 0x62) { 
        if (*data & (1 << 1)) { 
            self_test_enabled = true;
        } else {
            self_test_enabled = false;
        }
    }
    return W_SUCCESS;
}


static w_status_t i2c_read_reg_custom(i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len) {
    (void)bus;
    (void)dev_addr;
    (void)len;
    if (reg == 0x4F) { 
        *data = who_am_i_val;
    } else if (reg == 0x67) { 
        *data = 0x08; 
    } else if (reg == (0x68 | 0x80)) { 
        int16_t val = self_test_enabled ? 200 : 100; 
        
        
        data[0] = val & 0xFF;        
        data[1] = (val >> 8) & 0xFF; 
        data[2] = val & 0xFF;        
        data[3] = (val >> 8) & 0xFF; 
        data[4] = val & 0xFF;        
        data[5] = (val >> 8) & 0xFF; 
    }
    return W_SUCCESS;
}

class IIS2MDCTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(vTaskDelay);
        RESET_FAKE(i2c_write_reg);
        RESET_FAKE(i2c_read_reg);
        RESET_FAKE(log_text);
        RESET_FAKE(timer_get_ms);

        
        self_test_enabled = false;
        current_time_ms = 0;
        who_am_i_val = 0x40;

        timer_get_ms_fake.custom_fake = timer_get_ms_custom;
        i2c_write_reg_fake.custom_fake = i2c_write_reg_custom;
        i2c_read_reg_fake.custom_fake = i2c_read_reg_custom;
    }
};

TEST_F(IIS2MDCTest, InitSuccess) {
    // 
    w_status_t status = iis2mdc_init();
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(IIS2MDCTest, InitFail_WrongWhoAmI) {
    who_am_i_val = 0x00; 
    w_status_t status = iis2mdc_init();
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(IIS2MDCTest, InitFail_SelfTestDeltaTooSmall) {
    // return same field value for self test to fail
    i2c_read_reg_fake.custom_fake = [](i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len) -> w_status_t {
        (void)bus;
        (void)dev_addr;
        (void)len;
        if (reg == 0x4F) *data = 0x40;
        else if (reg == 0x67) *data = 0x08;
        else if (reg == (0x68 | 0x80)) {
            int16_t val = 100; 
            data[0] = val & 0xFF; data[1] = (val >> 8) & 0xFF;
            data[2] = val & 0xFF; data[3] = (val >> 8) & 0xFF;
            data[4] = val & 0xFF; data[5] = (val >> 8) & 0xFF;
        }
        return W_SUCCESS;
    };
    
    w_status_t status = iis2mdc_init();
    EXPECT_EQ(status, W_FAILURE);
}
