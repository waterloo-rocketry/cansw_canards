#include "fff.h"
#include <gtest/gtest.h>


DEFINE_FFF_GLOBALS;

extern "C" {
    #include "FreeRTOS.h"
    #include "queue.h"
    #include "task.h"
    #include "application/power_handler/power_handler.h"
    #include "application/logger/log.h"
    #include "drivers/adc/adc.h"
    #include "drivers/gpio/gpio.h"
    #include "drivers/timer/timer.h"
    #include "message_types.h"
    #include "rocketlib/include/common.h"

    // gpio fakes
    FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t);
    FAKE_VALUE_FUNC(w_status_t, gpio_read, gpio_pin_t, gpio_level_t*, uint32_t);
    // adc fakes
    FAKE_VALUE_FUNC(w_status_t, adc_get_converted_val, adc_channel_t, uint32_t*);
    // can handler fakes
    FAKE_VALUE_FUNC(w_status_t, can_handler_register_callback, can_msg_type_t, can_callback_t);
    FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
    FAKE_VALUE_FUNC(w_status_t, get_actuator_id, const can_msg_t *, can_actuator_id_t *);
    FAKE_VALUE_FUNC(w_status_t, get_cmd_actuator_state, const can_msg_t *, can_actuator_state_t *);
    FAKE_VALUE_FUNC(w_status_t, get_reset_board_id, const can_msg_t *, uint8_t *, uint8_t *);
    FAKE_VALUE_FUNC(w_status_t, check_board_need_reset, const can_msg_t *, bool *);
    // can msg fakes
    FAKE_VOID_FUNC(build_actuator_status_msg, can_msg_prio_t, uint16_t, can_actuator_id_t, can_actuator_state_t,
							   can_actuator_state_t , can_msg_t *);
    FAKE_VOID_FUNC(build_analog_sensor_16bit_msg, can_msg_prio_t , uint16_t ,
								   can_analog_sensor_id_t , uint16_t ,
								   can_msg_t *);
    FAKE_VOID_FUNC(build_general_board_status_msg, can_msg_prio_t , uint16_t ,
									uint32_t , can_msg_t *);
    // system reset fakes
    FAKE_VOID_FUNC(NVIC_SystemReset);
    // logging and timer fakes
    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
    FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);

}

// Captured CAN callbacks registered during power_handler_init()
static w_status_t (*actuator_cb)(const can_msg_t *) = nullptr;
static w_status_t (*reset_cb)(const can_msg_t *)    = nullptr;

// Stores each callback by message type so tests can invoke them directly
static w_status_t register_callback_fake(can_msg_type_t type, w_status_t (*cb)(const can_msg_t *)) {
    if (type == MSG_ACTUATOR_CMD) actuator_cb = cb;
    if (type == MSG_RESET_CMD)    reset_cb    = cb;
    return W_SUCCESS;
}

// Helper: fake adc_get_converted_val that writes a caller-supplied value into *out
static uint32_t adc_injected_value = 0;
static w_status_t adc_return_injected(adc_channel_t /*ch*/, uint32_t *out) {
    *out = adc_injected_value;
    return W_SUCCESS;
}

// Helper: fake gpio_read that returns a caller-supplied level
static gpio_level_t gpio_injected_level = GPIO_LEVEL_HIGH;
static w_status_t gpio_read_injected(gpio_pin_t /*pin*/, gpio_level_t *out, uint32_t /*timeout*/) {
    *out = gpio_injected_level;
    return W_SUCCESS;
}

class power_handler_test : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(gpio_write);
        RESET_FAKE(gpio_read);
        RESET_FAKE(adc_get_converted_val);
        RESET_FAKE(can_handler_register_callback);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(get_actuator_id);
        RESET_FAKE(get_cmd_actuator_state);
        RESET_FAKE(get_reset_board_id);
        RESET_FAKE(check_board_need_reset);
        RESET_FAKE(build_actuator_status_msg);
        RESET_FAKE(build_analog_sensor_16bit_msg);
        RESET_FAKE(build_general_board_status_msg);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(NVIC_SystemReset);
        FFF_RESET_HISTORY();

        gpio_write_fake.return_val                  = W_SUCCESS;
        gpio_read_fake.return_val                   = W_SUCCESS;
        adc_get_converted_val_fake.return_val       = W_SUCCESS;
        can_handler_register_callback_fake.return_val = W_SUCCESS;
        can_handler_transmit_fake.return_val        = W_SUCCESS;

        actuator_cb = nullptr;
        reset_cb    = nullptr;
        adc_injected_value   = 0;
        gpio_injected_level  = GPIO_LEVEL_HIGH;

        // Capture the CAN callbacks registered by power_handler_init()
        can_handler_register_callback_fake.custom_fake = register_callback_fake;
    }

    // Convenience: simulate an actuator CAN command arriving on the wire.
    // Primes get_actuator_id and get_cmd_actuator_state fakes, then fires the callback.
    w_status_t send_actuator_cmd(can_actuator_id_t id, can_actuator_state_t state) {
        static can_actuator_id_t   s_id;
        static can_actuator_state_t s_state;
        s_id    = id;
        s_state = state;

        get_actuator_id_fake.custom_fake =
            [](const can_msg_t *, can_actuator_id_t *out) -> w_status_t {
                *out = s_id; return W_SUCCESS;
            };
        get_cmd_actuator_state_fake.custom_fake =
            [](const can_msg_t *, can_actuator_state_t *out) -> w_status_t {
                *out = s_state; return W_SUCCESS;
            };

        can_msg_t dummy = {0};
        return actuator_cb ? actuator_cb(&dummy) : W_FAILURE;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Section 1 — Initialization / defaults
// ─────────────────────────────────────────────────────────────────────────────

// power_handler_init() shall default CHG_MUX_EN HIGH
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_HIGH) {
    power_handler_init();

    bool found = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_CHG_MUX_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "Expected CHG_MUX_EN to be set HIGH during init";
}

// power_handler_init() shall default EN_EXT_5V HIGH (external 5V on)
TEST_F(power_handler_test, DEFAULT_5V_EXTERNAL_ON) {
    power_handler_init();

    bool found = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "Expected EN_EXT_5V to be set HIGH during init";
}

// power_handler_init() shall default PWR_EN HIGH (LiPo on)
TEST_F(power_handler_test, DEFAULT_LIPO_ON) {
    power_handler_init();

    bool found = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "Expected PWR_EN to be set HIGH during init";
}

// power_handler_init() shall register CAN callbacks for ACTUATOR_CMD and RESET_CMD
TEST_F(power_handler_test, INIT_REGISTERS_CAN_CALLBACKS) {
    power_handler_init();

    EXPECT_NE(actuator_cb, nullptr) << "MSG_ACTUATOR_CMD callback not registered";
    EXPECT_NE(reset_cb,    nullptr) << "MSG_RESET_CMD callback not registered";
}

// power_handler_init() shall return W_SUCCESS when both callbacks register successfully
TEST_F(power_handler_test, INIT_RETURNS_SUCCESS) {
    EXPECT_EQ(power_handler_init(), W_SUCCESS);
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 2 — RocketCAN actuator commands
// ─────────────────────────────────────────────────────────────────────────────

// Shall enable external 5V rail in response to CANARD_5V_OUTPUT ACT_STATE_ON command
TEST_F(power_handler_test, EN_5V_EXTERNAL_CMD) {
    power_handler_init();
    power_handler_set_low_power_mode(false); // ensure precondition: not in low-power

    // All ADC reads return 0 → CHG is not the active source
    adc_get_converted_val_fake.return_val = W_SUCCESS; // values already 0 from memset

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = send_actuator_cmd(ACTUATOR_CANARD_5V_OUTPUT, ACT_STATE_ON);

    EXPECT_EQ(result, W_SUCCESS);

    // EN_EXT_5V must be driven HIGH
    bool en_high = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH) {
            en_high = true;
        }
    }
    EXPECT_TRUE(en_high) << "EN_EXT_5V should be HIGH after enable command";
}

// Shall disable external 5V rail in response to CANARD_5V_OUTPUT ACT_STATE_OFF command
TEST_F(power_handler_test, DIS_5V_EXTERNAL_CMD) {
    power_handler_init();

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = send_actuator_cmd(ACTUATOR_CANARD_5V_OUTPUT, ACT_STATE_OFF);

    EXPECT_EQ(result, W_SUCCESS);

    bool en_low = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            en_low = true;
        }
    }
    EXPECT_TRUE(en_low) << "EN_EXT_5V should be LOW after disable command";
}

// Shall enable LiPo (exit low power mode) in response to CANARD_LIPO_ON ACT_STATE_ON
TEST_F(power_handler_test, EN_LIPO_CMD) {
    power_handler_init();
    power_handler_set_low_power_mode(true); // start in low power

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = send_actuator_cmd(ACTUATOR_CANARD_LIPO_ON, ACT_STATE_ON);

    EXPECT_EQ(result, W_SUCCESS);

    // PWR_EN must be driven HIGH to re-enable LiPo
    bool pwr_high = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH) {
            pwr_high = true;
        }
    }
    EXPECT_TRUE(pwr_high) << "PWR_EN should be HIGH when LiPo enable command received";
}

// Shall disable LiPo (enter low power mode) in response to CANARD_LIPO_ON ACT_STATE_OFF
TEST_F(power_handler_test, DIS_LIPO_CMD) {
    power_handler_init();

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = send_actuator_cmd(ACTUATOR_CANARD_LIPO_ON, ACT_STATE_OFF);

    EXPECT_EQ(result, W_SUCCESS);

    bool pwr_low = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            pwr_low = true;
        }
    }
    EXPECT_TRUE(pwr_low) << "PWR_EN should be LOW when LiPo disable command received";
}

// Shall trigger NVIC_SystemReset() when a RESET_CMD targeting this board arrives
TEST_F(power_handler_test, RESET_CMD_TRIGGERS_SYSTEM_RESET) {
    power_handler_init();

    // Prime the reset callback: board needs reset = true
    check_board_need_reset_fake.custom_fake =
        [](const can_msg_t *, bool *out) -> w_status_t { *out = true; return W_SUCCESS; };
    get_reset_board_id_fake.return_val = W_SUCCESS;

    can_msg_t dummy = {0};
    ASSERT_NE(reset_cb, nullptr);
    reset_cb(&dummy);

    EXPECT_EQ(NVIC_SystemReset_fake.call_count, 1u);
}

// Shall NOT call NVIC_SystemReset() when RESET_CMD is not for this board
TEST_F(power_handler_test, RESET_CMD_NO_RESET_WHEN_NOT_TARGETED) {
    power_handler_init();

    check_board_need_reset_fake.custom_fake =
        [](const can_msg_t *, bool *out) -> w_status_t { *out = false; return W_SUCCESS; };
    get_reset_board_id_fake.return_val = W_SUCCESS;

    can_msg_t dummy = {0};
    ASSERT_NE(reset_cb, nullptr);
    reset_cb(&dummy);

    EXPECT_EQ(NVIC_SystemReset_fake.call_count, 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 3 — GPIO state rules
// ─────────────────────────────────────────────────────────────────────────────

// CHG_MUX_EN must be LOW whenever EN_EXT_5V is HIGH (mutual exclusion)
TEST_F(power_handler_test, CHG_MUX_EN_LOW_WHEN_5V_ENABLED) {
    power_handler_init();
    // Ensure not in low-power and CHG is not the active source (all ADC reads → 0)
    power_handler_set_low_power_mode(false);

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    power_handler_set_5V_external(true);

    // Find the ordering of CHG_MUX_EN=LOW and EN_EXT_5V=HIGH in the call history.
    // CHG_MUX_EN=LOW must appear BEFORE EN_EXT_5V=HIGH.
    int idx_mux_low = -1, idx_en_high = -1;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_CHG_MUX_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW)
            idx_mux_low = i;
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH)
            idx_en_high = i;
    }
    EXPECT_GE(idx_mux_low,  0) << "CHG_MUX_EN never set LOW";
    EXPECT_GE(idx_en_high,  0) << "EN_EXT_5V never set HIGH";
    EXPECT_LT(idx_mux_low, idx_en_high)
        << "CHG_MUX_EN must be cleared BEFORE EN_EXT_5V is asserted";
}

// Shall refuse to enable external 5V when low power mode is active
TEST_F(power_handler_test, NO_5V_EXTERNAL_IN_LOW_POWER_MODE) {
    power_handler_init();
    power_handler_set_low_power_mode(true);

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = power_handler_set_5V_external(true);

    EXPECT_EQ(result, W_FAILURE);

    // EN_EXT_5V must NOT be driven HIGH
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        EXPECT_FALSE(gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
                     gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH)
            << "EN_EXT_5V should not be set HIGH in low power mode";
    }
}

// Shall refuse to enable external 5V when CHG is the active power source
TEST_F(power_handler_test, NO_5V_EXTERNAL_WHEN_CHG_ACTIVE) {
    power_handler_init();
    power_handler_set_low_power_mode(false);

    // Make VSENS_CHG the highest ADC reading → CHG is active source
    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, uint32_t *out) -> w_status_t {
            *out = (ch == VSENS_CHG) ? 1200u : 0u; // CHG dominates
            return W_SUCCESS;
        };

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = power_handler_set_5V_external(true);

    EXPECT_EQ(result, W_FAILURE);

    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        EXPECT_FALSE(gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
                     gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH)
            << "EN_EXT_5V must not be set HIGH when CHG is the active source";
    }
}

// Entering low power mode shall disable LiPo (PWR_EN LOW)
TEST_F(power_handler_test, LOW_POWER_MODE_DISABLES_LIPO) {
    power_handler_init();

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    power_handler_set_low_power_mode(true);

    bool pwr_low = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            pwr_low = true;
        }
    }
    EXPECT_TRUE(pwr_low) << "PWR_EN must be driven LOW when low power mode is enabled";
}

// Entering low power mode shall also disable external 5V (EN_EXT_5V LOW)
TEST_F(power_handler_test, LOW_POWER_MODE_DISABLES_5V_EXTERNAL) {
    power_handler_init();

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    power_handler_set_low_power_mode(true);

    bool en_low = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_EN_EXT_5V &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            en_low = true;
        }
    }
    EXPECT_TRUE(en_low) << "EN_EXT_5V must be driven LOW when entering low power mode";
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 4 — Status reports / health checks
// ─────────────────────────────────────────────────────────────────────────────

// power_handler_get_status() shall return 0 when all sensors are within thresholds
TEST_F(power_handler_test, NO_FAULT_WHEN_ALL_NOMINAL) {
    power_handler_init();

    // All ADC reads → 0 → POWER_INPUT_NONE branch, no threshold checks fire
    // gpio_read defaults to GPIO_LEVEL_HIGH (no fault pins asserted)
    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    uint32_t faults = power_handler_get_status();
    EXPECT_EQ(faults, 0u);
}

// power_handler_get_status() shall set FAULT_5V_OUTPUT when PG_EXT_5V pin is LOW
TEST_F(power_handler_test, FAULT_5V_OUTPUT_WHEN_PG_LOW) {
    power_handler_init();

    gpio_read_fake.custom_fake =
        [](gpio_pin_t pin, gpio_level_t *out, uint32_t) -> w_status_t {
            *out = (pin == GPIO_PIN_PG_EXT_5V) ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status();
    EXPECT_TRUE(faults & FAULT_5V_OUTPUT);
}

// power_handler_get_status() shall set FAULT_5V_CURR when 5V current exceeds I5V_MAX
TEST_F(power_handler_test, FAULT_5V_CURR_ON_OVERCURRENT) {
    power_handler_init();

    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, uint32_t *out) -> w_status_t {
            *out = (ch == ISENS_5V) ? 5000u : 0u; // 5000 mA > I5V_MAX (4000)
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status();
    EXPECT_TRUE(faults & FAULT_5V_CURR);
}

// power_handler_get_status() shall set FAULT_CHG_VOLT when CHG voltage is below VCHG_MIN
TEST_F(power_handler_test, FAULT_CHG_VOLT_BELOW_MIN) {
    power_handler_init();

    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    // Make CHG the dominant input but set its voltage below VCHG_MIN (9)
    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, uint32_t *out) -> w_status_t {
            if (ch == VSENS_CHG) { *out = 1000u; return W_SUCCESS; } // dominates others
            if (ch == VSENS_CHG) { *out = 5u;    return W_SUCCESS; } // below min
            *out = 0u;
            return W_SUCCESS;
        };
    // Simpler: use a single custom fake where VSENS_CHG dominates AND is below threshold.
    // Achieved by returning a value that is highest (so get_active_input picks CHG) but
    // that same value is also < VCHG_MIN. Use raw ADC counts: 5 < 9.
    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, uint32_t *out) -> w_status_t {
            *out = (ch == VSENS_CHG) ? 5u : 0u; // CHG is highest AND below VCHG_MIN
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status();
    EXPECT_TRUE(faults & FAULT_CHG_VOLT);
}

// power_handler_get_status() shall set FAULT_RKT_VOLT when RKT voltage exceeds VRKT_MAX
TEST_F(power_handler_test, FAULT_RKT_VOLT_ABOVE_MAX) {
    power_handler_init();

    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, uint32_t *out) -> w_status_t {
            // RKT dominates and is above VRKT_MAX (12.8). Using raw ADC units here.
            *out = (ch == VSENS_RKT) ? 1000u : 0u;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status();
    EXPECT_TRUE(faults & FAULT_RKT_VOLT);
}

// power_handler_get_status() shall set FAULT_BAT1_CURR when BAT1 current exceeds IBAT_MAX
TEST_F(power_handler_test, FAULT_BAT1_CURR_ON_OVERCURRENT) {
    power_handler_init();

    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, uint32_t *out) -> w_status_t {
            if (ch == VSENS_BAT1) { *out = 2400u;  return W_SUCCESS; } // BAT is active
            if (ch == ISENS_BAT1) { *out = 10000u; return W_SUCCESS; } // > IBAT_MAX (8000)
            *out = 0u;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status();
    EXPECT_TRUE(faults & FAULT_BAT1_CURR);
}

// power_handler_get_status() shall NOT transmit a fault CAN message when there are no faults
TEST_F(power_handler_test, NO_FAULT_MSG_WHEN_ALL_NOMINAL) {
    power_handler_init();

    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;
    // All ADC reads → 0 → no thresholds exceeded

    RESET_FAKE(build_general_board_status_msg);

    power_handler_get_status();

    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 0u)
        << "build_general_board_status_msg must NOT be called when there are no faults";
}