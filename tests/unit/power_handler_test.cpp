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
    FAKE_VALUE_FUNC(w_status_t, adc_get_converted_val, adc_channel_t, float*);
    // can handler fakes
    FAKE_VALUE_FUNC(w_status_t, can_handler_act_cmd_register_callback, can_actuator_id_t, can_callback_t);
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
    // system reset fakes
    FAKE_VOID_FUNC(NVIC_SystemReset);
    // logging and timer fakes
    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char*, const char*, ...);
    FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
}

// Captured CAN callbacks registered during power_handler_init()
static w_status_t (*actuator_cb)(const can_msg_t *) = nullptr;
static w_status_t (*reset_cb)(const can_msg_t *)    = nullptr;

// Both actuator ids (5V external, LiPo) are registered against the same callback function,
// so a single captured pointer is sufficient.
static w_status_t act_cmd_register_callback_fake(can_actuator_id_t /*act_type*/, w_status_t (*cb)(const can_msg_t *)) {
    actuator_cb = cb;
    return W_SUCCESS;
}

// Stores the reset callback so tests can invoke it directly
static w_status_t register_callback_fake(can_msg_type_t type, w_status_t (*cb)(const can_msg_t *)) {
    if (type == MSG_RESET_CMD) reset_cb = cb;
    return W_SUCCESS;
}

// Helper: fake adc_get_converted_val that writes a caller-supplied value into *out
static float adc_injected_value = 0;
static w_status_t adc_return_injected(adc_channel_t /*ch*/, float *out) {
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
        RESET_FAKE(can_handler_act_cmd_register_callback);
        RESET_FAKE(can_handler_register_callback);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(get_actuator_id);
        RESET_FAKE(get_cmd_actuator_state);
        RESET_FAKE(get_reset_board_id);
        RESET_FAKE(check_board_need_reset);
        RESET_FAKE(build_actuator_status_msg);
        RESET_FAKE(build_analog_sensor_16bit_msg);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(NVIC_SystemReset);
        FFF_RESET_HISTORY();

        gpio_write_fake.return_val                     = W_SUCCESS;
        gpio_read_fake.return_val                      = W_SUCCESS;
        adc_get_converted_val_fake.return_val          = W_SUCCESS;
        can_handler_act_cmd_register_callback_fake.return_val = W_SUCCESS;
        can_handler_register_callback_fake.return_val  = W_SUCCESS;
        can_handler_transmit_fake.return_val           = W_SUCCESS;

        actuator_cb = nullptr;
        reset_cb    = nullptr;
        adc_injected_value   = 0;
        gpio_injected_level  = GPIO_LEVEL_HIGH;

        // Capture the CAN callbacks registered by power_handler_init()
        can_handler_act_cmd_register_callback_fake.custom_fake = act_cmd_register_callback_fake;
        can_handler_register_callback_fake.custom_fake         = register_callback_fake;
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

// power_handler_get_status() shall set CANARDS_MODULE_E_NOT_INIT before power_handler_init() runs
TEST_F(power_handler_test, NOT_INIT_FAULT_BEFORE_INIT) {
    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    uint32_t faults = power_handler_get_status().error_bitfield;
    EXPECT_TRUE(faults & (1u << CANARDS_MODULE_E_NOT_INIT_OFFSET));
}

// power_handler_init() shall default CHG_MUX_EN LOW
TEST_F(power_handler_test, DEFAULT_CHG_MUX_EN_LOW) {
    power_handler_init();

    bool found = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_CHG_MUX_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "Expected CHG_MUX_EN to be set LOW during init";
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

// power_handler_init() shall default PWR_EN LOW
TEST_F(power_handler_test, DEFAULT_PWR_EN_LOW) {
    power_handler_init();

    bool found = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "Expected PWR_EN to be set LOW during init";
}

// power_handler_init() shall register CAN callbacks for the actuator commands and RESET_CMD
TEST_F(power_handler_test, INIT_REGISTERS_CAN_CALLBACKS) {
    power_handler_init();

    EXPECT_NE(actuator_cb, nullptr) << "Actuator command callback not registered";
    EXPECT_NE(reset_cb,    nullptr) << "MSG_RESET_CMD callback not registered";
    EXPECT_EQ(can_handler_act_cmd_register_callback_fake.call_count, 2u)
        << "Expected callbacks registered for both CANARD_5V_OUTPUT and CANARD_LIPO_ON";
}

// power_handler_init() shall return W_SUCCESS when both callbacks register successfully
TEST_F(power_handler_test, INIT_RETURNS_SUCCESS) {
    EXPECT_EQ(power_handler_init(), W_SUCCESS);
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

// Shall enable LiPo (PWR_EN LOW) in response to CANARD_LIPO_ON ACT_STATE_ON
TEST_F(power_handler_test, EN_LIPO_CMD) {
    power_handler_init();

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = send_actuator_cmd(ACTUATOR_CANARD_LIPO_ON, ACT_STATE_ON);

    EXPECT_EQ(result, W_SUCCESS);

    bool pwr_low = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_LOW) {
            pwr_low = true;
        }
    }
    EXPECT_TRUE(pwr_low) << "PWR_EN should be LOW when LiPo enable command received";
}

// Shall disable LiPo in response to CANARD_LIPO_ON ACT_STATE_OFF
TEST_F(power_handler_test, DIS_LIPO_CMD) {
    power_handler_init();

    RESET_FAKE(gpio_write);
    gpio_write_fake.return_val = W_SUCCESS;

    w_status_t result = send_actuator_cmd(ACTUATOR_CANARD_LIPO_ON, ACT_STATE_OFF);

    EXPECT_EQ(result, W_SUCCESS);

    bool pwr_high = false;
    for (int i = 0; i < (int)gpio_write_fake.call_count; i++) {
        if (gpio_write_fake.arg0_history[i] == GPIO_PIN_PWR_EN &&
            gpio_write_fake.arg1_history[i] == GPIO_LEVEL_HIGH) {
            pwr_high = true;
        }
    }
    EXPECT_TRUE(pwr_high) << "PWR_EN should be HIGH when LiPo disable command received";
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

// power_handler_get_status() shall return 0 when all sensors are within thresholds
TEST_F(power_handler_test, NO_FAULT_WHEN_ALL_NOMINAL) {
    power_handler_init();

    // gpio_read defaults to GPIO_LEVEL_HIGH (no fault pins asserted)
    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;

    // Turn LiPo on so the "low power mode with ext 5V on" fault does not fire
    // (power_handler_init() leaves lipo_state false with external 5V enabled)
    ASSERT_EQ(send_actuator_cmd(ACTUATOR_CANARD_LIPO_ON, ACT_STATE_ON), W_SUCCESS);

    uint32_t faults = power_handler_get_status().error_bitfield;
    EXPECT_EQ(faults, 0u);
}

// power_handler_get_status() shall set CANARDS_MODULE_E_DEVICE_FAULT when PG_EXT_5V pin is LOW
TEST_F(power_handler_test, DEVICE_FAULT_WHEN_PG_LOW) {
    power_handler_init();

    gpio_read_fake.custom_fake =
        [](gpio_pin_t pin, gpio_level_t *out, uint32_t) -> w_status_t {
            *out = (pin == GPIO_PIN_PG_EXT_5V) ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status().error_bitfield;
    EXPECT_TRUE(faults & (1u << CANARDS_MODULE_E_DEVICE_FAULT_OFFSET));
}

// power_handler_get_status() shall set CANARDS_MODULE_E_BAT1_FAULT when BAT_FLT1 pin is LOW
TEST_F(power_handler_test, BAT1_FAULT_WHEN_FLT1_LOW) {
    power_handler_init();

    gpio_read_fake.custom_fake =
        [](gpio_pin_t pin, gpio_level_t *out, uint32_t) -> w_status_t {
            *out = (pin == GPIO_PIN_BAT_FLT1) ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_status().error_bitfield;
    EXPECT_TRUE(faults & (1u << CANARDS_MODULE_E_BAT1_FAULT_OFFSET));
}

// power_handler_get_board_status() shall set E_5V_OVER_CURR when 5V current exceeds I5V_MAX
TEST_F(power_handler_test, BOARD_FAULT_5V_OVERCURRENT) {
    power_handler_init();

    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, float *out) -> w_status_t {
            *out = (ch == ISENS_5V) ? 5000.0f : 0.0f; // 5000 mA > I5V_MAX (4000)
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_board_status();
    EXPECT_TRUE(faults & (1u << E_5V_OVER_CURR_OFFSET));
}

// power_handler_get_board_status() shall set E_12V_OVER_VOLT when rocket rail voltage exceeds VRKT_MAX
TEST_F(power_handler_test, BOARD_FAULT_RKT_VOLT_ABOVE_MAX) {
    power_handler_init();

    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, float *out) -> w_status_t {
            // RKT dominates the active-input check and is above VRKT_MAX (12.8).
            *out = (ch == VSENS_RKT) ? 1000.0f : 0.0f;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_board_status();
    EXPECT_TRUE(faults & (1u << E_12V_OVER_VOLT_OFFSET));
}

// power_handler_get_board_status() shall set E_BATT_OVER_CURR when BAT1 current exceeds IBAT_MAX
TEST_F(power_handler_test, BOARD_FAULT_BAT1_OVERCURRENT) {
    power_handler_init();

    adc_get_converted_val_fake.custom_fake =
        [](adc_channel_t ch, float *out) -> w_status_t {
            if (ch == VSENS_BAT1) { *out = 2400.0f;  return W_SUCCESS; } // BAT is active
            if (ch == ISENS_BAT1) { *out = 10000.0f; return W_SUCCESS; } // > IBAT_MAX (8000)
            *out = 0.0f;
            return W_SUCCESS;
        };

    uint32_t faults = power_handler_get_board_status();
    EXPECT_TRUE(faults & (1u << E_BATT_OVER_CURR_OFFSET));
}

// power_handler_get_board_status() shall return 0 when all sensors are within thresholds
TEST_F(power_handler_test, BOARD_NO_FAULT_WHEN_ALL_NOMINAL) {
    power_handler_init();

    gpio_read_fake.custom_fake = gpio_read_injected;
    gpio_injected_level = GPIO_LEVEL_HIGH;
    // All ADC reads → 0 → POWER_INPUT_NONE branch, no threshold checks fire

    uint32_t faults = power_handler_get_board_status();
    EXPECT_EQ(faults, 0u);
}
