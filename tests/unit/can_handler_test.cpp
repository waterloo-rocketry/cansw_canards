#include "fff.h"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

extern "C" {

#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "stm32h7/stm32h7_can.h"
#include "stm32h7xx_hal.h"
#include "task.h"

FAKE_VALUE_FUNC(bool, stm32h7_can_init, FDCAN_HandleTypeDef *, can_receive_callback)
FAKE_VALUE_FUNC(bool, stm32h7_can_send, const can_msg_t *)

// canlib reset-command helpers
FAKE_VALUE_FUNC(w_status_t, get_reset_board_id, const can_msg_t *, uint8_t *, uint8_t *)
FAKE_VALUE_FUNC(w_status_t, check_board_need_reset, const can_msg_t *, bool *)

// uint16_t get_message_type(const can_msg_t *msg)
FAKE_VALUE_FUNC(uint16_t, get_message_type, const can_msg_t *)

// w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t)

// logger
FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *, ...)
}

class CanHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(stm32h7_can_init);
        RESET_FAKE(stm32h7_can_send);
        RESET_FAKE(check_board_need_reset);
        RESET_FAKE(get_reset_board_id);
        RESET_FAKE(get_message_type);
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xQueueSend);
        RESET_FAKE(gpio_write);
        RESET_FAKE(log_text);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(CanHandlerTest, InitSucceeds) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    stm32h7_can_init_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(xQueueCreate_fake.call_count, 2);
    EXPECT_EQ(stm32h7_can_init_fake.call_count, 1);
}

TEST_F(CanHandlerTest, InitFails) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    stm32h7_can_init_fake.return_val = false; // Failed to initialize canlib
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(CanHandlerTest, InitFails2) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    stm32h7_can_init_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = NULL; // pass invalid FDCAN handle

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(CanHandlerTest, InitFails3) {
    // Arrange
    xQueueCreate_fake.return_val = NULL; // Failed to create queue
    stm32h7_can_init_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// --- Non-finite (NaN / +/-Inf) and out-of-range inverted sentinel mapping ------
// Contract (can_telemetry_scaling.h): +Inf/above-range -> type_min,
// -Inf/below-range -> type_max, NaN -> dropped (W_MATH_ERROR, out untouched).

// SCALE_NAV_Q -> TYPE_INT16 (max 32767), SCALE_NAV_RX -> TYPE_UINT16 (max 65535).

TEST_F(CanHandlerTest, EncodeNonFiniteSignedSaturates) {
    int16_t out = 0;

    // NaN is dropped and out is left untouched.
    out = 123;
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_Q, NAN, &out), W_MATH_ERROR);
    EXPECT_EQ(out, 123);

    // +Inf -> type_min, -Inf -> type_max (inverted).
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_Q, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, (int16_t)INT16_MIN);

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_Q, -INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, (int16_t)INT16_MAX);
}

TEST_F(CanHandlerTest, EncodeNonFiniteUnsignedSaturates) {
    uint16_t out = 0;

    out = 123;
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_RX, NAN, &out), W_MATH_ERROR);
    EXPECT_EQ(out, 123);

    // +Inf -> type_min (0), -Inf -> type_max (inverted).
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_RX, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, (uint16_t)0);

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_RX, -INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, (uint16_t)UINT16_MAX);
}

TEST_F(CanHandlerTest, EncodeFiniteOutOfRangeMapsInverted) {
    // A huge finite value above range -> type_min, below range -> type_max.
    uint16_t u_out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_RX, 1e9f, &u_out), W_SUCCESS);
    EXPECT_EQ(u_out, (uint16_t)0);

    int16_t s_out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_Q, 1e9f, &s_out), W_SUCCESS);
    EXPECT_EQ(s_out, (int16_t)INT16_MIN);

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_Q, -1e9f, &s_out), W_SUCCESS);
    EXPECT_EQ(s_out, (int16_t)INT16_MAX);
}

TEST_F(CanHandlerTest, EncodeRejectsBadParams) {
    int16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_COUNT, 1.0f, &out), W_INVALID_PARAM);
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_Q, 1.0f, nullptr), W_INVALID_PARAM);
}

TEST_F(CanHandlerTest, CanTransmitSucceeds) {
    // Arrange
    can_msg_t msg = {0};
    xQueueSend_fake.return_val = pdPASS;

    // Act
    w_status_t status = can_handler_transmit(&msg);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(xQueueSend_fake.call_count, 1);
}

