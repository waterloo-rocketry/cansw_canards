#include "fff.h"
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>

DEFINE_FFF_GLOBALS;

extern "C" {

#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/can_handler/can_telemetry_scaling.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "stm32h7/stm32h7_can.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include "third_party/canlib/message_types.h"
#include "utils/mock_log.hpp"

typedef void (*can_rx_callback_t)(const can_msg_t *message);
FAKE_VALUE_FUNC(bool, stm32h7_can_init, FDCAN_HandleTypeDef *, can_rx_callback_t)
FAKE_VALUE_FUNC(bool, stm32h7_can_send, const can_msg_t *)

FAKE_VALUE_FUNC(w_status_t, check_board_need_reset, const can_msg_t *, bool *)

// w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t)

FAKE_VOID_FUNC(NVIC_SystemReset);
}

// A no-op callback used to exercise the RX dispatch path.
static w_status_t test_dummy_callback(const can_msg_t *msg) {
    (void)msg;
    return W_SUCCESS;
}

class CanHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(stm32h7_can_init);
        RESET_FAKE(stm32h7_can_send);
        RESET_FAKE(check_board_need_reset);
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xQueueSend);
        RESET_FAKE(xQueueSendFromISR);
        RESET_FAKE(gpio_write);
        RESET_FAKE(log_text);
        RESET_FAKE(NVIC_SystemReset);
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
    EXPECT_EQ(xQueueCreate_fake.call_count, 2); // one rx queue, one tx queue
    EXPECT_EQ(stm32h7_can_init_fake.call_count, 1);
}

TEST_F(CanHandlerTest, InitFailsWhenCanlibInitFails) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    stm32h7_can_init_fake.return_val = false; // Failed to initialize canlib
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(CanHandlerTest, InitRejectsNullHandle) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    stm32h7_can_init_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = NULL; // pass invalid FDCAN handle

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(CanHandlerTest, InitFailsWhenQueueCreateFails) {
    // Arrange
    xQueueCreate_fake.return_val = NULL; // Failed to create queue
    stm32h7_can_init_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(CanHandlerTest, EncodeSentinelSignedInt16) {
    // SCALE_MTI_ACCEL is a signed TYPE_INT16 field.
    constexpr int16_t kMax = std::numeric_limits<int16_t>::max();
    int16_t out = 0;

    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_POS_INF);

    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, -INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_NEG_INF);
}

TEST_F(CanHandlerTest, EncodeSentinelSignedInt32) {
    // SCALE_NAV_VELOCITY is a signed TYPE_INT32 field.
    constexpr int32_t kMax = std::numeric_limits<int32_t>::max();
    int32_t out = 0;

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_VELOCITY, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_POS_INF);

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_VELOCITY, -INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - static_cast<int32_t>(SENTINEL_NEG_INF));
}

TEST_F(CanHandlerTest, EncodeSentinelUnsignedUint16) {
    // SCALE_NAV_ALTITUDE is an unsigned TYPE_UINT16 field.
    constexpr uint16_t kMax = std::numeric_limits<uint16_t>::max();
    uint16_t out = 0;

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_ALTITUDE, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_POS_INF);

    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_ALTITUDE, -INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_NEG_INF);
}

TEST_F(CanHandlerTest, EncodeSentinelUnsignedUint32) {
    // SCALE_MTI_PRESSURE is an unsigned TYPE_UINT32 field.
    constexpr uint32_t kMax = std::numeric_limits<uint32_t>::max();
    uint32_t out = 0;

    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_PRESSURE, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_POS_INF);

    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_PRESSURE, -INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_NEG_INF);
}

TEST_F(CanHandlerTest, SentinelIgnoresScaleFactor) {
    // Inf is encoded directly to the reserved code; the scale factor (100 for
    // SCALE_MTI_ACCEL) must not be applied to it.
    constexpr int16_t kMax = std::numeric_limits<int16_t>::max();
    int16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, INFINITY, &out), W_SUCCESS);
    EXPECT_EQ(out, kMax - SENTINEL_POS_INF);
}

TEST_F(CanHandlerTest, EncodeFloatNaNIsSentinelValue) {
    // NaN is signalled to the caller as an error rather than an in-band code.
    int16_t sout = 0;
    uint16_t uout = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, NAN, &sout), W_SUCCESS); // signed field
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_ALTITUDE, NAN, &uout), W_SUCCESS); // unsigned
}

TEST_F(CanHandlerTest, EncodeFloatAppliesScaleFactor) {
    // SCALE_MTI_ACCEL has scale 100: 1.5 -> 150.
    int16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, 1.5f, &out), W_SUCCESS);
    EXPECT_EQ(out, 150);
}

TEST_F(CanHandlerTest, EncodeFloatEncodesNegativeIntoSignedField) {
    // Signed field must accept negative values.
    int16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, -1.5f, &out), W_SUCCESS);
    EXPECT_EQ(out, -150);
}

TEST_F(CanHandlerTest, EncodeFloatTruncatesTowardZero) {
    // The cast to the integer type truncates (does not round).
    int16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, 1.999f, &out), W_SUCCESS);
    EXPECT_EQ(out, 199); // 1.999 * 100 = 199.9 -> 199
}

TEST_F(CanHandlerTest, EncodeFloatOverflowRejected) {
    // 400 * 100 = 40000 exceeds INT16_MAX (32767).
    int16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, 400.0f, &out), W_OVERFLOW);
}

TEST_F(CanHandlerTest, EncodeFloatNegativeIntoUnsignedRejected) {
    // A negative reading cannot be stored in an unsigned field.
    uint16_t out = 0;
    EXPECT_EQ(can_encode_scaled_float(SCALE_NAV_ALTITUDE, -1.0f, &out), W_OVERFLOW);
}

TEST_F(CanHandlerTest, EncodeIntAppliesScaleFactor) {
    // SCALE_NAV_ALTITUDE is TYPE_UINT16, scale 1.
    uint16_t out = 0;
    EXPECT_EQ(can_encode_scaled_int(SCALE_NAV_ALTITUDE, 500, &out), W_SUCCESS);
    EXPECT_EQ(out, 500);
}

TEST_F(CanHandlerTest, EncodeIntUnsignedNegativeRejected) {
    uint16_t out = 0;
    EXPECT_EQ(can_encode_scaled_int(SCALE_NAV_ALTITUDE, -1, &out), W_OVERFLOW);
}

TEST_F(CanHandlerTest, EncodeIntOverflowRejected) {
    // 70000 exceeds UINT16_MAX (65535).
    uint16_t out = 0;
    EXPECT_EQ(can_encode_scaled_int(SCALE_NAV_ALTITUDE, 70000, &out), W_OVERFLOW);
}

TEST_F(CanHandlerTest, EncodeIntInt32Field) {
    // SCALE_NAV_VELOCITY is TYPE_INT32 (was TYPE_INT24), scale 1000: 5 -> 5000.
    // Guards against regressing the int24->int32 width fix.
    int32_t out = 0;
    EXPECT_EQ(can_encode_scaled_int(SCALE_NAV_VELOCITY, 5, &out), W_SUCCESS);
    EXPECT_EQ(out, 5000);
}

TEST_F(CanHandlerTest, EncodeRejectsBadParams) {
    int16_t out = 0;
    // Out-of-range sensor enum.
    EXPECT_EQ(can_encode_scaled_float(SCALE_COUNT, 1.0f, &out), W_INVALID_PARAM);
    EXPECT_EQ(can_encode_scaled_int(SCALE_COUNT, 1, &out), W_INVALID_PARAM);
    // NULL output pointer.
    EXPECT_EQ(can_encode_scaled_float(SCALE_MTI_ACCEL, 1.0f, nullptr), W_INVALID_PARAM);
    EXPECT_EQ(can_encode_scaled_int(SCALE_MTI_ACCEL, 1, nullptr), W_INVALID_PARAM);
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

TEST_F(CanHandlerTest, CanTransmitFailsWhenQueueFull) {
    // Arrange
    can_msg_t msg = {0};
    xQueueSend_fake.return_val = pdFAIL;

    // Act
    w_status_t status = can_handler_transmit(&msg);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}


TEST_F(CanHandlerTest, RxDropsUnregisteredMessageType) {
    // Arrange: ensure MSG_RESET_CMD has no handler, then deliver one.
    can_handler_register_callback(MSG_RESET_CMD, NULL);
    can_msg_t msg = {0};
    msg.sid = (can_sid_t)MSG_RESET_CMD << 20; // get_message_type() extracts bits 20-26 of sid

    // Act
    can_handler_rx_message(&msg);

    // Assert: dropped before ever reaching the queue.
    EXPECT_EQ(xQueueSendFromISR_fake.call_count, 0);
}

TEST_F(CanHandlerTest, RxEnqueuesRegisteredMessageType) {
    // Arrange: register a handler for MSG_RESET_CMD.
    can_handler_register_callback(MSG_RESET_CMD, test_dummy_callback);
    xQueueSendFromISR_fake.return_val = pdPASS;
    can_msg_t msg = {0};
    msg.sid = (can_sid_t)MSG_RESET_CMD << 20; // get_message_type() extracts bits 20-26 of sid

    // Act
    can_handler_rx_message(&msg);

    // Assert: message forwarded to the RX queue exactly once.
    EXPECT_EQ(xQueueSendFromISR_fake.call_count, 1);

    // Cleanup so this registration doesn't leak into other tests.
    can_handler_register_callback(MSG_RESET_CMD, NULL);
}
