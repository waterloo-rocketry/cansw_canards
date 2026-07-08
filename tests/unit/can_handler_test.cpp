// #include "fff.h"
// #include <cmath>
// #include <gtest/gtest.h>
// #include <limits>

// extern "C" {

// #include "FreeRTOS.h"
// #include "application/can_handler/can_handler.h"
// #include "drivers/gpio/gpio.h"
// #include "queue.h"
// #include "stm32h7/stm32h7_can.h"
// #include "stm32h7xx_hal.h"
// #include "task.h"

// FAKE_VALUE_FUNC(bool, stm32h7_can_init, FDCAN_HandleTypeDef *, can_receive_callback)
// FAKE_VALUE_FUNC(bool, stm32h7_can_send, const can_msg_t *)

// // canlib reset-command helpers
// FAKE_VALUE_FUNC(w_status_t, get_reset_board_id, const can_msg_t *, uint8_t *, uint8_t *)
// FAKE_VALUE_FUNC(w_status_t, check_board_need_reset, const can_msg_t *, bool *)

// // uint16_t get_message_type(const can_msg_t *msg)
// FAKE_VALUE_FUNC(uint16_t, get_message_type, const can_msg_t *)

// // w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
// FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t)

// // logger
// FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *, ...)
// }

// class CanHandlerTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//         RESET_FAKE(stm32h7_can_init);
//         RESET_FAKE(stm32h7_can_send);
//         RESET_FAKE(check_board_need_reset);
//         RESET_FAKE(get_reset_board_id);
//         RESET_FAKE(get_message_type);
//         RESET_FAKE(xQueueCreate);
//         RESET_FAKE(xQueueSend);
//         RESET_FAKE(gpio_write);
//         RESET_FAKE(log_text);
//         FFF_RESET_HISTORY();
//     }

//     void TearDown() override {}
// };

// TEST_F(CanHandlerTest, InitSucceeds) {
//     // Arrange
//     xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
//     stm32h7_can_init_fake.return_val = true;
//     FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

//     // Act
//     w_status_t status = can_handler_init(hfdcan);

//     // Assert
//     EXPECT_EQ(status, W_SUCCESS);
//     EXPECT_EQ(xQueueCreate_fake.call_count, 2);
//     EXPECT_EQ(stm32h7_can_init_fake.call_count, 1);
// }

// TEST_F(CanHandlerTest, InitFails) {
//     // Arrange
//     xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
//     stm32h7_can_init_fake.return_val = false; // Failed to initialize canlib
//     FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

//     // Act
//     w_status_t status = can_handler_init(hfdcan);

//     // Assert
//     EXPECT_EQ(status, W_FAILURE);
// }

// TEST_F(CanHandlerTest, InitFails2) {
//     // Arrange
//     xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
//     stm32h7_can_init_fake.return_val = true;
//     FDCAN_HandleTypeDef *hfdcan = NULL; // pass invalid FDCAN handle

//     // Act
//     w_status_t status = can_handler_init(hfdcan);

//     // Assert
//     EXPECT_EQ(status, W_INVALID_PARAM);
// }

// TEST_F(CanHandlerTest, InitFails3) {
//     // Arrange
//     xQueueCreate_fake.return_val = NULL; // Failed to create queue
//     stm32h7_can_init_fake.return_val = true;
//     FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

//     // Act
//     w_status_t status = can_handler_init(hfdcan);

//     // Assert
//     EXPECT_EQ(status, W_FAILURE);
// }
// .

// TEST_F(CanHandlerTest, EncodeNonFiniteSignedSentinels) {
//     int16_t out = 0;

// }

// TEST_F(CanHandlerTest, EncodeNonFiniteUnsignedSentinels) {
//     uint16_t out = 0;

// }

// TEST_F(CanHandlerTest, EncodeRejectsBadParams) {
//     int16_t out = 0;
//     EXPECT_EQ(can_encode_scaled_float(SCALE_COUNT, 1.0f, &out), W_INVALID_PARAM);
// }

// TEST_F(CanHandlerTest, CanTransmitSucceeds) {
//     // Arrange
//     can_msg_t msg = {0};
//     xQueueSend_fake.return_val = pdPASS;

//     // Act
//     w_status_t status = can_handler_transmit(&msg);

//     // Assert
//     EXPECT_EQ(status, W_SUCCESS);
//     EXPECT_EQ(xQueueSend_fake.call_count, 1);
// }

