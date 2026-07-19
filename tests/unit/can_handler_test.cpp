#include "fff.h"
#include <gtest/gtest.h>

extern "C" {

#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"
#include "stm32h7/stm32h7_can.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include <stdint.h>

// bool can_init_stm(FDCAN_HandleTypeDef *handle,  can_receive_callback receive_callback)
FAKE_VALUE_FUNC(bool, can_init_stm, FDCAN_HandleTypeDef *, can_receive_callback)

FAKE_VALUE_FUNC(bool, can_send, const can_msg_t *)

// bool check_board_need_reset(const can_msg_t *msg)
FAKE_VALUE_FUNC(w_status_t, check_board_need_reset, const can_msg_t *, bool *)

// uint16_t get_message_type(const can_msg_t *msg)
FAKE_VALUE_FUNC(uint16_t, get_message_type, const can_msg_t *)

// w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t)
}

static w_status_t test_actuator_callback(const can_msg_t *msg) {
    (void)msg;
    return W_SUCCESS;
}

class CanHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(can_init_stm);
        RESET_FAKE(can_send);
        RESET_FAKE(check_board_need_reset);
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xQueueSend);
        RESET_FAKE(gpio_write);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(CanHandlerTest, InitSucceeds) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    can_init_stm_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(xQueueCreate_fake.call_count, 2);
    EXPECT_EQ(can_init_stm_fake.call_count, 1);
}

TEST_F(CanHandlerTest, InitFails) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    can_init_stm_fake.return_val = false; // Failed to initialize canlib
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(CanHandlerTest, InitFails2) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)0x1234;
    can_init_stm_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = NULL; // pass invalid FDCAN handle

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(CanHandlerTest, InitFails3) {
    // Arrange
    xQueueCreate_fake.return_val = NULL; // Failed to create queue
    can_init_stm_fake.return_val = true;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)0x1234;

    // Act
    w_status_t status = can_handler_init(hfdcan);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
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

TEST_F(CanHandlerTest, ActCmdRegisterCallbackFailsWithNullCallback) {
    // Arrange
    can_actuator_id_t actuator = (can_actuator_id_t)0;

    // Act
    w_status_t status = can_handler_act_cmd_register_callback(actuator, NULL);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(CanHandlerTest, ActCmdRegisterCallbackFailsWithInvalidActuatorId) {
    // Arrange
    can_actuator_id_t invalid_actuator = ACTUATOR_ENUM_MAX;

    // Act
    w_status_t status =
        can_handler_act_cmd_register_callback(invalid_actuator, test_actuator_callback);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(CanHandlerTest, ActCmdRegisterCallbackSucceedsWithValidActuatorId) {
    // Arrange
    can_actuator_id_t actuator = (can_actuator_id_t)0;

    // Act
    w_status_t status = can_handler_act_cmd_register_callback(actuator, test_actuator_callback);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

