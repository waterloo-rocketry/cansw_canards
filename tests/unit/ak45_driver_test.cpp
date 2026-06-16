#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "ak45_driver_test_helpers.h"
#include "application/logger/log.h"
#include "drivers/ak45_driver/ak45_driver.h"
#include "hal_fdcan_mock.h"
#include "queue.h"
#include "task.h"

FDCAN_HandleTypeDef hfdcan1;

DEFINE_FFF_GLOBALS;
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
}

// There is a while loop in ak45_driver initialization which waits for messages to be received.
// Calling this function set the variable it check to true and avoids the actualy time wait.
void vTaskDelay_custom_bypass_while(const TickType_t ticks) {
	HAL_FDCAN_RxFifo1Callback(&hfdcan1, 0);
}

// Causes the timer to fail to test a specific error case in ak_45 driver initialization
void vTaskDelay_custom_fail_timer(const TickType_t ticks) {
	timer_get_ms_fake.return_val = W_FAILURE;
}

// timer_get_ms is called to update the current time, the following function replaces it to merely
// increment the time, and since the time-out time is set to 0 in testing, it autofails.
w_status_t timer_get_ms_custom_exceed_time(uint32_t *time) {
	(*time)++;
	return W_SUCCESS;
}

class AK45DriverTest : public ::testing::Test {
protected:
	void SetUp() override {
		ak45_test_reset();

		RESET_FAKE(HAL_FDCAN_Start);
		RESET_FAKE(HAL_FDCAN_Stop);
		RESET_FAKE(HAL_FDCAN_AddMessageToTxFifoQ);
		RESET_FAKE(HAL_FDCAN_ConfigFilter);
		RESET_FAKE(HAL_FDCAN_ActivateNotification);
		RESET_FAKE(HAL_FDCAN_GetRxMessage);
		RESET_FAKE(xQueueCreate);
		RESET_FAKE(xQueuePeek);
		RESET_FAKE(xQueueOverwriteFromISR);
		RESET_FAKE(vTaskDelay);
		RESET_FAKE(log_text);
		RESET_FAKE(timer_get_ms);

		FFF_RESET_HISTORY();

		// default happy-path return values
		HAL_FDCAN_ConfigFilter_fake.return_val = HAL_OK;
		HAL_FDCAN_Start_fake.return_val = HAL_OK;
		HAL_FDCAN_Stop_fake.return_val = HAL_OK;
		HAL_FDCAN_ActivateNotification_fake.return_val = HAL_OK;
		HAL_FDCAN_AddMessageToTxFifoQ_fake.return_val = HAL_OK;
		HAL_FDCAN_GetRxMessage_fake.return_val = HAL_OK;
		xQueueCreate_fake.return_val = (QueueHandle_t)1;
	}

	void TearDown() override {}
};

TEST_F(AK45DriverTest, InitInvalidParams) {
	// Arrange + Act
	w_status_t status = ak45_driver_init(NULL, 0);

	// Assert
	EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(AK45DriverTest, InitFailsIfQueueCreationFails) {
	// Arrange
	xQueueCreate_fake.return_val = NULL;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, InitFailsIfFilterConfigFails) {
	// Arrange
	HAL_FDCAN_ConfigFilter_fake.return_val = HAL_ERROR;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, InitFailsIfFdcanStartFails) {
	// Arrange
	HAL_FDCAN_Start_fake.return_val = HAL_ERROR;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
	// This failure calls HAL_FD_CAN_Stop
	EXPECT_EQ(HAL_FDCAN_Stop_fake.call_count, 1);
}

TEST_F(AK45DriverTest, InitFailsIfActivateNotificationFails) {
	// Arrange
	HAL_FDCAN_ActivateNotification_fake.return_val = HAL_ERROR;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
	// This failure calls HAL_FD_CAN_Stop
	EXPECT_EQ(HAL_FDCAN_Stop_fake.call_count, 1);
}

TEST_F(AK45DriverTest, InitFailsIfTimerGetMsFails) {
	// Arrange
	timer_get_ms_fake.return_val = W_FAILURE;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
	// This failure calls HAL_FD_CAN_Stop
	EXPECT_EQ(HAL_FDCAN_Stop_fake.call_count, 1);
}

TEST_F(AK45DriverTest, InitFailsIfTimerGetMsFailsInsideWhileLoop) {
	// Arrange
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_fail_timer;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
	// This failure calls HAL_FD_CAN_Stop
	EXPECT_EQ(HAL_FDCAN_Stop_fake.call_count, 1);
}

TEST_F(AK45DriverTest, InitFailsIfWhileLoopTimesOutWithNoCanMsg) {
	// Arrange
	timer_get_ms_fake.custom_fake = timer_get_ms_custom_exceed_time;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
	// This failure calls HAL_FD_CAN_Stop
	EXPECT_EQ(HAL_FDCAN_Stop_fake.call_count, 1);
}

TEST_F(AK45DriverTest, InitFailsIfCANTransmitFails) {
	// Arrange
	HAL_FDCAN_AddMessageToTxFifoQ_fake.return_val = HAL_ERROR;
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
	// This failure increments the number of errors by 1
	EXPECT_EQ(ak45_get_tx_errors(), 1);
}

TEST_F(AK45DriverTest, InitFailsIfDoubleInit) {
	// Arrange
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;

	// Act
	// Attempt to initialize twice; fails on the second attempts.
	ak45_driver_init(&hfdcan1, 0);
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, InitSucceeds) {
	// Arrange
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;
	// Set errors to verify that initialization resets to 0.
	ak45_set_tx_errors(67);
	EXPECT_EQ(ak45_get_tx_errors(), 67);

	// Act
	w_status_t status = ak45_driver_init(&hfdcan1, 0);

	// Assert
	EXPECT_EQ(status, W_SUCCESS);
	EXPECT_EQ(ak45_get_tx_errors(), 0);
}

TEST_F(AK45DriverTest, SendPositionCmdFailsIfNotInit) {
	// Act
	w_status_t status = ak45_send_position_cmd(0);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, SendPositionCmdSucceeds) {
	// Arrange
	// Success requires initialization. Custom fake required for successful intialization.
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;
	ak45_driver_init(&hfdcan1, 0);

	// Act
	w_status_t status = ak45_send_position_cmd(0);

	// Assert
	EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(AK45DriverTest, SendDisableCmdFailsIfNotInit) {
	// Act
	w_status_t status = ak45_send_disable_cmd();

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, SendDisableCmdSucceeds) {
	// Arrange
	// Success requires initialization. Custom fake required for successful intialization.
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;
	ak45_driver_init(&hfdcan1, 0);

	// Act
	w_status_t status = ak45_send_disable_cmd();

	// Assert
	EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(AK45DriverTest, GetLatestFeedbackFailsIfNullFeedback) {
	// Act
	w_status_t status = ak45_get_latest_feedback(NULL);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, GetLatestFeedbackFailsIfNotInit) {
	// Act
	ak45_feedback_t fb;
	w_status_t status = ak45_get_latest_feedback(&fb);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, GetLatestFeedbackFailsIfQueuePeekFails) {
	// Arrange
	// Success requires initialization. Custom fake required for successful intialization.
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;
	ak45_driver_init(&hfdcan1, 0);

	xQueuePeek_fake.return_val = pdFAIL;
	ak45_feedback_t fb;

	// Act
	w_status_t status = ak45_get_latest_feedback(&fb);

	// Assert
	EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AK45DriverTest, GetLatestFeedbackSucceeds) {
	// Arrange
	// Success requires initialization. Custom fake required for successful intialization.
	vTaskDelay_fake.custom_fake = vTaskDelay_custom_bypass_while;
	ak45_driver_init(&hfdcan1, 0);

	ak45_feedback_t fb;
	xQueuePeek_fake.return_val = pdPASS;

	// Act
	w_status_t status = ak45_get_latest_feedback(&fb);

	// Assert
	EXPECT_EQ(status, W_SUCCESS);
}
