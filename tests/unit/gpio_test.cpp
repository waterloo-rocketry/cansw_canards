#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "utils/mock_log.hpp"
}

DEFINE_FFF_GLOBALS;

// Test fixture for GPIO tests
class GpioTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(HAL_GPIO_WritePin);
        RESET_FAKE(HAL_GPIO_ReadPin);
        RESET_FAKE(HAL_GPIO_TogglePin);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test gpio_init
TEST_F(GpioTest, InitCreatesMutexes) {
    // Arrange
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;

    // Act
    w_status_t status = gpio_init();

    // Assert
    EXPECT_EQ(status, W_SUCCESS); // Ensure initialization succeeds
    EXPECT_EQ(
        xSemaphoreCreateMutex_fake.call_count, GPIO_PIN_COUNT
    ); // Check mutex created for each pin
}

TEST_F(GpioTest, InitFailsIfMutexCreationFails) {
    // Arrange
    xSemaphoreCreateMutex_fake.return_val = NULL;

    // Act
    w_status_t status = gpio_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE); // Ensure initialization fails if mutex creation fails
    EXPECT_EQ(
        xSemaphoreCreateMutex_fake.call_count, GPIO_PIN_COUNT
    ); // Check attempted mutex creation for each pin
}

// Test gpio_read
TEST_F(GpioTest, ReadReturnsSuccess) {
    // Arrange
    gpio_level_t level;
    xSemaphoreTake_fake.return_val = pdTRUE;
    HAL_GPIO_ReadPin_fake.return_val = GPIO_PIN_SET;

    // Act
    w_status_t status = gpio_read(GPIO_PIN_RED_LED, &level, 100);

    // Assert
    EXPECT_EQ(status, W_SUCCESS); // Ensure read succeeds
    EXPECT_EQ(level, GPIO_LEVEL_HIGH); // Verify level is set to high
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore taken once
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1); // Verify semaphore released once
    EXPECT_EQ(HAL_GPIO_ReadPin_fake.call_count, 1); // Ensure HAL read is called once
    EXPECT_EQ(HAL_GPIO_ReadPin_fake.arg0_val, GPIOC); // Check correct port used
    EXPECT_EQ(HAL_GPIO_ReadPin_fake.arg1_val, GPIO_PIN_13); // Check correct pin used
}

TEST_F(GpioTest, ReadFailsIfSemaphoreNotTaken) {
    // Arrange
    gpio_level_t level;
    xSemaphoreTake_fake.return_val = pdFALSE;

    // Act
    w_status_t status = gpio_read(GPIO_PIN_RED_LED, &level, 100);

    // Assert
    EXPECT_EQ(status, W_IO_TIMEOUT); // Ensure read fails with timeout
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore take attempted
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(HAL_GPIO_ReadPin_fake.call_count, 0); // Ensure HAL read not called
}

TEST_F(GpioTest, ReadInvalidPinReturnsError) {
    // Arrange
    gpio_level_t level;

    // Act
    w_status_t status = gpio_read((gpio_pin_t)GPIO_PIN_COUNT, &level, 100);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM); // Ensure invalid pin returns error
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 0); // Verify no semaphore attempt
    EXPECT_EQ(HAL_GPIO_ReadPin_fake.call_count, 0); // Ensure HAL read not called
}

// Test gpio_write
TEST_F(GpioTest, WriteSetsPinHigh) {
    // Arrange
    xSemaphoreTake_fake.return_val = pdTRUE;

    // Act
    w_status_t status = gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 100);

    // Assert
    EXPECT_EQ(status, W_SUCCESS); // Ensure write succeeds
    EXPECT_EQ(HAL_GPIO_WritePin_fake.call_count, 1); // Ensure HAL write is called once
    EXPECT_EQ(HAL_GPIO_WritePin_fake.arg2_val, GPIO_PIN_SET); // Verify pin set high
    EXPECT_EQ(HAL_GPIO_WritePin_fake.arg1_val, GPIO_PIN_13); // Check correct pin used
    EXPECT_EQ(HAL_GPIO_WritePin_fake.arg0_val, GPIOC); // Check correct pin used
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore taken once
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1); // Verify semaphore released once
}

TEST_F(GpioTest, WriteSetsPinLow) {
    // Arrange
    xSemaphoreTake_fake.return_val = pdTRUE;

    // Act
    w_status_t status = gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_LOW, 100);

    // Assert
    EXPECT_EQ(status, W_SUCCESS); // Ensure write succeeds
    EXPECT_EQ(HAL_GPIO_WritePin_fake.call_count, 1); // Ensure HAL write is called once
    EXPECT_EQ(HAL_GPIO_WritePin_fake.arg2_val, GPIO_PIN_RESET); // Verify pin set low
    EXPECT_EQ(HAL_GPIO_WritePin_fake.arg1_val, GPIO_PIN_13); // Check correct pin used
    EXPECT_EQ(HAL_GPIO_WritePin_fake.arg0_val, GPIOC); // Check correct pin used
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore taken once
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1); // Verify semaphore released once
}

TEST_F(GpioTest, WriteFailsIfSemaphoreNotTaken) {
    // Arrange
    xSemaphoreTake_fake.return_val = pdFALSE;

    // Act
    w_status_t status = gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 100);

    // Assert
    EXPECT_EQ(status, W_IO_TIMEOUT); // Ensure write fails with timeout
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore take attempted
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(HAL_GPIO_WritePin_fake.call_count, 0); // Ensure HAL write not called
}

TEST_F(GpioTest, WriteInvalidPinReturnsError) {
    // Act
    w_status_t status = gpio_write((gpio_pin_t)GPIO_PIN_COUNT, GPIO_LEVEL_HIGH, 100);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM); // Ensure invalid pin returns error
    EXPECT_EQ(HAL_GPIO_WritePin_fake.call_count, 0); // Ensure HAL write not called
}

// Test gpio_toggle
TEST_F(GpioTest, ToggleSuccess) {
    // Arrange
    xSemaphoreTake_fake.return_val = pdTRUE;

    // Act
    w_status_t status = gpio_toggle(GPIO_PIN_RED_LED, 100);

    // Assert
    EXPECT_EQ(status, W_SUCCESS); // Ensure toggle succeeds
    EXPECT_EQ(HAL_GPIO_TogglePin_fake.call_count, 1); // Ensure HAL toggle is called once
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore taken once
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1); // Verify semaphore released once
}

TEST_F(GpioTest, ToggleFailsIfSemaphoreNotTaken) {
    // Arrange
    xSemaphoreTake_fake.return_val = pdFALSE;

    // Act
    w_status_t status = gpio_toggle(GPIO_PIN_RED_LED, 100);

    // Assert
    EXPECT_EQ(status, W_IO_TIMEOUT); // Ensure toggle fails with timeout
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1); // Verify semaphore take attempted
    EXPECT_EQ(xSemaphoreTake_fake.arg1_val, pdMS_TO_TICKS(100)); // Verify timeout is correct
    EXPECT_EQ(HAL_GPIO_TogglePin_fake.call_count, 0); // Ensure HAL toggle not called
}

TEST_F(GpioTest, ToggleInvalidPinReturnsError) {
    // Act
    w_status_t status = gpio_toggle((gpio_pin_t)GPIO_PIN_COUNT, 100);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM); // Ensure invalid pin returns error
    EXPECT_EQ(HAL_GPIO_TogglePin_fake.call_count, 0); // Ensure HAL toggle not called
}
