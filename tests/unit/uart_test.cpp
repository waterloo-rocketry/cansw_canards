#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/logger/log.h"
#include "drivers/uart/uart.h"
#include "hal_uart_mock.h"
#include "queue.h"
#include "rocketlib.h"
#include "stm32h7xx_hal.h"
#include "utils/mock_log.hpp"
}

DEFINE_FFF_GLOBALS;

// Helper functions for queue receive simulation
static BaseType_t QueueReceiveCustomFake(QueueHandle_t queue, void *buffer, TickType_t wait) {
    uart_msg_t **msg_ptr = (uart_msg_t **)buffer;
    static uart_msg_t msg;
    static uint8_t test_data[] = "test";

    msg.data = test_data;
    msg.len = 5;
    msg.busy = true;
    *msg_ptr = &msg;

    return pdTRUE;
}

static BaseType_t QueueReceiveOverflowFake(QueueHandle_t queue, void *buffer, TickType_t wait) {
    uart_msg_t **msg_ptr = (uart_msg_t **)buffer;
    static uart_msg_t msg;
    static uint8_t test_data[UART_MAX_LEN + 10];
    memset(test_data, 'A', sizeof(test_data));

    msg.data = test_data;
    msg.len = UART_MAX_LEN + 10;
    msg.busy = true;
    *msg_ptr = &msg;

    return pdTRUE;
}

// Test fixture for UART tests
class UartTest : public ::testing::Test {
protected:
    UART_HandleTypeDef huart;
    const uint32_t timeout = 100;
    const uart_channel_t TEST_CHANNEL = UART_DEBUG_SERIAL; // Use actual enum value

    void SetUp() override {
        // Reset all fakes before each test
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xQueueReceive);
        RESET_FAKE(HAL_UARTEx_ReceiveToIdle_IT);
        RESET_FAKE(HAL_UARTEx_ReceiveToIdle_DMA);
        RESET_FAKE(HAL_UART_RegisterCallback);
        RESET_FAKE(HAL_UART_RegisterRxEventCallback);
        RESET_FAKE(HAL_UART_Transmit_IT);
        RESET_FAKE(HAL_UART_Transmit_DMA);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreCreateBinary);

        UART_MOCK_RESET();

        // Initialize UART handle
        memset(&huart, 0, sizeof(huart));
        huart.Instance = (USART_TypeDef *)0x12345678; // Any non-null value

        // initialize common return values in uart_init
        xQueueCreate_fake.return_val = (QueueHandle_t)1; // Return valid queue handle
        HAL_UARTEx_ReceiveToIdle_DMA_fake.return_val = HAL_OK;
        HAL_UART_RegisterCallback_fake.return_val = HAL_OK;
        HAL_UART_RegisterRxEventCallback_fake.return_val = HAL_OK;
        xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
        xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)1;
    }
};

// Test initialization with valid parameters
TEST_F(UartTest, InitSuccess) {
    // Test initialization
    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);

    // Verify results
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(1, xSemaphoreCreateMutex_fake.call_count); // mutex
    EXPECT_EQ(1, xSemaphoreCreateBinary_fake.call_count); // transnmit complete semaphore
    EXPECT_EQ(1, xQueueCreate_fake.call_count);
    EXPECT_EQ(1, HAL_UARTEx_ReceiveToIdle_DMA_fake.call_count);
    EXPECT_EQ(2, HAL_UART_RegisterCallback_fake.call_count); // Error callback
    EXPECT_EQ(1, HAL_UART_RegisterRxEventCallback_fake.call_count); // RX Event callback
}

// Test initialization with invalid parameters
TEST_F(UartTest, InitInvalidParams) {
    // Test with invalid channel
    w_status_t status = uart_init((uart_channel_t)99, &huart, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);

    // Test with null UART handle
    status = uart_init(TEST_CHANNEL, NULL, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
}

// Test initialization failure cases
// TODO: figure out how to test each of the two HAL_UART_RegisterCallback_fake
TEST_F(UartTest, InitFailures) {
    // test semaphore/mutex creation failure
    xSemaphoreCreateMutex_fake.return_val = nullptr;
    xSemaphoreCreateBinary_fake.return_val = nullptr;
    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);
    EXPECT_EQ(W_FAILURE, status);

    // Test queue creation failure
    xQueueCreate_fake.return_val = nullptr;
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)1;
    status = uart_init(TEST_CHANNEL, &huart, timeout);
    EXPECT_EQ(W_FAILURE, status);

    // Test HAL initialization failure
    HAL_UART_RegisterCallback_fake.return_val = HAL_ERROR;
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    status = uart_init(TEST_CHANNEL, &huart, timeout);
    EXPECT_EQ(W_FAILURE, status);

    // Test HAL receive start failure
    HAL_UART_RegisterCallback_fake.return_val = HAL_OK;
    HAL_UARTEx_ReceiveToIdle_DMA_fake.return_val = HAL_ERROR;
    status = uart_init(TEST_CHANNEL, &huart, timeout);
    EXPECT_EQ(W_IO_ERROR, status);

    // Test RX Event callback registration failure
    HAL_UART_RegisterCallback_fake.return_val = HAL_OK;
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    HAL_UART_RegisterRxEventCallback_fake.return_val = HAL_ERROR;
    status = uart_init(TEST_CHANNEL, &huart, timeout);
    EXPECT_EQ(W_FAILURE, status);
}

// Test uart_write with valid message
TEST_F(UartTest, WriteSuccess) {
    uint8_t buffer[] = "ABCDEFGH";
    uint16_t length = sizeof(buffer) - 1; // Fix length initialization

    // Test initialization
    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);

    xSemaphoreTake_fake.return_val = pdTRUE;
    xSemaphoreGive_fake.return_val = pdTRUE;
    HAL_UART_Transmit_DMA_fake.return_val = HAL_OK; // Now uses DMA

    status = uart_write(TEST_CHANNEL, buffer, length, timeout);
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(2, xSemaphoreTake_fake.call_count);
    EXPECT_EQ(1, xSemaphoreGive_fake.call_count);
    EXPECT_EQ(1, HAL_UART_Transmit_DMA_fake.call_count); // Verify DMA function called
}

// TODO: add more tests for uart_write

TEST_F(UartTest, WriteInvalidParams) {
    uint8_t buffer[] = "ABCDEFGH";
    uint16_t length = sizeof(buffer) - 1; // Fix length initialization

    // Test initialization
    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);

    xSemaphoreTake_fake.return_val = pdTRUE;
    xSemaphoreGive_fake.return_val = pdTRUE;
    HAL_UART_Transmit_DMA_fake.return_val = HAL_OK; // Now uses DMA
    // Valid
    status = uart_write(TEST_CHANNEL, buffer, length, timeout);
    EXPECT_EQ(W_SUCCESS, status);
    // Invalid channel
    status = uart_write((uart_channel_t)99, buffer, length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
    // No matching s_uart_handles for channel
    status = uart_write((uart_channel_t)2, buffer, length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
    // Invalid buffer
    status = uart_write(TEST_CHANNEL, NULL, length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
    // Invalid length
    status = uart_write(TEST_CHANNEL, buffer, 0, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
}

// semaphore function timesout, return pdFalse
TEST_F(UartTest, WriteTimeout) {
    uint8_t buffer[] = "ABCDEFGH";
    uint16_t length = sizeof(buffer) - 1; // Fix length initialization

    // Test initialization
    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);

    xSemaphoreTake_fake.return_val = pdFALSE; // fails to acquire write mutex
    status = uart_write(TEST_CHANNEL, buffer, length, timeout);
    EXPECT_EQ(W_IO_TIMEOUT, status);

    xSemaphoreTake_fake.return_val = pdTRUE;
    HAL_UART_Transmit_DMA_fake.return_val = HAL_OK; // Now uses DMA
    xSemaphoreGive_fake.return_val = pdFALSE;
    status = uart_write(TEST_CHANNEL, buffer, length, timeout); // fails to return write mutex
    EXPECT_EQ(W_IO_TIMEOUT, status);
}

// Hal function return bad status
TEST_F(UartTest, WriteHal) {
    uint8_t buffer[] = "ABCDEFGH";
    uint16_t length = sizeof(buffer) - 1; // Fix length initialization

    // Test initialization
    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);

    xSemaphoreTake_fake.call_count = 0; // reset
    xSemaphoreGive_fake.call_count = 0; // reset

    xSemaphoreTake_fake.return_val = pdTRUE;
    xSemaphoreGive_fake.return_val = pdTRUE;
    HAL_UART_Transmit_DMA_fake.return_val = HAL_ERROR; // HAL DMA fails to write
    status = uart_write(TEST_CHANNEL, buffer, length, timeout);
    EXPECT_EQ(W_IO_ERROR, status);
    EXPECT_EQ(1, xSemaphoreTake_fake.call_count);
    EXPECT_EQ(1, xSemaphoreGive_fake.call_count);

    xSemaphoreTake_fake.call_count = 0; // reset
    xSemaphoreGive_fake.call_count = 0; // reset

    HAL_UART_Transmit_DMA_fake.return_val = HAL_BUSY; // HAL DMA times out
    status = uart_write(TEST_CHANNEL, buffer, length, timeout);
    EXPECT_EQ(W_IO_TIMEOUT, status);
    EXPECT_EQ(1, xSemaphoreTake_fake.call_count);
    EXPECT_EQ(1, xSemaphoreGive_fake.call_count);
}

// Test uart_read with valid message
TEST_F(UartTest, ReadSuccess) {
    uint8_t buffer[UART_MAX_LEN];
    uint16_t length;

    // Setup xQueueReceive to return a message
    xQueueReceive_fake.custom_fake = QueueReceiveCustomFake;

    w_status_t status = uart_read(TEST_CHANNEL, buffer, &length, timeout);

    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(5, length);
    EXPECT_EQ(1, xQueueReceive_fake.call_count);
    EXPECT_EQ(0, memcmp(buffer, "test", 5));
}

// Test uart_read with timeout
TEST_F(UartTest, ReadTimeout) {
    uint8_t buffer[UART_MAX_LEN];
    uint16_t length;

    // Setup xQueueReceive to simulate timeout
    xQueueReceive_fake.return_val = pdFALSE;

    w_status_t status = uart_read(TEST_CHANNEL, buffer, &length, timeout);

    EXPECT_EQ(W_IO_TIMEOUT, status);
    EXPECT_EQ(0, length);
    EXPECT_EQ(1, xQueueReceive_fake.call_count);
}

// Test uart_read with invalid parameters
TEST_F(UartTest, ReadInvalidParams) {
    uint8_t buffer[UART_MAX_LEN];
    uint16_t length;

    // Test invalid channel
    w_status_t status = uart_read((uart_channel_t)99, buffer, &length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);

    // Test null buffer
    status = uart_read(TEST_CHANNEL, NULL, &length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);

    // Test null length pointer
    status = uart_read(TEST_CHANNEL, buffer, NULL, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
}

// Test message overflow handling
TEST_F(UartTest, MessageOverflowHandling) {
    uint8_t buffer[UART_MAX_LEN];
    uint16_t length;

    // Setup queue to return oversized message using regular function
    xQueueReceive_fake.custom_fake = QueueReceiveOverflowFake;

    w_status_t status = uart_read(TEST_CHANNEL, buffer, &length, timeout);

    // Message should be received but truncated
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(UART_MAX_LEN, length); // Length should be truncated
    EXPECT_EQ(1, xQueueReceive_fake.call_count);
}

// Test circular buffer handling through ISR callback
TEST_F(UartTest, CircularBufferHandling) {
    // Initialize UART first
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)1;
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    HAL_UARTEx_ReceiveToIdle_DMA_fake.return_val = HAL_OK; // Now uses DMA
    HAL_UART_RegisterCallback_fake.return_val = HAL_OK;
    uart_init(TEST_CHANNEL, &huart, timeout);

    // Simulate multiple message receptions
    for (int i = 0; i < UART_NUM_RX_BUFFERS * 2; i++) {
        HAL_UARTEx_RxEventCallback(&huart, 10); // Simulate reception of 10 bytes
        // Each callback should trigger a new reception
        EXPECT_EQ(
            i + 2, HAL_UARTEx_ReceiveToIdle_DMA_fake.call_count
        ); // +1 for init, +1 per callback
    }
}

// Test error callback and error statistics
TEST_F(UartTest, ErrorHandlingAndStats) {
    // Initialize UART first
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)1;
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    HAL_UARTEx_ReceiveToIdle_DMA_fake.return_val = HAL_OK; // Now uses DMA
    HAL_UART_RegisterCallback_fake.return_val = HAL_OK;
    HAL_UART_RegisterRxEventCallback_fake.return_val = HAL_OK;

    w_status_t status = uart_init(TEST_CHANNEL, &huart, timeout);
    EXPECT_EQ(W_SUCCESS, status);

    // Set error code and trigger callback
    huart.ErrorCode = HAL_UART_ERROR_FE; // Frame error
    HAL_UART_ErrorCallback(&huart);

    // Should have attempted to restart reception
    EXPECT_EQ(2, HAL_UARTEx_ReceiveToIdle_DMA_fake.call_count); // 1 for init, 1 for error recovery

    // Test timeout error handling
    uint8_t buffer[UART_MAX_LEN];
    uint16_t length;
    xQueueReceive_fake.return_val = pdFALSE; // Simulate timeout
    status = uart_read(TEST_CHANNEL, buffer, &length, timeout);

    EXPECT_EQ(W_IO_TIMEOUT, status);
    EXPECT_EQ(0, length);
}