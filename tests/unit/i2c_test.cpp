#include "fff.h"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/i2c/i2c.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "utils/mock_log.hpp"

extern void i2c_transfer_complete_callback(I2C_HandleTypeDef *hi2c);
extern i2c_error_data i2c_error_stats[I2C_BUS_COUNT];
I2C_HandleTypeDef hi2c2;
}

/**
 * Test fixture for I2C module tests.
 * Assumes that all mocks and fakes are already defined in the included header files.
 */
class I2CTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset I2C module state for tests using the test-only reset API.
        i2c_reset_all();
        // Reset all FreeRTOS semaphore mocks
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreCreateBinary);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        RESET_FAKE(xSemaphoreGiveFromISR);
        RESET_FAKE(vSemaphoreDelete);

        // Reset all HAL I2C mocks
        RESET_FAKE(HAL_I2C_Mem_Read_IT);
        RESET_FAKE(HAL_I2C_Mem_Write_IT);
        RESET_FAKE(HAL_I2C_RegisterCallback);
        RESET_FAKE(HAL_I2C_Master_Abort_IT);

        // Reset FFF history
        FFF_RESET_HISTORY();

        // Set up common return values for semaphores
        xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
        xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
        xSemaphoreTake_fake.return_val = pdTRUE;

        // Reset hi2c2 error code
        hi2c2.ErrorCode = HAL_I2C_ERROR_NONE;
    }
};

/**
 * Test that initialization succeeds with valid parameters.
 */
TEST_F(I2CTest, InitSuccess) {
    w_status_t status = i2c_init(I2C_BUS_2, &hi2c2, 100); // Updated to I2C2
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_I2C_RegisterCallback_fake.call_count, 3);
}

/**
 * Test that reads fail when the bus is not initialized.
 */
TEST_F(I2CTest, ReadFailsWhenUninitialized) {
    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_2, 0x50, 0x10, data, sizeof(data)); // Updated to I2C2
    EXPECT_EQ(status, W_FAILURE);
}

/**
 * Test address shifting on read operation.
 */
TEST_F(I2CTest, AddressShiftingOnRead) {
    // Initialize the bus
    ASSERT_EQ(i2c_init(I2C_BUS_2, &hi2c2, 100), W_SUCCESS);

    // Set up for successful read
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;
    xSemaphoreTake_fake.return_val = pdTRUE;

    uint8_t data[4];
    uint8_t device_addr = 0x50; // 7-bit address
    w_status_t status = i2c_read_reg(I2C_BUS_2, device_addr, 0x10, data, sizeof(data));

    // Check that HAL was called with shifted address
    ASSERT_GT(HAL_I2C_Mem_Read_IT_fake.call_count, 0);
    uint16_t shifted_addr = (uint16_t)HAL_I2C_Mem_Read_IT_fake.arg1_val;
    EXPECT_EQ(shifted_addr, (device_addr << 1) & 0xFE);
}

/**
 * Test address shifting on write operation.
 */
TEST_F(I2CTest, AddressShiftingOnWrite) {
    // Initialize the bus
    ASSERT_EQ(i2c_init(I2C_BUS_2, &hi2c2, 100), W_SUCCESS);

    // Set up for successful write
    HAL_I2C_Mem_Write_IT_fake.return_val = HAL_OK;
    xSemaphoreTake_fake.return_val = pdTRUE;

    uint8_t data[4] = {0};
    uint8_t device_addr = 0x50; // 7-bit address
    w_status_t status = i2c_write_reg(I2C_BUS_2, device_addr, 0x10, data, sizeof(data));

    // Check that HAL was called with shifted address
    ASSERT_GT(HAL_I2C_Mem_Write_IT_fake.call_count, 0);
    uint16_t shifted_addr = (uint16_t)HAL_I2C_Mem_Write_IT_fake.arg1_val;
    EXPECT_EQ(shifted_addr, (device_addr << 1) & 0xFE);
}

/**
 * Test a successful read operation.
 */
TEST_F(I2CTest, ReadSuccess) {
    // Initialize the bus.
    ASSERT_EQ(i2c_init(I2C_BUS_2, &hi2c2, 100), W_SUCCESS);

    // Set up the fake for a successful I2C read.
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;

    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_2, 0x50, 0x10, data, sizeof(data));

    // Simulate the completion callback with no error.
    hi2c2.ErrorCode = HAL_I2C_ERROR_NONE;
    i2c_transfer_complete_callback(&hi2c2);

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_I2C_Mem_Read_IT_fake.call_count, 1);
}

/**
 * Test read operation when a device NACK occurs.
 * TODO: This test currently uses std::thread which isn't appropriate for unit testing.
 * Need to refactor to properly mock FreeRTOS behavior instead of using real threads.
 */
/*
TEST_F(I2CTest, ReadHandlesDeviceNack)
{
    // Initialize the bus.
    ASSERT_EQ(i2c_init(I2C_BUS_1, &hi2c1, 100), W_SUCCESS);

    // Set up the fake to return HAL_OK so that the transfer is initiated.
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;

    // Save any preexisting custom_fake (if any) so we can restore it later.
    auto orig_xSemaphoreTake = xSemaphoreTake_fake.custom_fake;

    // Override xSemaphoreTake for the transfer semaphore only.
    // Assume that the transfer semaphore is created with value 0x5678.
    // For calls with a nonzero timeout on the transfer semaphore, simulate a delay.
    xSemaphoreTake_fake.custom_fake = [](SemaphoreHandle_t sem, TickType_t ticks) -> BaseType_t
    {
        // If this is the transfer semaphore and a blocking call is expected...
        if (sem == (SemaphoreHandle_t)0x5678 && ticks != 0)
        {
            // Simulate blocking by sleeping for 20ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            return pdTRUE;
        }
        // For all other cases (including the zero-time "clear" call and mutex), return pdTRUE
immediately. return pdTRUE;
    };

    uint8_t data[4];
    w_status_t status = W_SUCCESS;

    // Start the read operation in a separate thread.
    std::thread readThread([&]()
                           { status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data)); });

    // Wait briefly to ensure the read operation has initiated and is "blocked" waiting on the
semaphore. std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Simulate a device NACK error by setting the error code and calling the callback.
    hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
    i2c_transfer_complete_callback(&hi2c1);

    // Wait for the read thread to finish.
    readThread.join();

    // Restore the original xSemaphoreTake fake.
    xSemaphoreTake_fake.custom_fake = orig_xSemaphoreTake;

    // Verify that the read operation returns an I/O error and increments the NACK counter.
    EXPECT_EQ(status, W_IO_ERROR);
    EXPECT_EQ(i2c_error_stats[I2C_BUS_1].nacks, 1);
}
*/

/**
 * Test a successful write operation.
 */
TEST_F(I2CTest, WriteSuccess) {
    // Initialize the bus.
    ASSERT_EQ(i2c_init(I2C_BUS_2, &hi2c2, 100), W_SUCCESS);

    // Set up the fake to return HAL_OK for write.
    xSemaphoreTake_fake.return_val = pdTRUE;
    HAL_I2C_Mem_Write_IT_fake.return_val = HAL_OK;

    uint8_t data[4] = {0xAA};
    w_status_t status = i2c_write_reg(I2C_BUS_2, 0x50, 0x10, data, sizeof(data));

    // Simulate a successful transfer completion.
    hi2c2.ErrorCode = HAL_I2C_ERROR_NONE;
    i2c_transfer_complete_callback(&hi2c2);

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_I2C_Mem_Write_IT_fake.call_count, 1);
}

/**
 * Test behavior when the bus mutex cannot be acquired.
 */
TEST_F(I2CTest, HandlesLockedMutex) {
    // Initialize the bus.
    ASSERT_EQ(i2c_init(I2C_BUS_2, &hi2c2, 100), W_SUCCESS);

    // Simulate a timeout by having xSemaphoreTake for the mutex return pdFALSE.
    xSemaphoreTake_fake.return_val = pdFALSE;

    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_2, 0x50, 0x10, data, sizeof(data));

    EXPECT_EQ(status, W_IO_TIMEOUT);
}
