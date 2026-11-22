#include "drivers/raw_logger/raw_logger.h"
#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include "FreeRTOS.h"
#include "drivers/sd_card/sd_card.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

extern raw_sd_card_health_t raw_sd_card_health;
extern SemaphoreHandle_t raw_sd_mutex;
}

DEFINE_FFF_GLOBALS;

class RAW_SDTest : public ::testing::Test {
protected:
    SD_HandleTypeDef hsd; // Add SD handle
    uint8_t buffer[BLOCKSIZE]

    void SetUp() override {
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        RESET_FAKE(HAL_SD_GetCardState);
        FFF_RESET_HISTORY();

        // Default success returns
        xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
        xSemaphoreTake_fake.return_val = pdTRUE;
        HAL_SD_GetCardState_fake.return_val = HAL_SD_CARD_TRANSFER;

        // Initialize SD handle
        memset(&hsd, 0, sizeof(SD_HandleTypeDef));
        hsd.SdCard.LogBlockNbr = 1000;
    }

    void TearDown() override {
        // Reset the SD card state after each test
        raw_sd_card_health.is_init = false;
        raw_sd_mutex = NULL;
    }
};

TEST_F(RAW_SDTest, InitSuccess) {
    EXPECT_EQ(raw_logger_SD_init(&hsd), W_SUCCESS);
}

TEST_F(RAW_SDTest, InitFailMutex) {
    EXPECT_EQ(raw_logger_SD_init(NULL), W_FAILURE);
}






// EVERYTHING BELOW IS FROM SD_CARD_TEST
// JUST FOR REFERENCE!!!

// TEST_F(RAW_SDTest, ReadSuccess) {
//     char buffer[100];
//     uint32_t bytes_read;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_OK;
//     f_read_fake.return_val = FR_OK;
//     f_close_fake.return_val = FR_OK;

//     EXPECT_EQ(sd_card_file_read("test.txt", buffer, 100, &bytes_read), W_SUCCESS);
// }

// TEST_F(RAW_SDTest, ReadFailNullParams) {
//     char buffer[100];
//     uint32_t bytes_read;

//     // Initialize SD card first
//     sd_card_init();

//     EXPECT_EQ(sd_card_file_read(NULL, buffer, 100, &bytes_read), W_INVALID_PARAM);
//     EXPECT_EQ(sd_card_file_read("test.txt", NULL, 100, &bytes_read), W_INVALID_PARAM);
//     EXPECT_EQ(sd_card_file_read("test.txt", buffer, 100, NULL), W_INVALID_PARAM);
// }

// TEST_F(RAW_SDTest, ReadFailOpenFile) {
//     char buffer[100];
//     uint32_t bytes_read;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_DISK_ERR;

//     EXPECT_EQ(sd_card_file_read("test.txt", buffer, 100, &bytes_read), W_FAILURE);
// }

// TEST_F(RAW_SDTest, ReadFailReadOperation) {
//     RESET_FAKE(f_close);
//     char buffer[100];
//     uint32_t bytes_read;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_OK;
//     f_read_fake.return_val = FR_DISK_ERR;
//     f_close_fake.return_val = FR_OK;

//     EXPECT_EQ(sd_card_file_read("test.txt", buffer, 100, &bytes_read), W_FAILURE);
//     // Verify file was closed even though read failed
//     EXPECT_EQ(f_close_fake.call_count, 1);
// }

// TEST_F(RAW_SDTest, WriteSuccess) {
//     const char buffer[] = "test data";
//     uint32_t bytes_written;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_OK;
//     f_write_fake.return_val = FR_OK;
//     f_lseek_fake.return_val = FR_OK;

//     EXPECT_EQ(
//         sd_card_file_write("test.txt", buffer, strlen(buffer), true, &bytes_written), W_SUCCESS
//     );
// }

// TEST_F(RAW_SDTest, WriteFailOpenFile) {
//     const char buffer[] = "test data";
//     uint32_t bytes_written;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_NO_FILE;

//     EXPECT_EQ(
//         sd_card_file_write("test.txt", buffer, strlen(buffer), true, &bytes_written), W_FAILURE
//     );
// }

// TEST_F(RAW_SDTest, WriteFailLseek) {
//     RESET_FAKE(f_close);
//     const char buffer[] = "test data";
//     uint32_t bytes_written;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_OK;
//     f_lseek_fake.return_val = FR_DISK_ERR;
//     f_close_fake.return_val = FR_OK;

//     EXPECT_EQ(
//         sd_card_file_write("test.txt", buffer, strlen(buffer), true, &bytes_written), W_FAILURE
//     );
//     // Verify file was closed after lseek failure
//     EXPECT_EQ(f_close_fake.call_count, 1);
// }

// TEST_F(RAW_SDTest, WriteFailWriteOperation) {
//     RESET_FAKE(f_close);
//     const char buffer[] = "test data";
//     uint32_t bytes_written;

//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_OK;
//     f_lseek_fake.return_val = FR_OK;
//     f_write_fake.return_val = FR_DISK_ERR;
//     f_close_fake.return_val = FR_OK;

//     EXPECT_EQ(
//         sd_card_file_write("test.txt", buffer, strlen(buffer), true, &bytes_written), W_FAILURE
//     );
//     // Verify file was closed after write failure
//     EXPECT_EQ(f_close_fake.call_count, 1);
// }

// TEST_F(RAW_SDTest, CreateFileSuccess) {
//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_OK;
//     f_close_fake.return_val = FR_OK;

//     EXPECT_EQ(sd_card_file_create("test.txt"), W_SUCCESS);
// }

// TEST_F(RAW_SDTest, CreateFileFailExists) {
//     // Initialize SD card first
//     sd_card_init();

//     f_open_fake.return_val = FR_DISK_ERR;

//     EXPECT_EQ(sd_card_file_create("test.txt"), W_FAILURE);
// }

// TEST_F(RAW_SDTest, IsWritableSuccess) {
//     // Initialize SD card first
//     sd_card_init();

//     HAL_SD_GetCardState_fake.return_val = HAL_SD_CARD_TRANSFER;
//     EXPECT_EQ(sd_card_is_writable(&hsd), W_SUCCESS);
// }

// TEST_F(RAW_SDTest, IsWritableFailure) {
//     // Initialize SD card first
//     sd_card_init();

//     HAL_SD_GetCardState_fake.return_val = HAL_SD_CARD_ERROR;
//     EXPECT_EQ(sd_card_is_writable(&hsd), W_FAILURE);
// }

// TEST_F(RAW_SDTest, MutexTakeFailure) {
//     char buffer[10];
//     uint32_t bytes_read;

//     // Initialize SD card first
//     sd_card_init();

//     // Now set up mutex take to fail
//     xSemaphoreTake_fake.return_val = pdFALSE;

//     EXPECT_EQ(sd_card_file_read("test.txt", buffer, 10, &bytes_read), W_FAILURE);
//     EXPECT_EQ(sd_card_file_create("test.txt"), W_FAILURE);
// }

// TEST_F(RAW_SDTest, WriteFailMutexTake) {
//     RESET_FAKE(f_open);
//     const char buffer[] = "test data";
//     uint32_t bytes_written;

//     // Initialize SD card first
//     sd_card_init();

//     // Now set up mutex take to fail
//     xSemaphoreTake_fake.return_val = pdFALSE;

//     EXPECT_EQ(
//         sd_card_file_write("test.txt", buffer, strlen(buffer), true, &bytes_written), W_FAILURE
//     );
//     // Verify f_open was not called since mutex take failed
//     EXPECT_EQ(f_open_fake.call_count, 0);
// }
