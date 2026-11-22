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
