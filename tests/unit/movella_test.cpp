#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "drivers/movella/movella.h"
#include "semphr.h"
#include "third_party/rocketlib/include/common.h"
}

class MovellaTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        FFF_RESET_HISTORY();

        xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1;
        xSemaphoreTake_fake.return_val = pdTRUE;
        xSemaphoreGive_fake.return_val = pdTRUE;

        movella_init();
    }
};

TEST_F(MovellaTest, InitSuccess) {
    EXPECT_EQ(W_SUCCESS, movella_init());
}

TEST_F(MovellaTest, GetDataNullPointer) {
    EXPECT_EQ(W_INVALID_PARAM, movella_get_data(NULL, 100));
}

TEST_F(MovellaTest, GetDataMutexTimeout) {
    xSemaphoreTake_fake.return_val = pdFALSE;
    movella_data_t data;
    EXPECT_EQ(W_FAILURE, movella_get_data(&data, 100));
}

TEST_F(MovellaTest, GetDataSuccess) {
    movella_data_t data;
    EXPECT_EQ(W_SUCCESS, movella_get_data(&data, 100));
}

TEST_F(MovellaTest, GetDataAccSaturation) {
    movella_data_t data;
    w_status_t result = movella_get_data(&data, 100);
    EXPECT_EQ(W_SUCCESS, result);
    EXPECT_LE(data.acc.x, 10.0 * 9.81);
    EXPECT_GE(data.acc.x, -10.0 * 9.81);
}

TEST_F(MovellaTest, GetDataPresSaturation) {
    movella_data_t data;
    w_status_t result = movella_get_data(&data, 100);
    EXPECT_EQ(W_SUCCESS, result);
    EXPECT_LE(data.pres, 110000.0f);
    EXPECT_GE(data.pres, 6000.0f);
}