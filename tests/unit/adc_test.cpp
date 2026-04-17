#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "drivers/adc/adc.h"
#include "hal_adc_mock.h"
#include "semphr.h"

ADC_HandleTypeDef hadc1;
}

class ADCTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(xSemaphoreCreateBinary);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreGive);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(HAL_ADC_RegisterCallback);
        RESET_FAKE(HAL_ADCEx_Calibration_Start);
        RESET_FAKE(HAL_ADC_Start_IT);
        RESET_FAKE(HAL_ADC_Stop_IT);
        RESET_FAKE(HAL_ADC_GetValue);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(ADCTest, InitSucceeds) {
    // Arrange
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x1;
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1;
    xSemaphoreGive_fake.return_val = pdTRUE;
    HAL_ADC_RegisterCallback_fake.return_val = HAL_OK;
    HAL_ADCEx_Calibration_Start_fake.return_val = HAL_OK;

    // Act
    w_status_t status = adc_init(&hadc1);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(ADCTest, GetValueSucceeds) {
    // Arrange
    xSemaphoreTake_fake.return_val = pdTRUE;
    HAL_ADC_Start_IT_fake.return_val = HAL_OK;
    HAL_ADCEx_Calibration_Start_fake.return_val = HAL_OK;
    HAL_ADC_GetValue_fake.return_val = 0x1234;

    // Act
    uint32_t counts;
    w_status_t status = adc_get_raw_counts(PROCESSOR_BOARD_VOLTAGE, &counts, 100);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(counts, 0x1234);
}

