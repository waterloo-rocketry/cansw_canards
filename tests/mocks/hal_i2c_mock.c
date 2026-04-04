#include "hal_i2c_mock.h"

// Define the mock functions - exact match with declarations
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
    uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
    uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_I2C_RegisterCallback, I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef,
    pI2C_CallbackTypeDef
);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Master_Abort_IT, I2C_HandleTypeDef *, uint16_t);
DEFINE_FAKE_VALUE_FUNC(uint32_t, HAL_I2C_GetError, I2C_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Init, I2C_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_DeInit, I2C_HandleTypeDef *);
