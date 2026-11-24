#include "stm32h7xx_hal.h"

// Define here
DEFINE_FAKE_VOID_FUNC(HAL_Init);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_ReadBlocks_DMA, SD_HandleTypeDef *, uint8_t *,
                       uint32_t, uint32_t);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_WriteBlocks_DMA, SD_HandleTypeDef *,
                       const uint8_t *, uint32_t, uint32_t);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_Abort, SD_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_GetCardInfo, SD_HandleTypeDef *,
                       HAL_SD_CardInfoTypeDef *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_GetCardStatus, SD_HandleTypeDef *,
                       HAL_SD_CardStatusTypeDef *);
