#include "hal_fdcan_mock.h"

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_Start, FDCAN_HandleTypeDef);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_Stop, FDCAN_HandleTypeDef);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_AddMessageToTxFifoQ, FDCAN_HandleTypeDef *,
					   FDCAN_TxHeaderTypeDef *, const uint8_t *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_ConfigFilter, FDCAN_HandleTypeDef,
					   FDCAN_FilterTypeDef *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_ActivateNotification, FDCAN_HandleTypeDef,
					   uint32_t, uint32_t);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_GetRxMessage, FDCAN_HandleTypeDef, uint32_t,
					   FDCAN_RxHeaderTypeDef *, const uint8_t *);
