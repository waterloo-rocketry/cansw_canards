#ifndef HAL_FDCAN_MOCK_H
#define HAL_FDCAN_MOCK_H

#include "fff.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

// Added function declaration for use in ak45_driver_test.cpp
extern "C" {
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
}

// From stm32h7xx_hal_fdcan.h
#define FDCAN_EXTENDED_ID ((uint32_t)0x40000000U)
#define FDCAN_DATA_FRAME ((uint32_t)0x00000000U)
#define FDCAN_ESI_ACTIVE ((uint32_t)0x00000000U)
#define FDCAN_BRS_OFF ((uint32_t)0x00000000U)
#define FDCAN_CLASSIC_CAN ((uint32_t)0x00000000U)
#define FDCAN_NO_TX_EVENTS ((uint32_t)0x00000000U)
#define FDCAN_FILTER_DUAL ((uint32_t)0x00000001U)
#define FDCAN_FILTER_TO_RXFIFO1 ((uint32_t)0x00000002U)
#define FDCAN_RX_FIFO1 ((uint32_t)0x00000041U)
#define FDCAN_DLC_BYTES_1 ((uint32_t)0x00000001U)
#define FDCAN_DLC_BYTES_4 ((uint32_t)0x00000004U)
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE ((uint32_t)0x00000010U)

typedef void *FDCAN_HandleTypeDef; // Mock as a void*

typedef struct __FDCAN_TxHeaderTypeDef {
	uint32_t Identifier;
	uint32_t IdType;
	uint32_t TxFrameType;
	uint32_t DataLength;
	uint32_t ErrorStateIndicator;
	uint32_t BitRateSwitch;
	uint32_t FDFormat;
	uint32_t TxEventFifoControl;
	uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct __FDCAN_RxHeaderTypeDef {
	uint32_t Identifier;
	uint32_t IdType;
	uint32_t RxFrameType;
	uint32_t DataLength;
	uint32_t ErrorStateIndicator;
	uint32_t BitRateSwitch;
	uint32_t FDFormat;
	uint32_t RxTimestamp;
	uint32_t FilterIndex;
	uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

typedef struct __FDCAN_FilterTypeDef {
	uint32_t IdType;
	uint32_t FilterIndex;
	uint32_t FilterType;
	uint32_t FilterConfig;
	uint32_t FilterID1;
	uint32_t FilterID2;
	uint32_t RxBufferIndex;
	uint32_t IsCalibrationMsg;
} FDCAN_FilterTypeDef;

// Declare mock functions for FDCAN HAL functions here
// HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_Start, FDCAN_HandleTypeDef);

// HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_Stop, FDCAN_HandleTypeDef);

// HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, const
// FDCAN_TxHeaderTypeDef *pTxHeader, const uint8_t *pTxData);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_AddMessageToTxFifoQ, FDCAN_HandleTypeDef *,
						FDCAN_TxHeaderTypeDef *, const uint8_t *);

// HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, const FDCAN_FilterTypeDef
// *sFilterConfig)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_ConfigFilter, FDCAN_HandleTypeDef,
						FDCAN_FilterTypeDef *);

// HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, uint32_t ActiveITs,
// uint32_t BufferIndexes)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_ActivateNotification, FDCAN_HandleTypeDef,
						uint32_t, uint32_t);

// HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation,
// FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_FDCAN_GetRxMessage, FDCAN_HandleTypeDef, uint32_t,
						FDCAN_RxHeaderTypeDef *, const uint8_t *);

#endif // HAL_FDCAN_MOCK_H
