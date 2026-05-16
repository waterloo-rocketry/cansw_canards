#ifndef MOCK_I2C_HAL_H
#define MOCK_I2C_HAL_H

#include "stm32h7xx_hal.h"
#include "fff.h"
#include <stdint.h>

// Callback ID type
typedef enum
{
  HAL_I2C_MASTER_TX_COMPLETE_CB_ID = 0x00U,
  HAL_I2C_MASTER_RX_COMPLETE_CB_ID = 0x01U,
  HAL_I2C_SLAVE_TX_COMPLETE_CB_ID = 0x02U,
  HAL_I2C_SLAVE_RX_COMPLETE_CB_ID = 0x03U,
  HAL_I2C_LISTEN_COMPLETE_CB_ID = 0x04U,
  HAL_I2C_MEM_TX_COMPLETE_CB_ID = 0x05U,
  HAL_I2C_MEM_RX_COMPLETE_CB_ID = 0x06U,
  HAL_I2C_ERROR_CB_ID = 0x07U,
  HAL_I2C_ABORT_CB_ID = 0x08U,
  HAL_I2C_MSPINIT_CB_ID = 0x09U,
  HAL_I2C_MSPDEINIT_CB_ID = 0x0AU
} HAL_I2C_CallbackIDTypeDef;

// Forward declare the handle type for the callback typedef
struct __I2C_HandleTypeDef;
typedef void (*pI2C_CallbackTypeDef)(struct __I2C_HandleTypeDef *hi2c);

// I2C handle type
typedef struct __I2C_HandleTypeDef
{
  uint8_t *pBuffPtr;
  uint16_t XferSize;
  uint16_t XferCount;
  uint32_t XferOptions;
  uint32_t PreviousState;
  uint32_t ErrorCode;
  uint32_t AddrEventCount;
  uint32_t Devaddress;
  uint32_t Memaddress;
  pI2C_CallbackTypeDef MasterTxCpltCallback;
  pI2C_CallbackTypeDef MasterRxCpltCallback;
  pI2C_CallbackTypeDef SlaveTxCpltCallback;
  pI2C_CallbackTypeDef SlaveRxCpltCallback;
  pI2C_CallbackTypeDef ListenCpltCallback;
  pI2C_CallbackTypeDef MemTxCpltCallback;
  pI2C_CallbackTypeDef MemRxCpltCallback;
  pI2C_CallbackTypeDef ErrorCallback;
  pI2C_CallbackTypeDef AbortCpltCallback;
  void (*AddrCallback)(struct __I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
  pI2C_CallbackTypeDef MspInitCallback;
  pI2C_CallbackTypeDef MspDeInitCallback;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1

// Error codes
#define HAL_I2C_ERROR_NONE 0x00000000U
#define HAL_I2C_ERROR_BERR 0x00000001U
#define HAL_I2C_ERROR_ARLO 0x00000002U
#define HAL_I2C_ERROR_AF 0x00000004U
#define HAL_I2C_ERROR_OVR 0x00000008U
#define HAL_I2C_ERROR_DMA 0x00000010U
#define HAL_I2C_ERROR_TIMEOUT 0x00000020U
#define HAL_I2C_ERROR_SIZE 0x00000040U
#define HAL_I2C_ERROR_DMA_PARAM 0x00000080U
#define HAL_I2C_ERROR_INVALID_CALLBACK 0x00000100U
#define HAL_I2C_ERROR_INVALID_PARAM 0x00000200U

// Mock function declarations - exact match with HAL signatures
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_RegisterCallback, I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef, pI2C_CallbackTypeDef);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Master_Abort_IT, I2C_HandleTypeDef *, uint16_t);
DECLARE_FAKE_VALUE_FUNC(uint32_t, HAL_I2C_GetError, I2C_HandleTypeDef *);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Init, I2C_HandleTypeDef *);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_DeInit, I2C_HandleTypeDef *);

#endif // MOCK_I2C_HAL_H
