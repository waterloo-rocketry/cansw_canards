#ifndef FATFS_MOCK_H
#define FATFS_MOCK_H

#include "fff.h"
#include <stdint.h>

// Needed for HAL_StatusTypeDef and the HAL_SD_* types used below.
// Safe in either include order: stm32h7xx_hal.h defines those types before it includes this file.
#include "stm32h7xx_hal.h"

// Mock SD HAL types
typedef enum {
	HAL_SD_CARD_READY = 0,
	HAL_SD_CARD_TRANSFER = 1,
	HAL_SD_CARD_ERROR = 2
	// etc...
} HAL_SD_CardStateTypeDef;

// SDMMC bus speed modes (values from stm32h7xx_ll_sdmmc.h)
#define SDMMC_SPEED_MODE_AUTO ((uint32_t)0x00000000U)
#define SDMMC_SPEED_MODE_DEFAULT ((uint32_t)0x00000001U)
#define SDMMC_SPEED_MODE_HIGH ((uint32_t)0x00000002U)

// SDMMC bus widths (values from stm32h7xx_ll_sdmmc.h)
#define SDMMC_BUS_WIDE_1B ((uint32_t)0x00000000U)
#define SDMMC_BUS_WIDE_4B ((uint32_t)0x00004000U)
#define SDMMC_BUS_WIDE_8B ((uint32_t)0x00008000U)

// Same fields as the real SDMMC_InitTypeDef
typedef struct {
	uint32_t ClockEdge;
	uint32_t ClockPowerSave;
	uint32_t BusWide;
	uint32_t HardwareFlowControl;
	uint32_t ClockDiv;
} SD_InitTypeDef;

typedef struct {
	uint32_t dummy; // Dummy member for mock struct
	SD_InitTypeDef Init;
	uint32_t ErrorCode;
	HAL_SD_CardInfoTypeDef SdCard;
} SD_HandleTypeDef;

// Declare the external SD handle that the real code uses
extern SD_HandleTypeDef hsd1;

// Type definitions for FATFS
typedef unsigned char BYTE;
typedef unsigned int UINT;
typedef unsigned long long FSIZE_t;
typedef char TCHAR;
typedef int FRESULT;

// Mock structures
typedef struct {
	uint32_t dummy; // Dummy member for mock struct
} FIL;

typedef struct {
	uint32_t dummy; // Dummy member for mock struct
} FATFS;

// Constants
#define FA_READ 0x01
#define FA_WRITE 0x02
#define FA_CREATE_NEW 0x04
#define FA_OPEN_EXISTING 0x00

#define FR_OK 0
#define FR_DISK_ERR 1
#define FR_NO_FILE 4
#define FR_EXIST 8

// Function declarations for mocked functions
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_open, FIL *, const TCHAR *, BYTE);
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_close, FIL *);
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_read, FIL *, void *, UINT, UINT *);
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_write, FIL *, const void *, UINT, UINT *);
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_lseek, FIL *, FSIZE_t);
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_unlink, const TCHAR *);
DECLARE_FAKE_VALUE_FUNC(FRESULT, f_mount, FATFS *, const TCHAR *, BYTE);
DECLARE_FAKE_VALUE_FUNC(FSIZE_t, f_size, FIL *);

// HAL SD mock function declarations (definitions are in stm32h7xx_hal.c / fatfs.c)
DECLARE_FAKE_VALUE_FUNC(HAL_SD_CardStateTypeDef, HAL_SD_GetCardState, SD_HandleTypeDef *);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_ReadBlocks_DMA, SD_HandleTypeDef *, uint8_t *,
                        uint32_t, uint32_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_WriteBlocks_DMA, SD_HandleTypeDef *,
                        const uint8_t *, uint32_t, uint32_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_Abort, SD_HandleTypeDef *);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_GetCardInfo, SD_HandleTypeDef *,
                        HAL_SD_CardInfoTypeDef *);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_GetCardStatus, SD_HandleTypeDef *,
                        HAL_SD_CardStatusTypeDef *);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_ConfigSpeedBusOperation, SD_HandleTypeDef *,
                        uint32_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_SD_ConfigWideBusOperation, SD_HandleTypeDef *,
                        uint32_t);

#endif // FATFS_MOCK_H
