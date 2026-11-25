#include "fff.h"
#include <gtest/gtest.h>
#include <inttypes.h> // included here first so raw_logger.c's copy isn't pulled in inside extern "C"
#include <string.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/raw_logger/raw_logger.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

// Fakes for modules that have no mock file. Must come before the driver is included.
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *,
                       ...);

// For accessing and changing static variables sigh
#include "drivers/raw_logger/raw_logger.c"
}

DEFINE_FFF_GLOBALS;

// Size of the pretend card, in 512-byte blocks (BLOCKSIZE comes from the mock HAL)
#define FAKE_CARD_BLOCKS 1000U

// HAL_SD_GetCardInfo / HAL_SD_GetCardStatus fill in structs through pointers.
// A plain return_val leaves them all zeros, so these custom fakes describe a valid card.
static HAL_StatusTypeDef fake_get_card_info(SD_HandleTypeDef *, HAL_SD_CardInfoTypeDef *info) {
	memset(info, 0, sizeof(*info));
	info->BlockNbr = FAKE_CARD_BLOCKS;
	info->BlockSize = BLOCKSIZE;
	info->LogBlockNbr = FAKE_CARD_BLOCKS;
	info->LogBlockSize = BLOCKSIZE;
	return HAL_OK;
}

static HAL_StatusTypeDef fake_get_card_status(SD_HandleTypeDef *,
                                              HAL_SD_CardStatusTypeDef *status) {
	memset(status, 0, sizeof(*status));
	status->AllocationUnitSize = 0x9; // SD spec AU code: 4 MB
	status->EraseSize = 1;
	return HAL_OK;
}

static uint32_t fake_notify_dma_complete(UBaseType_t, BaseType_t, TickType_t ticks) {
	if (0U == ticks) {
		return 0U; // call 1: nothing pending
	}
	if (RAW_READING == action_state) {
		HAL_SD_RxCpltCallback(raw_logger_handle);
	} else if (RAW_WRITING == action_state) {
		HAL_SD_TxCpltCallback(raw_logger_handle);
	}
	return 1U; // call 2: notification received
}

// Same idea, but the DMA reports an error instead of finishing.
static uint32_t fake_notify_dma_error(UBaseType_t, BaseType_t, TickType_t ticks) {
	if (0U == ticks) {
		return 0U;
	}
	HAL_SD_ErrorCallback(raw_logger_handle);
	return 1U;
}

class RawSDTest : public ::testing::Test {
protected:
	SD_HandleTypeDef hsd;
	uint8_t buffer[BLOCKSIZE];

	void SetUp() override {
		RESET_FAKE(xSemaphoreCreateMutex);
		RESET_FAKE(xSemaphoreTake);
		RESET_FAKE(xSemaphoreGive);
		RESET_FAKE(xTaskGetCurrentTaskHandle);
		RESET_FAKE(vTaskDelay);
		RESET_FAKE(timer_get_ms);
		RESET_FAKE(ulTaskNotifyTakeIndexed);
		RESET_FAKE(vTaskNotifyGiveIndexedFromISR);
		RESET_FAKE(HAL_SD_GetCardState);
		RESET_FAKE(HAL_SD_GetCardInfo);
		RESET_FAKE(HAL_SD_GetCardStatus);
		RESET_FAKE(HAL_SD_ReadBlocks_DMA);
		RESET_FAKE(HAL_SD_WriteBlocks_DMA);
		RESET_FAKE(HAL_SD_Abort);
		RESET_FAKE(log_text);
		FFF_RESET_HISTORY();

		// Default: everything succeeds
		xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
		xSemaphoreTake_fake.return_val = pdTRUE;
		xTaskGetCurrentTaskHandle_fake.return_val = (TaskHandle_t)1; // callbacks ignore NULL
		HAL_SD_GetCardState_fake.return_val = HAL_SD_CARD_TRANSFER;
		HAL_SD_GetCardInfo_fake.custom_fake = fake_get_card_info;
		HAL_SD_GetCardStatus_fake.custom_fake = fake_get_card_status;
		HAL_SD_ReadBlocks_DMA_fake.return_val = HAL_OK;
		HAL_SD_WriteBlocks_DMA_fake.return_val = HAL_OK;
		ulTaskNotifyTakeIndexed_fake.custom_fake = fake_notify_dma_complete;
		timer_get_ms_fake.return_val = W_SUCCESS;

		memset(&hsd, 0, sizeof(hsd));
		hsd.SdCard.LogBlockNbr = FAKE_CARD_BLOCKS;
		memset(buffer, 0, sizeof(buffer));
	}

	void TearDown() override {
		// Put every piece of driver state back to how it is at power-on
		memset(&raw_sd_card_health, 0, sizeof(raw_sd_card_health));
		sd_mutex = NULL;
		raw_logger_handle = NULL;
		task_handler = NULL;
		action_state = RAW_UNINIT;
	}
};

TEST_F(RawSDTest, InitSuccess) {
	EXPECT_EQ(raw_logger_SD_init(&hsd), W_SUCCESS);
	EXPECT_EQ(action_state, RAW_READY);
}

TEST_F(RawSDTest, InitFail) {
	EXPECT_EQ(raw_logger_SD_init(NULL), W_FAILURE);
}

TEST_F(RawSDTest, InitMutexFail) {
	xSemaphoreCreateMutex_fake.return_val = NULL;
	EXPECT_EQ(raw_logger_SD_init(&hsd), W_FAILURE);
}

TEST_F(RawSDTest, ReadNotInitFail) {
	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
}

TEST_F(RawSDTest, ReadSuccess) {
	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_SUCCESS);
	EXPECT_EQ(raw_sd_card_health.read_count, 1U);
	EXPECT_EQ(action_state, RAW_READY);
	// The mutex taken at the start must be given back
	EXPECT_EQ(xSemaphoreGive_fake.call_count, xSemaphoreTake_fake.call_count);
}

TEST_F(RawSDTest, WriteSuccess) {
	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_write(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_SUCCESS);
	EXPECT_EQ(raw_sd_card_health.write_count, 1U);
	EXPECT_EQ(action_state, RAW_READY);
	EXPECT_EQ(xSemaphoreGive_fake.call_count, xSemaphoreTake_fake.call_count);
}

TEST_F(RawSDTest, ReadInvalidParamFail) {
	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(NULL, BLOCKSIZE, BLOCKSIZE, 200), W_INVALID_PARAM);
	EXPECT_EQ(raw_logger_SD_read(buffer, 0, BLOCKSIZE, 200), W_INVALID_PARAM);
	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, 0, 200), W_INVALID_PARAM);
	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE - 1, 200), W_INVALID_PARAM);
	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE - 1, BLOCKSIZE, 200), W_INVALID_PARAM);
	// Past the end of the card
	EXPECT_EQ(raw_logger_SD_read(buffer, (uint64_t)FAKE_CARD_BLOCKS * BLOCKSIZE, BLOCKSIZE, 200),
	          W_INVALID_PARAM);
}

TEST_F(RawSDTest, TimerFail) {
	timer_get_ms_fake.return_val = W_FAILURE;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
	EXPECT_EQ(raw_logger_SD_write(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
}

TEST_F(RawSDTest, MutexTakeFail) {
	xSemaphoreTake_fake.return_val = pdFALSE;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_IO_TIMEOUT);
	EXPECT_EQ(raw_logger_SD_write(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_IO_TIMEOUT);
	EXPECT_EQ(HAL_SD_ReadBlocks_DMA_fake.call_count, 0U);
	EXPECT_EQ(HAL_SD_WriteBlocks_DMA_fake.call_count, 0U);
}

TEST_F(RawSDTest, IndexTakeFail) {
	// DMA never signals completion. custom_fake wins over return_val, so clear it.
	ulTaskNotifyTakeIndexed_fake.custom_fake = NULL;
	ulTaskNotifyTakeIndexed_fake.return_val = 0U;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_IO_TIMEOUT);
	EXPECT_EQ(action_state, RAW_RECOVERING);
	EXPECT_EQ(HAL_SD_Abort_fake.call_count, 1U);

	EXPECT_EQ(raw_logger_SD_write(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_IO_TIMEOUT);
	EXPECT_EQ(HAL_SD_Abort_fake.call_count, 2U);
}

TEST_F(RawSDTest, ReadHALFail) {
	HAL_SD_ReadBlocks_DMA_fake.return_val = HAL_ERROR;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
	EXPECT_EQ(raw_sd_card_health.read_error, 1U);
}

TEST_F(RawSDTest, WriteHALFail) {
	HAL_SD_WriteBlocks_DMA_fake.return_val = HAL_ERROR;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_write(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
	EXPECT_EQ(raw_sd_card_health.write_error, 1U);
}

TEST_F(RawSDTest, ReadDMAErrorCallback) {
	ulTaskNotifyTakeIndexed_fake.custom_fake = fake_notify_dma_error;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_read(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
	EXPECT_EQ(raw_sd_card_health.read_error, 1U);
	EXPECT_EQ(action_state, RAW_READY); // driver recovers for the next call
}

TEST_F(RawSDTest, WriteDMAErrorCallback) {
	ulTaskNotifyTakeIndexed_fake.custom_fake = fake_notify_dma_error;

	raw_logger_SD_init(&hsd);

	EXPECT_EQ(raw_logger_SD_write(buffer, BLOCKSIZE, BLOCKSIZE, 200), W_FAILURE);
	EXPECT_EQ(raw_sd_card_health.write_error, 1U);
	EXPECT_EQ(action_state, RAW_READY);
}
