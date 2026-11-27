#include "fff.h"
#include <gtest/gtest.h>
#include <inttypes.h> // included first so the .c's copy isn't pulled in inside extern "C"
#include <string.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "application/logging_formatter/logging_formatter.h"
#include "drivers/raw_logger/raw_logger.h"
#include "stm32h7xx_hal.h"
#include "task.h"

// The formatter is under test, so the raw logger below it is faked.
// Signatures must match raw_logger.h exactly.
FAKE_VALUE_FUNC(w_status_t, raw_logger_SD_init, SD_HandleTypeDef *);
FAKE_VALUE_FUNC(w_status_t, raw_logger_SD_read, uint8_t *, uint64_t, uint32_t, uint32_t);
FAKE_VALUE_FUNC(w_status_t, raw_logger_SD_write, const uint8_t *, uint64_t, uint32_t, uint32_t);
FAKE_VALUE_FUNC(w_status_t, raw_logger_get_SD_specs, uint64_t *, uint32_t *, uint32_t);
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, log_level_t, const char *, const char *,
                       ...);

// Compile the formatter into this test so its static variables can be read and reset.
// logging_formatter.c and raw_logger.c must NOT also be sources of this test target in CMake.
#include "application/logging_formatter/logging_formatter.c"
}

DEFINE_FFF_GLOBALS;

#define TOTAL_CHUNKS 100U // pretend card size, in chunks
#define PAYLOAD_SIZE (CHUNK_SIZE - FORMATTER_HEADER_SIZE - FORMATTER_CRC_SIZE)

// Chunks 1 .. g_first_empty_chunk-1 already hold data; everything after is blank.
static uint64_t g_first_empty_chunk;

// raw_logger_SD_read fills a buffer through a pointer, so it needs a custom fake.
static w_status_t fake_card_read(uint8_t *buffer, uint64_t byte_address, uint32_t num_bytes,
                                 uint32_t) {
	memset(buffer, 0, num_bytes);

	uint64_t chunk = byte_address / CHUNK_SIZE;
	if ((chunk >= 1U) && (chunk < g_first_empty_chunk)) {
		memcpy(buffer, HEADER, 4); // written chunk: non-zero header, file identifier 0
	}
	return W_SUCCESS;
}

static w_status_t fake_get_specs(uint64_t *block_count, uint32_t *erase_size, uint32_t) {
	*block_count = (uint64_t)TOTAL_CHUNKS * BLOCKS_PER_CHUNK;
	*erase_size = 4U * 1024U * 1024U;
	return W_SUCCESS;
}

class FormatterTest : public ::testing::Test {
protected:
	SD_HandleTypeDef hsd;
	uint8_t payload[PAYLOAD_SIZE];
	formatter_file_ctx_t ctx;

	void SetUp() override {
		RESET_FAKE(raw_logger_SD_init);
		RESET_FAKE(raw_logger_SD_read);
		RESET_FAKE(raw_logger_SD_write);
		RESET_FAKE(raw_logger_get_SD_specs);
		RESET_FAKE(log_text);
		RESET_FAKE(HAL_SD_GetCardState);
		RESET_FAKE(vTaskDelay);
		FFF_RESET_HISTORY();

		// Default: blank card, everything succeeds
		g_first_empty_chunk = 1;
		raw_logger_SD_init_fake.return_val = W_SUCCESS;
		raw_logger_SD_read_fake.custom_fake = fake_card_read;
		raw_logger_SD_write_fake.return_val = W_SUCCESS;
		raw_logger_get_SD_specs_fake.custom_fake = fake_get_specs;
		// The formatter loops until the card reports TRANSFER, so without this the tests hang
		HAL_SD_GetCardState_fake.return_val = HAL_SD_CARD_TRANSFER;

		memset(&hsd, 0, sizeof(hsd));
		memset(&ctx, 0, sizeof(ctx));
		memset(payload, 0xab, sizeof(payload));
	}

	void TearDown() override {
		// Put the formatter's static state back to power-on
		memset(&logging_formatter_health, 0, sizeof(logging_formatter_health));
		memset(&formatter_buffer, 0, sizeof(formatter_buffer));
		logging_formatter_handle = NULL;
		chunk_num = 0;
	}
};

TEST_F(FormatterTest, InitBlankCardSuccess) {
	EXPECT_EQ(logging_formatter_init(&hsd), W_SUCCESS);
	EXPECT_EQ(formatter_buffer.chunk, 1U); // blank card starts at chunk 1
}

TEST_F(FormatterTest, InitUsedCardSuccess) {
	g_first_empty_chunk = 37;

	EXPECT_EQ(logging_formatter_init(&hsd), W_SUCCESS);
	EXPECT_EQ(formatter_buffer.chunk, 37U); // binary search found the first blank chunk
}

TEST_F(FormatterTest, CreateFileSuccess) {
	ASSERT_EQ(logging_formatter_init(&hsd), W_SUCCESS);

	EXPECT_EQ(logging_formatter_create_file(&ctx, LOGGING_FORMATTING_DATA), W_SUCCESS);
	EXPECT_EQ(ctx.chunk_count, 0U);
	EXPECT_EQ(ctx.data_type, LOGGING_FORMATTING_DATA);
}

TEST_F(FormatterTest, WriteSuccess) {
	ASSERT_EQ(logging_formatter_init(&hsd), W_SUCCESS);
	ASSERT_EQ(logging_formatter_create_file(&ctx, LOGGING_FORMATTING_DATA), W_SUCCESS);

	EXPECT_EQ(logging_formatter_write(payload, PAYLOAD_SIZE, &ctx), W_SUCCESS);

	// One full chunk written to chunk 1
	EXPECT_EQ(raw_logger_SD_write_fake.call_count, 1U);
	EXPECT_EQ(raw_logger_SD_write_fake.arg1_val, (uint64_t)CHUNK_SIZE);
	EXPECT_EQ(raw_logger_SD_write_fake.arg2_val, (uint32_t)CHUNK_SIZE);

	// Header first, then the payload
	const uint8_t *chunk = raw_logger_SD_write_fake.arg0_val;
	EXPECT_EQ(memcmp(chunk, HEADER, 4), 0);
	EXPECT_EQ(memcmp(&chunk[FORMATTER_HEADER_SIZE], payload, PAYLOAD_SIZE), 0);

	// Next write goes to the next chunk
	EXPECT_EQ(ctx.chunk_count, 1U);
	EXPECT_EQ(formatter_buffer.chunk, 2U);
}
