#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "message_types.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "semphr.h"
#include "third_party/printf/printf.h"

// number of times to try writing a log message in case fails once
#define LOG_WRITE_TRY_COUNT 2

/* Filename for the master log index file that stores the run count */
static const char *LOG_RUN_COUNT_FILENAME = "LOGRUN.BIN";

static char text_log_filename[8 + 1 + 3 + 1] = "XXXXXXXX.TXT";
static char data_log_filename[8 + 1 + 3 + 1] = "XXXXXXXX.BIN";

typedef struct {
	bool is_text;
	bool is_full;
	uint32_t next_msg_num;
	SemaphoreHandle_t next_msg_num_mutex;
	SemaphoreHandle_t msgs_done_semaphore;
	char data[LOG_BUFFER_SIZE];
} log_buffer_t;

static log_buffer_t log_text_buffers[NUM_TEXT_LOG_BUFFERS];
static log_buffer_t log_data_buffers[NUM_DATA_LOG_BUFFERS];
static uint32_t current_text_buf_num = 0;
static uint32_t current_data_buf_num = 0;
/* Mutex that protects access of current_text_buf_num */
static SemaphoreHandle_t log_text_write_mutex = NULL;
/* Mutex that protects access of current_data_buf_num */
static SemaphoreHandle_t log_data_write_mutex = NULL;
/* Queue of full log buffers sent to the task to be flushed */
static QueueHandle_t full_buffers_queue = NULL;
/* Count of data buffers initialized, for buffer header index value */
static uint32_t total_data_log_buffers = 0;

static logger_health_t logger_health = {0};

/**
 * @brief Write a data log message's content to a specific region in a given buffer.
 * @note Not for external use.
 * @param buffer Pointer to data log buffer to write into
 * @param msg_num Index of the message region to write to
 * @param type Type of data log message
 * @param timestamp Timestamp of data log message
 * @param data Pointer to raw payload data of data log message
 */
static w_status_t log_data_write_to_region(log_buffer_t *const buffer, const uint32_t msg_num,
										   log_data_type_t type, float timestamp,
										   const log_data_container_t *data) {
	// Validate arguments
	// Assumption: logger_health.is_init == true OR (during init) all buffer semaphores are valid
	if ((NULL == buffer) || (msg_num >= DATA_MSGS_PER_BUFFER) || (NULL == data)) {
		return W_INVALID_PARAM;
	}

	// Get pointer to message region to write to
	char *const msg_dest = buffer->data + (msg_num * MAX_DATA_MSG_LENGTH);

	uint32_t chars_written = 0;
	bool trunc = false;
	// Write log message type
	uint32_t type_int = (uint32_t)type;
	size_t size = sizeof(type_int);
	// Truncate if write would extend beyond message region excluding last char for newline
	if (MAX_DATA_MSG_LENGTH - 1 - chars_written < size) {
		size = MAX_DATA_MSG_LENGTH - 1 - chars_written;
		trunc = true;
	}
	memcpy(msg_dest + chars_written, &type_int, size);
	chars_written += size;

	// Write log message timestamp
	size = sizeof(timestamp);
	if (MAX_DATA_MSG_LENGTH - 1 - chars_written < size) {
		size = MAX_DATA_MSG_LENGTH - 1 - chars_written;
		trunc = true;
	}
	memcpy(msg_dest + chars_written, &timestamp, size);
	chars_written += size;

	// Write log message data
	size = sizeof(*data);
	if (MAX_DATA_MSG_LENGTH - 1 - chars_written < size) {
		size = MAX_DATA_MSG_LENGTH - 1 - chars_written;
		trunc = true;
	}
	memcpy(msg_dest + chars_written, data, size);
	chars_written += size;

	// Remaining bytes have already been zeroed by log_reset_buffer
	// Set the last char of the message buffer to a newline
	msg_dest[MAX_DATA_MSG_LENGTH - 1] = '\n';

	if (trunc) {
		// TODO: mark the message as truncated
		logger_health.trunc_msgs++;
	}

	// Increment the number of completed log messages in this buffer
	xSemaphoreGive(buffer->msgs_done_semaphore);
	// If last message in buffer, send buffer to queue
	if (msg_num == DATA_MSGS_PER_BUFFER - 1) {
		if (xQueueSendToBack(full_buffers_queue, &buffer, 0) != pdPASS) {
			// This should never be reached as the queue should have space for all buffers
			logger_health.crit_errs++;
			return W_FAILURE;
		}
	}
	// msg_num is guaranteed not to be greater than DATA_MSGS_PER_BUFFER by condition before write

	return W_SUCCESS;
}

/**
 * @brief Reset a log buffer for reuse, and if it's a data buffer, write its buffer header message.
 * @details Resets is_full, next_msg_num, msgs_done_semaphore, and clears the data region
 * @param buffer Pointer to the log buffer to reset
 */
static void log_reset_buffer(log_buffer_t *buffer) {
	// Reset counting semaphore to 0 by taking without delay until failure
	while (xSemaphoreTake(buffer->msgs_done_semaphore, 0) == pdPASS) {}

	// Clear data region
	memset(buffer->data, 0, LOG_BUFFER_SIZE);

	// Write the buffer header message if it's a data buffer
	if (!buffer->is_text) {
		log_data_container_t data = {0};
		data.header.version = LOG_DATA_FORMAT_VERSION;
		data.header.index = total_data_log_buffers;
		// Use dummy timestamp of -1.0f if failed to get timestamp, as in log_text()
		float timestamp = -1.0f;
		(void)timer_get_ms(&timestamp);
		log_data_write_to_region(buffer, 0, LOG_TYPE_HEADER, timestamp, &data);
		total_data_log_buffers++;

		// Reset these last to avoid another task using this buffer before we're done resetting it
		buffer->next_msg_num = 1;
	} else {
		buffer->next_msg_num = 0;
	}
	buffer->is_full = false;
}

w_status_t log_init(void) {
	// alr init so its good to go
	if (logger_health.is_init) {
		return W_SUCCESS;
	}

	full_buffers_queue =
		xQueueCreate(NUM_TEXT_LOG_BUFFERS + NUM_DATA_LOG_BUFFERS, sizeof(log_buffer_t *));
	if (NULL == full_buffers_queue) {
		return W_FAILURE;
	}

	log_text_write_mutex = xSemaphoreCreateMutex();
	if (NULL == log_text_write_mutex) {
		return W_FAILURE;
	}
	log_data_write_mutex = xSemaphoreCreateMutex();
	if (NULL == log_data_write_mutex) {
		return W_FAILURE;
	}

	// Initialize text buffers
	for (int i = 0; i < NUM_TEXT_LOG_BUFFERS; i++) {
		log_buffer_t *buffer = &log_text_buffers[i];
		buffer->is_text = true;
		buffer->next_msg_num_mutex = xSemaphoreCreateMutex();
		if (NULL == buffer->next_msg_num_mutex) {
			return W_FAILURE;
		}
		buffer->msgs_done_semaphore = xSemaphoreCreateCounting(TEXT_MSGS_PER_BUFFER, 0);
		if (NULL == buffer->msgs_done_semaphore) {
			return W_FAILURE;
		}
		log_reset_buffer(buffer);
	}

	// Initialize data buffers
	for (int i = 0; i < NUM_DATA_LOG_BUFFERS; i++) {
		log_buffer_t *buffer = &log_data_buffers[i];
		buffer->is_text = false;
		buffer->next_msg_num_mutex = xSemaphoreCreateMutex();
		if (NULL == buffer->next_msg_num_mutex) {
			return W_FAILURE;
		}
		buffer->msgs_done_semaphore = xSemaphoreCreateCounting(DATA_MSGS_PER_BUFFER, 0);
		if (NULL == buffer->msgs_done_semaphore) {
			return W_FAILURE;
		}
		log_reset_buffer(buffer);
	}

	// Read the logger run count file
	char run_count_buf[sizeof(uint32_t)] = {0};
	uint32_t size = 0;
	w_status_t status =
		sd_card_file_read(LOG_RUN_COUNT_FILENAME, run_count_buf, sizeof(run_count_buf), &size);
	// Get new run count
	uint32_t run_count = 0;
	if ((W_SUCCESS == status) && (sizeof(run_count_buf) >= size)) {
		memcpy(&run_count, run_count_buf, sizeof(run_count));
		run_count++;
	} else {
		return W_IO_ERROR;
	}

	// Write new run count to file
	memcpy(run_count_buf, &run_count, sizeof(run_count));
	// use append=true to overwrite the existing count
	status |= sd_card_file_write(
		LOG_RUN_COUNT_FILENAME, run_count_buf, sizeof(run_count_buf), false, &size);

	// Form log filenames using the run count
	snprintf_(text_log_filename, sizeof(text_log_filename), "%08" PRIx32 ".TXT", run_count);
	snprintf_(data_log_filename, sizeof(data_log_filename), "%08" PRIx32 ".BIN", run_count);

	// Create log files
	status |= sd_card_file_create(text_log_filename);
	status |= sd_card_file_create(data_log_filename);

	if (W_SUCCESS == status) {
		logger_health.is_init = true;
	}
	return status;
}

w_status_t log_text(uint32_t timeout, const char *source, const char *format, ...) {
	// Get timestamp as close to time of call as possible
	// If we fail to get a timestamp, use a dummy value of -1.0f and continue to write the log
	// message anyway. We're trying to log as much as possible and a missing timestamp does not
	// inherently critically affect the ability to write the log message
	float timestamp = -1.0f;
	(void)timer_get_ms(&timestamp);

	// Validate arguments
	if ((!logger_health.is_init) || (NULL == source) || (NULL == format)) {
		return W_INVALID_PARAM;
	}

	// Attempt to acquire mutex to safely access current_text_buf_num
	if (xSemaphoreTake(log_text_write_mutex, pdMS_TO_TICKS(timeout)) != pdPASS) {
		logger_health.log_write_timeouts++;
		logger_health.dropped_txt_msgs++;
		return W_FAILURE;
	}

	// Get text buffer to write into
	log_buffer_t *const buffer = &log_text_buffers[current_text_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_text_write_mutex);
		logger_health.full_buffer_moments++;
		logger_health.dropped_txt_msgs++;
		return W_FAILURE;
	}

	// Get index of message region to write to
	const uint32_t msg_num = buffer->next_msg_num;
	buffer->next_msg_num++;

	// If there are no more message regions in this buffer, mark it full and advance to the next
	// buffer
	if (buffer->next_msg_num >= TEXT_MSGS_PER_BUFFER) {
		buffer->is_full = true;
		current_text_buf_num = (current_text_buf_num + 1) % NUM_TEXT_LOG_BUFFERS;
	}

	xSemaphoreGive(log_text_write_mutex);

	if (msg_num >= TEXT_MSGS_PER_BUFFER) {
		logger_health.invalid_region_moments++;
		logger_health.dropped_txt_msgs++;
		return W_FAILURE;
	}

	// Get pointer to message region to write to
	char *const msg_dest = buffer->data + (msg_num * MAX_TEXT_MSG_LENGTH);

	uint32_t chars_written = 0;
	// Write log message header to region
	chars_written += snprintf_(msg_dest + chars_written,
							   MAX_TEXT_MSG_LENGTH - chars_written,
							   "[%.1f] %s;",
							   timestamp,
							   source);
	// If truncated, set first char to '!'
	if (chars_written >= MAX_TEXT_MSG_LENGTH) {
		msg_dest[0] = '!';
		logger_health.trunc_msgs++;
	} else {
		// Write log message contents to region
		va_list format_args;
		va_start(format_args, format);
		chars_written += vsnprintf_(
			msg_dest + chars_written, MAX_TEXT_MSG_LENGTH - chars_written, format, format_args);
		va_end(format_args);
		// If truncated, set first char to '!'
		if (chars_written >= MAX_TEXT_MSG_LENGTH) {
			msg_dest[0] = '!';
			logger_health.trunc_msgs++;
		}
	}
	// Remaining bytes have already been zeroed by log_reset_buffer
	// Set the last char of the message buffer to a newline
	msg_dest[MAX_TEXT_MSG_LENGTH - 1] = '\n';

	// Increment the number of completed log messages in this buffer
	xSemaphoreGive(buffer->msgs_done_semaphore);
	// If last message in buffer, send buffer to queue
	if (msg_num == TEXT_MSGS_PER_BUFFER - 1) {
		if (xQueueSendToBack(full_buffers_queue, &buffer, 0) != pdPASS) {
			// This should never be reached as the queue should have space for all buffers
			logger_health.crit_errs++;
			return W_FAILURE;
		}
	}
	// msg_num is guaranteed not to be greater than TEXT_MSGS_PER_BUFFER by condition before write

	return W_SUCCESS;
}

w_status_t log_data(uint32_t timeout, log_data_type_t type, const log_data_container_t *data) {
	// Get timestamp as close to time of call as possible
	// Use dummy timestamp of -1.0f if failed to get timestamp, as in log_text()
	float timestamp = -1.0f;
	(void)timer_get_ms(&timestamp);

	// Validate arguments
	if ((!logger_health.is_init) || (NULL == data)) {
		return W_INVALID_PARAM;
	}

	// Attempt to acquire mutex to safely access current_data_buf_num
	if (xSemaphoreTake(log_data_write_mutex, pdMS_TO_TICKS(timeout)) != pdPASS) {
		logger_health.log_write_timeouts++;
		logger_health.dropped_data_msgs++;
		return W_FAILURE;
	}

	// Get text buffer to write into
	log_buffer_t *const buffer = &log_data_buffers[current_data_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_data_write_mutex);
		logger_health.full_buffer_moments++;
		logger_health.dropped_data_msgs++;
		return W_FAILURE;
	}

	// Get index of message region to write to
	const uint32_t msg_num = buffer->next_msg_num;
	buffer->next_msg_num++;

	// If there are no more message regions in this buffer, mark it full and advance to the next
	// buffer
	if (buffer->next_msg_num >= DATA_MSGS_PER_BUFFER) {
		buffer->is_full = true;
		current_data_buf_num = (current_data_buf_num + 1) % NUM_DATA_LOG_BUFFERS;
	}

	xSemaphoreGive(log_data_write_mutex);

	// Write the message to the buffer
	w_status_t res = log_data_write_to_region(buffer, msg_num, type, timestamp, data);
	if (W_INVALID_PARAM == res) {
		// msg_num was invalid (is_init, buffer, data must all have made sense at this point)
		logger_health.invalid_region_moments++;
		logger_health.dropped_data_msgs++;
		return W_FAILURE;
	}
	return res;
}

void log_task(void *argument) {
	(void)argument;

	log_buffer_t *buffer_to_print = NULL;
	for (;;) {
		if (xQueueReceive(full_buffers_queue, &buffer_to_print, 5000) == pdPASS) {
			// Retrieve number of completed log messages in the received buffer
			uint32_t msgs_done = 0;
			uint32_t max_msgs = TEXT_MSGS_PER_BUFFER;
			char *filename = text_log_filename;
			if (!buffer_to_print->is_text) {
				max_msgs = DATA_MSGS_PER_BUFFER;
				filename = data_log_filename;
			}
			while (msgs_done < max_msgs) {
				if (xSemaphoreTake(buffer_to_print->msgs_done_semaphore, 10) == pdPASS) {
					msgs_done++;
				} else {
					logger_health.unsafe_buffer_flushes++;
					break;
				}
			}

			// try several times to buffer to SD card
			uint32_t size = 0;
			for (uint32_t i = 0; i < LOG_WRITE_TRY_COUNT; i++) {
				gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_LOW, 0);

				if (sd_card_file_write(
						filename, buffer_to_print->data, LOG_BUFFER_SIZE, true, &size) ==
					W_SUCCESS) {
					gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 0);
					break; // Successfully wrote the buffer
				} else {
					gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 0);
				}
				if ((LOG_WRITE_TRY_COUNT - 1) == i) {
					logger_health.buffer_flush_fails++;
					break; // Failed to write after all attempts
				} else {
					// wait some time before retrying, shld allow wear leveling to finish ideally..
					vTaskDelay(pdMS_TO_TICKS(20));
				}
			}

			// Reinitialize buffer for reuse
			log_reset_buffer(buffer_to_print);
		} else {
			logger_health.no_full_buf_moments++;
		}
	}
}

uint32_t logger_get_status(void) {
	uint32_t status_bitfield = 0;

	if (!logger_health.is_init) {
		log_text(0, "logger", "not init");
		return 1 << E_FS_ERROR_OFFSET;
	}

	log_text(10,
			 "logger",
			 "init=%d, drop_txt=%d, drop_data=%d, trunc=%d, "
			 "full_buff=%d, log_w_timeouts=%d",
			 logger_health.is_init,
			 logger_health.dropped_txt_msgs,
			 logger_health.dropped_data_msgs,
			 logger_health.trunc_msgs,
			 logger_health.full_buffer_moments,
			 logger_health.log_write_timeouts);

	log_text(10,
			 "logger",
			 "invalid_region=%d, crit_errs=%d, no_full_buf=%d, "
			 "buffer_flush_fails=%d, unsafe_buffer_flush=%d",
			 logger_health.invalid_region_moments,
			 logger_health.crit_errs,
			 logger_health.no_full_buf_moments,
			 logger_health.buffer_flush_fails,
			 logger_health.unsafe_buffer_flushes);

	return status_bitfield;
}
