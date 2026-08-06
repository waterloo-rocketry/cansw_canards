#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
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
#define DATA_WRITE_TRY_COUNT 50
#define TEXT_WRITE_TRY_COUNT 20

static const uint32_t MAX_TXT_LOGGING_PERIOD_MS = 120000; // 2 min

/* Filename for the master log index file that stores the run count */
static const char *LOG_RUN_COUNT_FILENAME = "LOGRUN.BIN";

static char text_log_filename[FILENAME_STRING_SIZE] = "XXXXXXXX.TXT";
static char data_log_filename[FILENAME_STRING_SIZE] = "XXXXXXXX.BIN";

typedef struct {
	char data[LOG_BUFFER_SIZE];
	uint32_t next_msg_num;
	SemaphoreHandle_t next_msg_num_mutex;
	SemaphoreHandle_t msgs_done_semaphore;
	bool is_text;
	bool is_full;
} log_buffer_t;

/**
 * A collection of status variables describing the current health of the logger module.
 */
typedef struct {
	bool is_init;
	uint32_t trunc_msgs;
	uint32_t full_buffer_moments;
	uint32_t log_write_timeouts;
	uint32_t invalid_region_moments;
	uint32_t queue_send_fails;
	uint32_t no_full_buf_moments;
	uint32_t buffer_flush_fails;
	uint32_t unsafe_buffer_flushes;
	uint32_t null_param_count;
	bool buffer_is_full; // flag for full buffer since last health check
	bool timeout_occurred; // flag for timeout since last health check
	bool os_error_occurred; // flag for queue send fail
	bool invalid_param; // flag for NULL/bad-arg call since last health check
	bool region_overflow; // flag for invalid msg region index since last health check
} logger_health_t;

static log_buffer_t log_text_buffers[NUM_TEXT_LOG_BUFFERS];
static log_buffer_t log_data_buffers[NUM_DATA_LOG_BUFFERS];
static uint32_t current_text_buf_num = 0;
static uint32_t current_data_buf_num = 0;
/* Mutex that protects access of current_text_buf_num */
static SemaphoreHandle_t log_text_write_mutex = NULL;
/* Mutex that protects access of current_data_buf_num */
static SemaphoreHandle_t log_data_write_mutex = NULL;
/* Queue of full log buffers sent to the task to be flushed */
static QueueHandle_t data_buffers_queue = NULL;
static QueueHandle_t text_buffers_queue = NULL;
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
										   log_data_type_t type, uint32_t timestamp,
										   const log_data_container_t *data) {
	// Validate arguments
	// Assumption: logger_health.is_init == true OR (during init) all buffer semaphores are valid
	if ((NULL == buffer) || (msg_num >= DATA_MSGS_PER_BUFFER) || (NULL == data)) {
		logger_health.null_param_count++;
		logger_health.invalid_param = true;
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
	if (MAX_DATA_MSG_LENGTH - chars_written < size) {
		size = MAX_DATA_MSG_LENGTH - chars_written;
		trunc = true;
	}
	memcpy(msg_dest + chars_written, &type_int, size);
	chars_written += size;

	// Write log message timestamp
	size = sizeof(timestamp);
	if (MAX_DATA_MSG_LENGTH - chars_written < size) {
		size = MAX_DATA_MSG_LENGTH - chars_written;
		trunc = true;
	}
	memcpy(msg_dest + chars_written, &timestamp, size);
	chars_written += size;

	// Write log message data
	size = sizeof(*data);
	if (MAX_DATA_MSG_LENGTH - chars_written < size) {
		size = MAX_DATA_MSG_LENGTH - chars_written;
		trunc = true;
	}
	memcpy(msg_dest + chars_written, data, size);
	chars_written += size;

	if (trunc) {
		// TODO: mark the message as truncated
		logger_health.trunc_msgs++;
	}

	// Increment the number of completed log messages in this buffer
	xSemaphoreGive(buffer->msgs_done_semaphore);
	// If last message in buffer, send buffer to queue
	if (msg_num == DATA_MSGS_PER_BUFFER - 1) {
		if (xQueueSendToBack(data_buffers_queue, &buffer, 0) != pdPASS) {
			// This should never be reached as the queue should have space for all buffers
			logger_health.queue_send_fails++;
			logger_health.os_error_occurred = true;
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
		// Use dummy timestamp of 0 if failed to get timestamp, as in log_text()
		uint32_t timestamp = 0;
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

	// Note: log_text() no-ops until logger_health.is_init is set at the end of this function, so
	// these calls won't actually reach the log file on a real failure here. Kept for consistency
	// with other modules' init-failure logging and in case that gating ever changes.

	full_buffers_queue =
		xQueueCreate(NUM_TEXT_LOG_BUFFERS + NUM_DATA_LOG_BUFFERS, sizeof(log_buffer_t *));
	if (NULL == full_buffers_queue) {
		return W_FAILURE;
	}

	log_text_write_mutex = xSemaphoreCreateMutex();
	if (NULL == log_text_write_mutex) {
		log_text(1, LOG_LVL_WARN, "logger", "Failed to create text write mutex.");
		return W_FAILURE;
	}
	log_data_write_mutex = xSemaphoreCreateMutex();
	if (NULL == log_data_write_mutex) {
		log_text(1, LOG_LVL_WARN, "logger", "Failed to create data write mutex.");
		return W_FAILURE;
	}

	// Initialize text buffers
	for (int i = 0; i < NUM_TEXT_LOG_BUFFERS; i++) {
		log_buffer_t *buffer = &log_text_buffers[i];
		buffer->is_text = true;
		buffer->next_msg_num_mutex = xSemaphoreCreateMutex();
		if (NULL == buffer->next_msg_num_mutex) {
			log_text(1, LOG_LVL_WARN, "logger", "Failed to create text buffer mutex.");
			return W_FAILURE;
		}
		buffer->msgs_done_semaphore = xSemaphoreCreateCounting(TEXT_MSGS_PER_BUFFER, 0);
		if (NULL == buffer->msgs_done_semaphore) {
			log_text(1, LOG_LVL_WARN, "logger", "Failed to create text buffer semaphore.");
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
			log_text(1, LOG_LVL_WARN, "logger", "Failed to create data buffer mutex.");
			return W_FAILURE;
		}
		buffer->msgs_done_semaphore = xSemaphoreCreateCounting(DATA_MSGS_PER_BUFFER, 0);
		if (NULL == buffer->msgs_done_semaphore) {
			log_text(1, LOG_LVL_WARN, "logger", "Failed to create data buffer semaphore.");
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
		// LOGRUN.BIN missing
		run_count = 1;
		status = sd_card_file_create(LOG_RUN_COUNT_FILENAME);
		if (W_SUCCESS != status) {
			log_text(1, LOG_LVL_WARN, "logger", "Failed to create run count file.");
			return W_IO_ERROR;
		}
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
	} else {
		log_text(1, LOG_LVL_WARN, "logger", "Failed to write run count or create log files.");
	}
	return status;
}

w_status_t log_text(uint32_t timeout, log_level_t level, const char *source, const char *format,
					...) {
	// Get timestamp as close to time of call as possible
	// If we fail to get a timestamp, use a dummy value of 0 and continue to write the log
	// message anyway. We're trying to log as much as possible and a missing timestamp does not
	// inherently critically affect the ability to write the log message
	uint32_t timestamp = 0;
	(void)timer_get_ms(&timestamp);

	// Validate arguments
	if (!logger_health.is_init) {
		return W_FAILURE;
	}

	if ((NULL == source) || (NULL == format)) {
		logger_health.null_param_count++;
		logger_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	// Attempt to acquire mutex to safely access current_text_buf_num
	if (xSemaphoreTake(log_text_write_mutex, pdMS_TO_TICKS(timeout)) != pdPASS) {
		logger_health.log_write_timeouts++;
		logger_health.timeout_occurred = true;
		return W_FAILURE;
	}

	// Get text buffer to write into
	log_buffer_t *const buffer = &log_text_buffers[current_text_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_text_write_mutex);
		logger_health.full_buffer_moments++;
		logger_health.buffer_is_full = true;
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
		logger_health.region_overflow = true;
		return W_FAILURE;
	}

	// Get pointer to message region to write to
	char *const msg_dest = buffer->data + (msg_num * MAX_TEXT_MSG_LENGTH);

	// Determine the string to display for log severity
	const char *level_string = "";
	switch (level) {
		case LOG_LVL_FATAL:
			level_string = "FATAL";
			break;
		case LOG_LVL_WARN:
			level_string = "WARN";
			break;
		case LOG_LVL_INFO:
			level_string = "INFO";
			break;
		case LOG_LVL_DEBUG:
			level_string = "DEBUG";
			break;
		default:
			level_string = "LOG LEVEL ERROR";
			break;
	}

	uint32_t chars_written = 0;
	// Write log message header to region
	chars_written += snprintf_(msg_dest + chars_written,
							   MAX_TEXT_MSG_LENGTH - chars_written,
							   "[%" PRIu32 "] %s; %s; ",
							   timestamp,
							   level_string,
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
		if (xQueueSendToBack(text_buffers_queue, &buffer, 0) != pdPASS) {
			// This should never be reached as the queue should have space for all buffers
			logger_health.queue_send_fails++;
			logger_health.os_error_occurred = true;
			return W_FAILURE;
		}
	}
	// msg_num is guaranteed not to be greater than TEXT_MSGS_PER_BUFFER by condition before write

	return W_SUCCESS;
}

w_status_t log_data(uint32_t timeout, log_data_type_t type, const log_data_container_t *data) {
	// Get timestamp as close to time of call as possible
	// Use dummy timestamp of 0 if failed to get timestamp, as in log_text()
	uint32_t timestamp = 0;
	(void)timer_get_ms(&timestamp);

	if (!logger_health.is_init) {
		return W_FAILURE;
	}

	// Validate arguments
	if (NULL == data) {
		logger_health.null_param_count++;
		logger_health.invalid_param = true;
		return W_INVALID_PARAM;
	}

	// Attempt to acquire mutex to safely access current_data_buf_num
	if (xSemaphoreTake(log_data_write_mutex, pdMS_TO_TICKS(timeout)) != pdPASS) {
		logger_health.log_write_timeouts++;
		logger_health.timeout_occurred = true;
		return W_FAILURE;
	}

	// Get text buffer to write into
	log_buffer_t *const buffer = &log_data_buffers[current_data_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_data_write_mutex);
		logger_health.full_buffer_moments++;
		logger_health.buffer_is_full = true;
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
		logger_health.region_overflow = true;
		return W_FAILURE;
	}
	return res;
}

static sd_card_file_ctx_t text_file_ctx;
static sd_card_file_ctx_t data_file_ctx;

void log_task(void *argument) {
	(void)argument;

	log_buffer_t *buffer_to_print = NULL;

	// Open persistent streaming files.
	// Keep these open for the duration of logging to avoid FAT traversal overhead.
	// write the file names
	memcpy(text_file_ctx.filename, text_log_filename, sizeof(text_file_ctx.filename));
	memcpy(data_file_ctx.filename, data_log_filename, sizeof(data_file_ctx.filename));
	if ((sd_card_file_open(&text_file_ctx) != W_SUCCESS) ||
		(sd_card_file_open(&data_file_ctx) != W_SUCCESS)) {
		logger_health.crit_errs++;
	}

	uint32_t last_text_logging_time = 0;
	if (timer_get_ms(&last_text_logging_time) != W_SUCCESS) {
		last_text_logging_time = UINT32_MAX;
	}

	uint32_t cur_timestamps = 0;

	for (;;) {
		// this is kept in case any other issues require this fix
		// bool must_log_txt = (timer_get_ms(&cur_timestamps) == W_SUCCESS) &&
		// ((last_text_logging_time + MAX_TXT_LOGGING_PERIOD_MS) < cur_timestamps);

		bool must_log_txt = false;
		bool have_msg = false;
		uint32_t msgs_done = 0;
		uint32_t max_msgs = DATA_MSGS_PER_BUFFER;
		sd_card_file_ctx_t *file_ctx = &data_file_ctx;
		uint32_t log_write_try_count = 0;
		// check data
		if ((xQueueReceive(data_buffers_queue, &buffer_to_print, 100) == pdPASS) &&
			(!must_log_txt)) {
			log_write_try_count = DATA_WRITE_TRY_COUNT;
			have_msg = true;
		} else if (xQueueReceive(text_buffers_queue, &buffer_to_print, 10) == pdPASS) {
			log_write_try_count = TEXT_WRITE_TRY_COUNT;
			max_msgs = TEXT_MSGS_PER_BUFFER;
			file_ctx = &text_file_ctx;
			have_msg = true;

			if (timer_get_ms(&cur_timestamps) == W_SUCCESS) {
				last_text_logging_time = cur_timestamps;
			}
		}

		if (have_msg) {
			while (msgs_done < max_msgs) {
				if (xSemaphoreTake(buffer_to_print->msgs_done_semaphore, 10) == pdPASS) {
					msgs_done++;
				} else {
					logger_health.unsafe_buffer_flushes++;
					logger_health.os_error_occurred = true;
					break;
				}
			}

			// try several times to buffer to SD card
			uint32_t size = 0;
			for (uint32_t i = 0; i < log_write_try_count; i++) {
				uint32_t open_start_ms = 0;
				timer_get_ms(&open_start_ms);
				if (sd_card_file_write_open(
						file_ctx, buffer_to_print->data, LOG_BUFFER_SIZE, &size) == W_SUCCESS) {
					uint32_t open_end_ms = 0;
					timer_get_ms(&open_end_ms);
					/*
					 * Sync after every buffer for maximum data retention.
					 * Consider reducing this to periodic syncs once reliability
					 * is confirmed.
					 */
					uint32_t sync_start_ms = 0;
					timer_get_ms(&sync_start_ms);
					if (sd_card_file_sync(file_ctx) == W_SUCCESS) {
						uint32_t sync_end_ms = 0;
						timer_get_ms(&sync_end_ms);

						// REMOVE BEFORE FLIGHT
						log_text(10,
								 LOG_LVL_INFO,
								 "logger_write",
								 "open_time=%d, sync_time=%d, try=%d",
								 open_end_ms - open_start_ms,
								 sync_end_ms - sync_start_ms,
								 i + 1);
						gpio_toggle(GPIO_PIN_BLUE_LED, 0);
						break;
					} else {
						log_text(10,
								 LOG_LVL_WARN,
								 "logger",
								 "sd_card_file_sync failed on try %d",
								 i + 1);
					}
				}

				gpio_toggle(GPIO_PIN_RED_LED, 0);

				if ((log_write_try_count - 1) == i) {
					logger_health.buffer_flush_fails++;
					break; // Failed to write after all attempts
				} else {
					// Allow SD card time to recover.
					vTaskDelay(pdMS_TO_TICKS(2));
				}
			}

			// Reinitialize buffer for reuse
			log_reset_buffer(buffer_to_print);
		} else {
			logger_health.no_full_buf_moments++;
		}
	}
}

health_status_t logger_get_status(void) {
	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_LOGGER,
							  .error_bitfield = 0};

	if (!logger_health.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_NOT_INIT_OFFSET;
	}

	if (logger_health.buffer_is_full) {
		logger_health.buffer_is_full = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= (1 << CANARDS_MODULE_E_OVERFLOW_OFFSET);
	}

	if (logger_health.timeout_occurred) {
		logger_health.timeout_occurred = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= (1 << CANARDS_MODULE_E_TIMEOUT_OFFSET);
	}

	if (logger_health.os_error_occurred) {
		logger_health.os_error_occurred = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= (1 << CANARDS_MODULE_E_OS_OFFSET);
	}

	if (logger_health.invalid_param) {
		logger_health.invalid_param = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= (1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET);
	}

	if (logger_health.region_overflow) {
		logger_health.region_overflow = false;
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= (1 << CANARDS_MODULE_E_OUT_OF_BOUNDS_OFFSET);
	}

	log_text(10,
			 LOG_LVL_INFO,
			 "logger",
			 "init=%d, trunc=%d, full_buff=%d, log_w_timeouts=%d, invalid_region=%d",
			 logger_health.is_init,
			 logger_health.trunc_msgs,
			 logger_health.full_buffer_moments,
			 logger_health.log_write_timeouts,
			 logger_health.invalid_region_moments);

	log_text(10,
			 LOG_LVL_INFO,
			 "logger",
			 "q_fail=%d, no_buf=%d, flush_fail=%d, unsafe_fl=%d, nparam=%d",
			 logger_health.queue_send_fails,
			 logger_health.no_full_buf_moments,
			 logger_health.buffer_flush_fails,
			 logger_health.unsafe_buffer_flushes,
			 logger_health.null_param_count);

	return status;
}
