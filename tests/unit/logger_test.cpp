#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

DEFINE_FFF_GLOBALS;

extern "C" {
#include "FreeRTOS.h"
#include "application/fsm/fsm.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "rocketlib.h"
#include "semphr.h"
#include "task.h"

// FreeRTOS fakes (xQueueCreate, xSemaphoreTake, uxTaskGetTaskNumber, ...) come from tests/mocks

// timer fakes
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
// sd card fakes
FAKE_VALUE_FUNC(w_status_t, sd_card_file_read, const char *, char *, uint32_t, uint32_t *);
FAKE_VALUE_FUNC(w_status_t, sd_card_file_write, const char *, const char *, uint32_t, bool,
                uint32_t *);
FAKE_VALUE_FUNC(w_status_t, sd_card_file_create, const char *);
FAKE_VALUE_FUNC(w_status_t, sd_card_file_open, sd_card_file_ctx_t *);
FAKE_VALUE_FUNC(w_status_t, sd_card_file_write_open, sd_card_file_ctx_t *, const char *, uint32_t,
                uint32_t *);
FAKE_VALUE_FUNC(w_status_t, sd_card_file_sync, sd_card_file_ctx_t *);
// fsm fakes
FAKE_VALUE_FUNC(fsm_state_t, fsm_get_state);
// gpio fakes
FAKE_VALUE_FUNC(w_status_t, gpio_toggle, gpio_pin_t, uint32_t);
}

/*
 * NOTE: log.c keeps all of its state (init flag, buffers, health flags) in file-scope statics that
 * cannot be reset from a test. ctest (via gtest_discover_tests) runs every TEST_F in its own
 * process, so each test starts from a fresh logger. Running the test binary directly runs every
 * test in one process, in which case the tests that rely on a fresh logger may fail.
 */

// Mirror of the private log_error_msg_t in log.c, used to decode records from the error buffer
typedef struct {
    uint32_t timestamp;
    log_error_id_t log_error_id;
    log_task_id_t log_task_id;
    log_file_id_t log_file_id;
    uint16_t line_number;
    uint16_t counter;
    uint32_t args[LOG_ERROR_MAX_ARGS];
} test_error_msg_t;
static_assert(sizeof(test_error_msg_t) == MAX_ERROR_MSG_LENGTH, "must match log_error_msg_t");

// Dummy handles handed out by the FreeRTOS fakes. Queues are created in the order error, data, text
static int error_queue_dummy = 0;
static int data_queue_dummy = 0;
static int text_queue_dummy = 0;
static QueueHandle_t queue_handles[] = {&error_queue_dummy, &data_queue_dummy, &text_queue_dummy};
static int mutex_dummy = 0;
static int counting_sem_dummy = 0;

// Controllable fake state
static uint32_t mock_now_ms = 0;
static BaseType_t mutex_take_result = pdPASS;
static uint32_t mock_run_count = 0;
static w_status_t mock_run_count_read_status = W_FAILURE;

// The error buffer passed to xQueueSendToBack once it fills up (buffer data is at offset 0)
static const char *captured_error_buffer = nullptr;

// Timer custom function to get any time
static w_status_t timer_get_ms_custom(uint32_t *p_ms) {
    *p_ms = mock_now_ms;
    return W_SUCCESS;
}

// Semaphore custom
static BaseType_t semaphore_take_custom(SemaphoreHandle_t handle, TickType_t) {
    // log_reset_buffer() drains counting semaphores until a take fails, so always fail those
    if (handle == &counting_sem_dummy) {
        return pdFAIL;
    }
    return mutex_take_result;
}

static w_status_t sd_card_file_read_custom(const char *, char *buffer, uint32_t num_bytes,
                                           uint32_t *bytes_read) {
    if (W_SUCCESS == mock_run_count_read_status) {
        memcpy(buffer, &mock_run_count, sizeof(mock_run_count));
        *bytes_read = num_bytes;
    }
    return mock_run_count_read_status;
}

static BaseType_t queue_send_to_back_custom(QueueHandle_t queue, const void *item, TickType_t) {
    if (queue == &error_queue_dummy) {
        captured_error_buffer = *(const char *const *)item;
    }
    return pdPASS;
}

class LoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xQueueSendToBack);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreCreateCounting);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        RESET_FAKE(xTaskGetCurrentTaskHandle);
        RESET_FAKE(uxTaskGetTaskNumber);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(sd_card_file_read);
        RESET_FAKE(sd_card_file_write);
        RESET_FAKE(sd_card_file_create);
        RESET_FAKE(sd_card_file_open);
        RESET_FAKE(sd_card_file_write_open);
        RESET_FAKE(sd_card_file_sync);
        RESET_FAKE(fsm_get_state);
        RESET_FAKE(gpio_toggle);
        FFF_RESET_HISTORY();

        mock_now_ms = 1000;
        mutex_take_result = pdPASS;
        mock_run_count = 0;
        mock_run_count_read_status = W_FAILURE; // LOGRUN.BIN missing by default
        captured_error_buffer = nullptr;

        SET_RETURN_SEQ(xQueueCreate, queue_handles, 3);
        xQueueSendToBack_fake.custom_fake = queue_send_to_back_custom;
        xSemaphoreCreateMutex_fake.return_val = &mutex_dummy;
        xSemaphoreCreateCounting_fake.return_val = &counting_sem_dummy;
        xSemaphoreTake_fake.custom_fake = semaphore_take_custom;
        xSemaphoreGive_fake.return_val = pdPASS;
        uxTaskGetTaskNumber_fake.return_val = LOG_TASK_FSM;
        timer_get_ms_fake.custom_fake = timer_get_ms_custom;

        sd_card_file_read_fake.custom_fake = sd_card_file_read_custom;
        sd_card_file_write_fake.return_val = W_SUCCESS;
        sd_card_file_create_fake.return_val = W_SUCCESS;
    }

    // Call log_error() with the given site and first arg, keeping everything else fixed
    w_status_t log_error_with_arg(log_error_site_t *site, uint32_t arg0, uint16_t line = 42) {
        const uint32_t args[LOG_ERROR_MAX_ARGS] = {arg0, 0, 0, 0, 0};
        return log_error(site, 10, LOG_FILES_TELEMETRY_C, line, LOG_ERRORS_ERROR, args);
    }

    // Write unique records until the error buffer fills and is queued. Returns the queued buffer
    const char *fill_error_buffer() {
        log_error_site_t filler_site = {0};
        for (uint32_t i = 0; (i < ERROR_MSGS_PER_BUFFER) && (nullptr == captured_error_buffer);
             i++) {
            // unique arg each time so nothing aggregates
            log_error_with_arg(&filler_site, 0xF0000000u + i, 1);
        }
        return captured_error_buffer;
    }

    // Decode the record at msg_num from an error buffer
    static test_error_msg_t read_record(const char *buffer, uint16_t msg_num) {
        test_error_msg_t msg = {};
        memcpy(&msg, buffer + (msg_num * MAX_ERROR_MSG_LENGTH), sizeof(msg));
        return msg;
    }

    static bool has_error(const health_status_t &status, uint32_t offset) {
        return (status.error_bitfield & (1U << offset)) != 0;
    }
};

/* ---------------------------------------------------------------------------------------------
 * Before init
 * ------------------------------------------------------------------------------------------- */

// All log functions shall fail before log_init()
TEST_F(LoggerTest, LOG_FUNCTIONS_FAIL_BEFORE_INIT) {
    log_error_site_t site = {0};
    log_data_container_t data = {};

    EXPECT_EQ(W_FAILURE, log_error_with_arg(&site, 0));
    EXPECT_EQ(W_FAILURE, log_text(10, LOG_LVL_INFO, "test", "hello"));
    EXPECT_EQ(W_FAILURE, log_data(10, LOG_TYPE_TEST, &data));

    // nothing should have been written or locked
    EXPECT_EQ(0u, xSemaphoreTake_fake.call_count);
    EXPECT_EQ(0u, site.last_logged_version);
}

// logger_get_status() shall report NOT_INIT before log_init()
TEST_F(LoggerTest, STATUS_NOT_INIT_BEFORE_INIT) {
    health_status_t status = logger_get_status();

    EXPECT_EQ(CANARDS_MODULE_ID_LOGGER, status.module_id);
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_ERROR, status.severity);
    EXPECT_TRUE(has_error(status, CANARDS_MODULE_E_NOT_INIT_OFFSET));
}

/* ---------------------------------------------------------------------------------------------
 * log_init()
 * ------------------------------------------------------------------------------------------- */

// log_init() shall fail and report an OS error if a queue can't be created
TEST_F(LoggerTest, INIT_FAILS_WHEN_QUEUE_CREATE_FAILS) {
    QueueHandle_t handles[] = {&error_queue_dummy, NULL, &text_queue_dummy};
    SET_RETURN_SEQ(xQueueCreate, handles, 3);

    EXPECT_EQ(W_FAILURE, log_init());

    health_status_t status = logger_get_status();
    EXPECT_TRUE(has_error(status, CANARDS_MODULE_E_NOT_INIT_OFFSET));
    EXPECT_TRUE(has_error(status, CANARDS_MODULE_E_OS_OFFSET));
    // no files should be touched
    EXPECT_EQ(0u, sd_card_file_create_fake.call_count);
}

// log_init() shall fail and report an OS error if a mutex can't be created
TEST_F(LoggerTest, INIT_FAILS_WHEN_MUTEX_CREATE_FAILS) {
    xSemaphoreCreateMutex_fake.return_val = NULL;

    EXPECT_EQ(W_FAILURE, log_init());
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_OS_OFFSET));
}

// log_init() shall fail and report an OS error if a counting semaphore can't be created
TEST_F(LoggerTest, INIT_FAILS_WHEN_COUNTING_SEMAPHORE_CREATE_FAILS) {
    xSemaphoreCreateCounting_fake.return_val = NULL;

    EXPECT_EQ(W_FAILURE, log_init());
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_OS_OFFSET));
}

// With no LOGRUN.BIN, log_init() shall create it, start at run 1, and create the 3 log files
TEST_F(LoggerTest, INIT_CREATES_RUN_COUNT_AND_LOG_FILES) {
    EXPECT_EQ(W_SUCCESS, log_init());

    // LOGRUN.BIN + .ERR + .TXT + .BIN
    ASSERT_EQ(4u, sd_card_file_create_fake.call_count);
    EXPECT_STREQ("LOGRUN.BIN", sd_card_file_create_fake.arg0_history[0]);
    EXPECT_STREQ("00000001.ERR", sd_card_file_create_fake.arg0_history[1]);
    EXPECT_STREQ("00000001.TXT", sd_card_file_create_fake.arg0_history[2]);
    EXPECT_STREQ("00000001.BIN", sd_card_file_create_fake.arg0_history[3]);

    // run count 1 is written back to LOGRUN.BIN
    ASSERT_EQ(1u, sd_card_file_write_fake.call_count);
    EXPECT_STREQ("LOGRUN.BIN", sd_card_file_write_fake.arg0_val);
    EXPECT_EQ(sizeof(uint32_t), sd_card_file_write_fake.arg2_val);

    health_status_t status = logger_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_OK, status.severity);
    EXPECT_EQ(0u, status.error_bitfield);
}

// With an existing LOGRUN.BIN, log_init() shall increment the run count and use it for filenames
TEST_F(LoggerTest, INIT_INCREMENTS_EXISTING_RUN_COUNT) {
    mock_run_count = 0x1A;
    mock_run_count_read_status = W_SUCCESS;

    EXPECT_EQ(W_SUCCESS, log_init());

    // LOGRUN.BIN already exists so only the 3 log files are created
    ASSERT_EQ(3u, sd_card_file_create_fake.call_count);
    EXPECT_STREQ("0000001b.ERR", sd_card_file_create_fake.arg0_history[0]);
    EXPECT_STREQ("0000001b.TXT", sd_card_file_create_fake.arg0_history[1]);
    EXPECT_STREQ("0000001b.BIN", sd_card_file_create_fake.arg0_history[2]);
}

// log_init() shall return W_IO_ERROR and report a file system error if LOGRUN.BIN can't be created
TEST_F(LoggerTest, INIT_FAILS_WHEN_RUN_COUNT_FILE_CREATE_FAILS) {
    sd_card_file_create_fake.return_val = W_FAILURE;

    EXPECT_EQ(W_IO_ERROR, log_init());

    health_status_t status = logger_get_status();
    EXPECT_TRUE(has_error(status, CANARDS_MODULE_E_NOT_INIT_OFFSET));
    EXPECT_TRUE(has_error(status, CANARDS_MODULE_E_FILE_SYSTEM_OFFSET));
}

// log_init() shall fail and report a file system error if a log file can't be created
TEST_F(LoggerTest, INIT_FAILS_WHEN_LOG_FILE_CREATE_FAILS) {
    // LOGRUN.BIN create succeeds, .ERR create fails
    w_status_t create_results[] = {W_SUCCESS, W_FAILURE, W_SUCCESS, W_SUCCESS};
    SET_RETURN_SEQ(sd_card_file_create, create_results, 4);

    EXPECT_NE(W_SUCCESS, log_init());
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_FILE_SYSTEM_OFFSET));
}

// A second log_init() call shall succeed without re-creating anything
TEST_F(LoggerTest, INIT_IS_IDEMPOTENT) {
    ASSERT_EQ(W_SUCCESS, log_init());
    const unsigned queue_creates = xQueueCreate_fake.call_count;
    const unsigned file_creates = sd_card_file_create_fake.call_count;

    EXPECT_EQ(W_SUCCESS, log_init());
    EXPECT_EQ(queue_creates, xQueueCreate_fake.call_count);
    EXPECT_EQ(file_creates, sd_card_file_create_fake.call_count);
}

/* ---------------------------------------------------------------------------------------------
 * log_error()
 * ------------------------------------------------------------------------------------------- */

// log_error() shall reject a NULL site and report an invalid param
TEST_F(LoggerTest, LOG_ERROR_NULL_SITE_IS_INVALID) {
    ASSERT_EQ(W_SUCCESS, log_init());

    EXPECT_EQ(W_INVALID_PARAM, log_error_with_arg(NULL, 0));
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_INVALID_PARAM_OFFSET));
}

// log_error() shall fail and report a timeout if the mutex can't be taken in time
TEST_F(LoggerTest, LOG_ERROR_MUTEX_TIMEOUT) {
    ASSERT_EQ(W_SUCCESS, log_init());
    mutex_take_result = pdFAIL;
    log_error_site_t site = {0};
    const uint32_t args[LOG_ERROR_MAX_ARGS] = {0};

    EXPECT_EQ(W_FAILURE, log_error(&site, 25, LOG_FILES_TELEMETRY_C, 1, LOG_ERRORS_ERROR, args));

    // timeout is passed through to the mutex take
    EXPECT_EQ(&mutex_dummy, xSemaphoreTake_fake.arg0_val);
    EXPECT_EQ(pdMS_TO_TICKS(25), xSemaphoreTake_fake.arg1_val);
    // site untouched since nothing was written
    EXPECT_EQ(0u, site.last_logged_version);

    mutex_take_result = pdPASS;
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_TIMEOUT_OFFSET));
}

// The first log_error() from a site shall write a record and record where it went in the site
TEST_F(LoggerTest, LOG_ERROR_FIRST_CALL_UPDATES_SITE) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};
    const unsigned gives_before = xSemaphoreGive_fake.call_count;

    EXPECT_EQ(W_SUCCESS, log_error_with_arg(&site, 7));

    EXPECT_EQ(1u, site.last_logged_version); // buffer versions start at 1
    EXPECT_EQ(0u, site.last_logged_buffer);
    // write mutex released + msgs_done semaphore given
    EXPECT_EQ(gives_before + 2, xSemaphoreGive_fake.call_count);
    // the task number of the calling task is looked up
    EXPECT_EQ(1u, uxTaskGetTaskNumber_fake.call_count);
}

// Consecutive records from different sites shall go into consecutive regions
TEST_F(LoggerTest, LOG_ERROR_DIFFERENT_SITES_GET_CONSECUTIVE_REGIONS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site_a = {0};
    log_error_site_t site_b = {0};

    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site_a, 1));
    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site_b, 1));

    EXPECT_EQ(site_a.last_logged_msg_num + 1, site_b.last_logged_msg_num);
}

// The same site, task, and args within 1s shall aggregate instead of writing a new record
TEST_F(LoggerTest, LOG_ERROR_AGGREGATES_REPEATS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};
    log_error_site_t next_site = {0};

    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    const uint16_t first_msg_num = site.last_logged_msg_num;

    mock_now_ms += 1000; // exactly 1s later still aggregates
    EXPECT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    EXPECT_EQ(first_msg_num, site.last_logged_msg_num);

    // no region was used by the repeat, so the next new record goes right after the first
    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&next_site, 5));
    EXPECT_EQ(first_msg_num + 1, next_site.last_logged_msg_num);
}

// Different args shall not aggregate
TEST_F(LoggerTest, LOG_ERROR_NO_AGGREGATE_DIFFERENT_ARGS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};

    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    const uint16_t first_msg_num = site.last_logged_msg_num;

    EXPECT_EQ(W_SUCCESS, log_error_with_arg(&site, 6));
    EXPECT_EQ(first_msg_num + 1, site.last_logged_msg_num);
}

// More than 1s after the first record shall not aggregate
TEST_F(LoggerTest, LOG_ERROR_NO_AGGREGATE_AFTER_1S) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};

    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    const uint16_t first_msg_num = site.last_logged_msg_num;

    mock_now_ms += 1001;
    EXPECT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    EXPECT_EQ(first_msg_num + 1, site.last_logged_msg_num);
}

// The 1s window is measured from the first record, not from the latest repeat
TEST_F(LoggerTest, LOG_ERROR_AGGREGATE_WINDOW_FROM_FIRST_RECORD) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};

    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    const uint16_t first_msg_num = site.last_logged_msg_num;

    mock_now_ms += 600;
    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5)); // aggregates
    EXPECT_EQ(first_msg_num, site.last_logged_msg_num);

    mock_now_ms += 600; // 1200ms after the first record
    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5)); // new record
    EXPECT_EQ(first_msg_num + 1, site.last_logged_msg_num);
}

// The same site called from a different task shall not aggregate
TEST_F(LoggerTest, LOG_ERROR_NO_AGGREGATE_DIFFERENT_TASK) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};

    uxTaskGetTaskNumber_fake.return_val = LOG_TASK_FSM;
    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    const uint16_t first_msg_num = site.last_logged_msg_num;

    uxTaskGetTaskNumber_fake.return_val = LOG_TASK_TELEMETRY;
    EXPECT_EQ(W_SUCCESS, log_error_with_arg(&site, 5));
    EXPECT_EQ(first_msg_num + 1, site.last_logged_msg_num);
}

// Records shall hold the right fields, and repeats shall bump the counter of the original record
TEST_F(LoggerTest, LOG_ERROR_RECORD_CONTENTS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};
    const uint32_t args[LOG_ERROR_MAX_ARGS] = {0xDEADBEEF, (uint32_t)-3, 'c', 1, 0x12345678};

    mock_now_ms = 5000;
    uxTaskGetTaskNumber_fake.return_val = LOG_TASK_TELEMETRY;
    ASSERT_EQ(W_SUCCESS, log_error(&site, 10, LOG_FILES_TELEMETRY_C, 123,
                                   LOG_ERRORS_LOG_SD_CARD_FILE_SYNC_FAIL, args));
    // 2 repeats
    mock_now_ms = 5500;
    ASSERT_EQ(W_SUCCESS, log_error(&site, 10, LOG_FILES_TELEMETRY_C, 123,
                                   LOG_ERRORS_LOG_SD_CARD_FILE_SYNC_FAIL, args));
    ASSERT_EQ(W_SUCCESS, log_error(&site, 10, LOG_FILES_TELEMETRY_C, 123,
                                   LOG_ERRORS_LOG_SD_CARD_FILE_SYNC_FAIL, args));

    const char *buffer = fill_error_buffer();
    ASSERT_NE(nullptr, buffer);

    test_error_msg_t msg = read_record(buffer, site.last_logged_msg_num);
    EXPECT_EQ(5000u, msg.timestamp); // timestamp of the first occurrence
    EXPECT_EQ(LOG_ERRORS_LOG_SD_CARD_FILE_SYNC_FAIL, msg.log_error_id);
    EXPECT_EQ(LOG_TASK_TELEMETRY, msg.log_task_id);
    EXPECT_EQ(LOG_FILES_TELEMETRY_C, msg.log_file_id);
    EXPECT_EQ(123u, msg.line_number);
    EXPECT_EQ(2u, msg.counter); // counter is extra repeats, first occurrence is 0
    for (int i = 0; i < LOG_ERROR_MAX_ARGS; i++) {
        EXPECT_EQ(args[i], msg.args[i]);
    }
}

// The repeat counter shall saturate at UINT16_MAX instead of wrapping
TEST_F(LoggerTest, LOG_ERROR_COUNTER_SATURATES) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};

    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 9));
    for (uint32_t i = 0; i < (uint32_t)UINT16_MAX + 10; i++) {
        ASSERT_EQ(W_SUCCESS, log_error_with_arg(&site, 9));
    }

    const char *buffer = fill_error_buffer();
    ASSERT_NE(nullptr, buffer);
    EXPECT_EQ(UINT16_MAX, read_record(buffer, site.last_logged_msg_num).counter);
}

// Filling the error buffer shall queue it once, then reject new records until it's flushed
TEST_F(LoggerTest, LOG_ERROR_FULL_BUFFER) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t old_site = {0};
    ASSERT_EQ(W_SUCCESS, log_error_with_arg(&old_site, 1));

    ASSERT_NE(nullptr, fill_error_buffer());
    EXPECT_EQ(1u, xQueueSendToBack_fake.call_count);
    EXPECT_EQ(&error_queue_dummy, xQueueSendToBack_fake.arg0_val);

    // a brand new record is rejected since the buffer is full
    log_error_site_t new_site = {0};
    EXPECT_EQ(W_FAILURE, log_error_with_arg(&new_site, 2));

    // the old site can't aggregate either: the buffer version was bumped when it filled
    EXPECT_EQ(W_FAILURE, log_error_with_arg(&old_site, 1));

    health_status_t status = logger_get_status();
    EXPECT_TRUE(has_error(status, CANARDS_MODULE_E_OVERFLOW_OFFSET));
}

/* ---------------------------------------------------------------------------------------------
 * log_text() / log_data()
 * ------------------------------------------------------------------------------------------- */

// log_text() shall succeed after init and release its mutex
TEST_F(LoggerTest, LOG_TEXT_SUCCESS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    const unsigned takes_before = xSemaphoreTake_fake.call_count;
    const unsigned gives_before = xSemaphoreGive_fake.call_count;

    EXPECT_EQ(W_SUCCESS, log_text(10, LOG_LVL_WARN, "test", "value=%d", 5));

    // text write mutex taken once, then released + msgs_done semaphore given
    EXPECT_EQ(takes_before + 1, xSemaphoreTake_fake.call_count);
    EXPECT_EQ(&mutex_dummy, xSemaphoreTake_fake.arg0_val);
    EXPECT_EQ(gives_before + 2, xSemaphoreGive_fake.call_count);
}

// log_text() shall reject NULL source or format
TEST_F(LoggerTest, LOG_TEXT_NULL_PARAMS) {
    ASSERT_EQ(W_SUCCESS, log_init());

    EXPECT_EQ(W_INVALID_PARAM, log_text(10, LOG_LVL_INFO, NULL, "hello"));
    EXPECT_EQ(W_INVALID_PARAM, log_text(10, LOG_LVL_INFO, "test", NULL));
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_INVALID_PARAM_OFFSET));
}

// log_text() shall fail and report a timeout if the mutex can't be taken in time
TEST_F(LoggerTest, LOG_TEXT_MUTEX_TIMEOUT) {
    ASSERT_EQ(W_SUCCESS, log_init());
    mutex_take_result = pdFAIL;

    EXPECT_EQ(W_FAILURE, log_text(10, LOG_LVL_INFO, "test", "hello"));

    mutex_take_result = pdPASS;
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_TIMEOUT_OFFSET));
}

// log_data() shall succeed after init
TEST_F(LoggerTest, LOG_DATA_SUCCESS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_data_container_t data = {};
    data.test.test_val = 1.5f;

    EXPECT_EQ(W_SUCCESS, log_data(10, LOG_TYPE_TEST, &data));
}

// log_data() shall reject NULL data
TEST_F(LoggerTest, LOG_DATA_NULL_PARAM) {
    ASSERT_EQ(W_SUCCESS, log_init());

    EXPECT_EQ(W_INVALID_PARAM, log_data(10, LOG_TYPE_TEST, NULL));
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_INVALID_PARAM_OFFSET));
}

/* ---------------------------------------------------------------------------------------------
 * logger_get_status()
 * ------------------------------------------------------------------------------------------- */

// Transient error flags shall clear after being reported once
TEST_F(LoggerTest, STATUS_CLEARS_TRANSIENT_FLAGS) {
    ASSERT_EQ(W_SUCCESS, log_init());
    log_error_site_t site = {0};

    mutex_take_result = pdFAIL;
    EXPECT_EQ(W_FAILURE, log_error_with_arg(&site, 0));
    mutex_take_result = pdPASS;
    EXPECT_EQ(W_INVALID_PARAM, log_error_with_arg(NULL, 0));

    health_status_t first = logger_get_status();
    EXPECT_TRUE(has_error(first, CANARDS_MODULE_E_TIMEOUT_OFFSET));
    EXPECT_TRUE(has_error(first, CANARDS_MODULE_E_INVALID_PARAM_OFFSET));

    health_status_t second = logger_get_status();
    EXPECT_EQ(CANARDS_HEALTH_SEVERITY_HEALTH_OK, second.severity);
    EXPECT_EQ(0u, second.error_bitfield);
}

// Init failure flags shall keep reporting on every status check
TEST_F(LoggerTest, STATUS_KEEPS_INIT_FAILURE_FLAGS) {
    xSemaphoreCreateMutex_fake.return_val = NULL;
    ASSERT_EQ(W_FAILURE, log_init());

    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_OS_OFFSET));
    EXPECT_TRUE(has_error(logger_get_status(), CANARDS_MODULE_E_OS_OFFSET));
}
