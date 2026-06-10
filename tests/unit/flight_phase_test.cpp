#include "fff.h"
#include <gtest/gtest.h>

DEFINE_FFF_GLOBALS;

extern "C" {
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/fsm/fsm.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "timers.h"

// FAKES
// w_status_t log_init(void)
FAKE_VALUE_FUNC(w_status_t, log_init);
//  w_status_t log_text(const char *source, const char *format, ...)
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
// w_status_t log_data(uint32_t id, log_data_type_t type, const log_data_container_t *data)
FAKE_VALUE_FUNC(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
// w_status_t can_handler_register_callback(can_msg_type_t msg_type, can_callback_t callback)
FAKE_VALUE_FUNC(w_status_t, can_handler_register_callback, can_msg_type_t, can_callback_t)

FAKE_VALUE_FUNC(w_status_t, get_actuator_id, const can_msg_t *, can_actuator_id_t *)

// w_status_t get_cmd_actuator_state(const can_msg_t *msg, can_actuator_state_t *cmd_actuator_state);
FAKE_VALUE_FUNC(w_status_t, get_cmd_actuator_state, const can_msg_t *, can_actuator_state_t *)

BaseType_t
xQueuePeek_state_pad(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait) {
    fsm_state_t *p = (fsm_state_t *)pvBuffer;
    *p = STATE_IDLE;
    return pdPASS;
}


}

flight_phase_event_t queue_send_event = EVENT_NONE;
BaseType_t xQueueSend_check_input_event(QueueHandle_t xQueue, const void * pvItemToQueue, TickType_t xTicksToWait) {
    queue_send_event = *((flight_phase_event_t*)pvItemToQueue);
    return pdPASS;
}

class FlightPhaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(xQueueSend);
        RESET_FAKE(xQueuePeek);
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xTimerCreate);
        RESET_FAKE(xQueueOverwrite);
        RESET_FAKE(log_text);
        RESET_FAKE(can_handler_register_callback);
        RESET_FAKE(get_actuator_id);
        RESET_FAKE(get_cmd_actuator_state);
        RESET_FAKE(timer_get_ms);
        FFF_RESET_HISTORY();
        queue_send_event = EVENT_NONE;
    }

    void TearDown() override {}
};

// flight_phase_init should pass when valid handles are created/callback is registered
TEST_F(FlightPhaseTest, InitCreatesMutexes) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    xTimerCreate_fake.return_val = (TimerHandle_t)1;
    can_handler_register_callback_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, InitFailsIfMutexCreationFails) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)NULL;
    can_handler_register_callback_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, InitFailsIfRegistrationFails) {
    // // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    xTimerCreate_fake.return_val = (TimerHandle_t)1;
    can_handler_register_callback_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, SendEventSucceeds) {
    // Arrange
    xQueueSend_fake.return_val = pdPASS;

    // Act
    w_status_t status = flight_phase_send_event(EVENT_PAD_FILTER);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, SendEventFails) {
    // Arrange
    xQueueSend_fake.return_val = pdFAIL;

    // Act
    w_status_t status = flight_phase_send_event(EVENT_PAD_FILTER);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// TODO: update launch and act allowed timestamps
TEST_F(FlightPhaseTest, UpdateFailAsInvalidCtxPtr) {
    // Arrange
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_PAD_FILTER, curr_state, NULL);

    // Assert
    EXPECT_EQ(new_state, STATE_IDLE);
}

TEST_F(FlightPhaseTest, IdleToPadfilter) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_PAD_FILTER, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_PAD_FILTER);
}

TEST_F(FlightPhaseTest, IdleToIdle) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_INJ_OPEN, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_IDLE);
}

TEST_F(FlightPhaseTest, IdleToIdle2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_IDLE);
}

TEST_F(FlightPhaseTest, PadfilterToPadNav) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_FILTER;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_IGNITOR, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_PAD_NAV);
}

TEST_F(FlightPhaseTest, PadfilterToBoostLaunchAccel) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_FILTER;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_LAUNCH_ACCEL, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(timer_get_ms_fake.arg0_history[0], &(ctx.launch_timestamp_ms)); // check correct ptr
}

TEST_F(FlightPhaseTest, PadfilterToBoostInjOpen) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_FILTER;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_INJ_OPEN, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(timer_get_ms_fake.arg0_history[0], &(ctx.launch_timestamp_ms)); // check correct ptr
}

TEST_F(FlightPhaseTest, PadfilterToPadfilter) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_FILTER;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_PAD_FILTER, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_PAD_FILTER);
}

TEST_F(FlightPhaseTest, PadfilterToPadfilter2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_FILTER;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_PAD_FILTER);
}

TEST_F(FlightPhaseTest, PadNavToBoostLaunchAccel) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_NAV;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_LAUNCH_ACCEL, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(timer_get_ms_fake.arg0_history[0], &(ctx.launch_timestamp_ms)); // check correct ptr
}

TEST_F(FlightPhaseTest, PadNavToBoostInjOpen) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_NAV;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_INJ_OPEN, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(timer_get_ms_fake.arg0_history[0], &(ctx.launch_timestamp_ms)); // check correct ptr
}

TEST_F(FlightPhaseTest, PadNavToPadNav) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_NAV;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_IGNITOR, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_PAD_NAV);
}

TEST_F(FlightPhaseTest, PadNavToPadNav2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_PAD_NAV;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_PAD_NAV);
}

TEST_F(FlightPhaseTest, BoostToActAllowed) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_ACT_ALLOWED);
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(timer_get_ms_fake.arg0_history[0], &(ctx.act_allowed_timestamp_ms)); // check correct ptr
}

TEST_F(FlightPhaseTest, BoostToBoostTimerReset) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_INJ_OPEN, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(timer_get_ms_fake.arg0_history[0], &(ctx.launch_timestamp_ms)); // check correct ptr
}

TEST_F(FlightPhaseTest, BoostToBoost) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_LAUNCH_ACCEL, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
    EXPECT_EQ(timer_get_ms_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, BoostToBoost2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_RECOVERY_START, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_BOOST);
}

TEST_F(FlightPhaseTest, ActAllowedToRecovery) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_ACT_ALLOWED;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_RECOVERY_START, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_RECOVERY);
}

TEST_F(FlightPhaseTest, ActAllowedToActAllowed) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_ACT_ALLOWED;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_ACT_ALLOWED);
}

TEST_F(FlightPhaseTest, ActAllowedToActAllowed2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_ACT_ALLOWED;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_SLEEP_START, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_ACT_ALLOWED);
}

TEST_F(FlightPhaseTest, RecoveryToSleepy) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_RECOVERY;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_SLEEP_START, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_SLEEPY);
}

TEST_F(FlightPhaseTest, RecoveryToRecovery) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_RECOVERY;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_RECOVERY_START, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_RECOVERY);
}

TEST_F(FlightPhaseTest, RecoveryToRecovery2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_RECOVERY;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_PAD_FILTER, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_RECOVERY);
}

TEST_F(FlightPhaseTest, SleepyToSleepy) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_SLEEPY;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_SLEEP_START, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_SLEEPY);
}

TEST_F(FlightPhaseTest, SleepyToSleepy2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_SLEEPY;

    // Act
    fsm_state_t new_state = flight_phase_update_state(EVENT_PAD_FILTER, curr_state, &ctx);

    // Assert
    EXPECT_EQ(new_state, STATE_SLEEPY);
}

TEST_F(FlightPhaseTest, GenSyncNoEventIdle) {
    // this is done to allow for any state timer base transition
    // TODO: add sensor base transition criteria as well
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 0,
        .act_allowed_timestamp_ms = 0,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_IDLE;
    uint32_t timestamp_ms = UINT32_MAX; // should trigger any timer base transition
    all_sensors_data_t sensor_data = {0}; // edit to launch detection capable

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 0);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 0);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, GenSyncNoEventPadFilter) {
    // this is done to allow for any state timer base transition
    // TODO: add sensor base transition criteria as well
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 0,
        .act_allowed_timestamp_ms = 0,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_PAD_FILTER;
    uint32_t timestamp_ms = UINT32_MAX; // should trigger any timer base transition
    all_sensors_data_t sensor_data = {0}; // edit to launch detection capable

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 0);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 0);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, GenSyncNoEventPadNav) {
    // this is done to allow for any state timer base transition
    // TODO: add sensor base transition criteria as well
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 0,
        .act_allowed_timestamp_ms = 0,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_PAD_NAV;
    uint32_t timestamp_ms = UINT32_MAX; // should trigger any timer base transition
    all_sensors_data_t sensor_data = {0}; // edit to launch detection capable

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 0);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 0);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, GenSyncNoEventBoost) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = UINT32_MAX,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_BOOST;
    uint32_t timestamp_ms = 16000;
    all_sensors_data_t sensor_data = {0}; 

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, UINT32_MAX);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, GenSyncTimerEventActDelayBoost) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = UINT32_MAX,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_BOOST;
    uint32_t timestamp_ms = 17001;
    all_sensors_data_t sensor_data = {0}; 

    xQueueSend_fake.custom_fake = xQueueSend_check_input_event;

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, UINT32_MAX);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    EXPECT_EQ(xQueueSend_fake.call_count, 1);
    EXPECT_EQ(queue_send_event, EVENT_ACT_DELAY_ELAPSED);
}

TEST_F(FlightPhaseTest, GenSyncNoEventActAllowed) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = 17000,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_ACT_ALLOWED;
    uint32_t timestamp_ms = 290000;
    all_sensors_data_t sensor_data = {0}; 

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 17000);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, GenSyncTimerEventRecoActAllowed) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = 17000,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_ACT_ALLOWED;
    uint32_t timestamp_ms = (5*60000) + 10000 + 1; // after 5 minutes
    all_sensors_data_t sensor_data = {0}; 

    xQueueSend_fake.custom_fake = xQueueSend_check_input_event;

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 17000);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    EXPECT_EQ(xQueueSend_fake.call_count, 1);
    EXPECT_EQ(queue_send_event, EVENT_RECOVERY_START);
}

TEST_F(FlightPhaseTest, GenSyncNoEventRecov) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = 17000,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_RECOVERY;
    uint32_t timestamp_ms = 1500000;
    all_sensors_data_t sensor_data = {0}; 

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 17000);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}

TEST_F(FlightPhaseTest, GenSyncTimerEventSleepyRecov) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = 17000,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_RECOVERY;
    uint32_t timestamp_ms = (1600000) + 10000 + 1; // after 5 minutes
    all_sensors_data_t sensor_data = {0}; 

    xQueueSend_fake.custom_fake = xQueueSend_check_input_event;

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 17000);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    EXPECT_EQ(xQueueSend_fake.call_count, 1);
    EXPECT_EQ(queue_send_event, EVENT_SLEEP_START);
}

TEST_F(FlightPhaseTest, GenSyncNoEventSleepy) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 0,
        .act_allowed_timestamp_ms = 0,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_SLEEPY;
    uint32_t timestamp_ms = UINT32_MAX;
    all_sensors_data_t sensor_data = {0}; 

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 0);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, 0);
    EXPECT_EQ(ctx.num_consec_detections, 0);

    // no events sent
    EXPECT_EQ(xQueueSend_fake.call_count, 0);
}


TEST_F(FlightPhaseTest, GenSyncFail) {
    flight_phase_ctx_t ctx = {
        .launch_timestamp_ms = 10000,
        .act_allowed_timestamp_ms = UINT32_MAX,
        .num_consec_detections = 0
    };

    fsm_state_t curr_state = STATE_BOOST;
    uint32_t timestamp_ms = 17001;
    all_sensors_data_t sensor_data = {0}; 

    xQueueSend_fake.return_val = pdFAIL;

    w_status_t status = flight_phase_gen_sync_events(&ctx, curr_state, timestamp_ms, &sensor_data);

    // Assert
    EXPECT_EQ(status, W_FAILURE);

    // no changes to ctx
    EXPECT_EQ(ctx.launch_timestamp_ms, 10000);
    EXPECT_EQ(ctx.act_allowed_timestamp_ms, UINT32_MAX);
    EXPECT_EQ(ctx.num_consec_detections, 0);
}