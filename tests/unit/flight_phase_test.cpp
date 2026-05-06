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

FAKE_VALUE_FUNC(int, get_actuator_id, const can_msg_t *)

// int get_cmd_actuator_state(const can_msg_t *msg);
FAKE_VALUE_FUNC(int, get_cmd_actuator_state, const can_msg_t *)

BaseType_t
xQueuePeek_state_pad(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait) {
    fsm_state_t *p = (fsm_state_t *)pvBuffer;
    *p = STATE_IDLE;
    return pdPASS;
}
}

class FlightPhaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(xQueueSend);
        RESET_FAKE(xQueuePeek);
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xTimerCreate);
        RESET_FAKE(xQueueOverwrite);
        RESET_FAKE(log_text)
        RESET_FAKE(can_handler_register_callback)
        RESET_FAKE(get_actuator_id)
        RESET_FAKE(get_cmd_actuator_state)
        FFF_RESET_HISTORY();
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
    w_status_t status = flight_phase_send_event(EVENT_ESTIMATOR_INIT);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, SendEventFails) {
    // Arrange
    xQueueSend_fake.return_val = pdFAIL;

    // Act
    w_status_t status = flight_phase_send_event(EVENT_ESTIMATOR_INIT);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, ResetSendsResetEvent) {
    // Arrange
    xQueueSend_fake.return_val = pdPASS;

    // Act
    w_status_t status = flight_phase_reset();

    // Assert
    EXPECT_EQ(xQueueSend_fake.call_count, 1);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, UpdateFailAsInvalidStatePtr) {
    // Arrange
    flight_phase_ctx_t ctx = {0};

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, NULL, &ctx);

    // Assert
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(FlightPhaseTest, UpdateFailAsInvalidCtxPtr) {
    // Arrange
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, &curr_state, NULL);

    // Assert
    EXPECT_EQ(curr_state, STATE_IDLE);
    EXPECT_EQ(status, W_INVALID_PARAM);
}

TEST_F(FlightPhaseTest, IdleToPadfilter) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_SE_INIT);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, IdleToBoost) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_INJ_OPEN, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_BOOST);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, IdleToIdle) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_IDLE;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_IDLE);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, PadfilterToBoost) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_SE_INIT;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_INJ_OPEN, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_BOOST);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, PadfilterToPadfilter) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_SE_INIT;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_SE_INIT);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, PadfilterToPadfilter2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_SE_INIT;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_SE_INIT);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, BoostToActAllowed) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ACT_DELAY_ELAPSED, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_ACT_ALLOWED);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, BoostToRecovery) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_FLIGHT_ELAPSED, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_RECOVERY);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, BoostToBoost) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_INJ_OPEN, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_BOOST);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, BoostToBoost2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_BOOST;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_BOOST);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, ActAllowedToRecovery) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_ACT_ALLOWED;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_FLIGHT_ELAPSED, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_RECOVERY);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, ActAllowedToActAllowed) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_ACT_ALLOWED;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_ACT_ALLOWED);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, ActAllowedToActAllowed2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_ACT_ALLOWED;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_INJ_OPEN, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_ACT_ALLOWED);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, RecoveryToRecovery) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_RECOVERY;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_INJ_OPEN, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_RECOVERY);
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, RecoveryToRecovery2) {
    // Arrange
    flight_phase_ctx_t ctx = {0};
    fsm_state_t curr_state = STATE_RECOVERY;

    // Act
    w_status_t status = flight_phase_update_state(EVENT_ESTIMATOR_INIT, &curr_state, &ctx);

    // Assert
    EXPECT_EQ(curr_state, STATE_RECOVERY);
    EXPECT_EQ(status, W_SUCCESS);
}
