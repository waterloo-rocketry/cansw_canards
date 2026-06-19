// #include "fff.h"
// #include <gtest/gtest.h>

// #include "utils/mock_helpers.hpp"

// extern "C" {
// #include "FreeRTOS.h"
// #include "application/can_handler/can_handler.h"
// #include "application/controller/controller.h"
// #include "application/estimator/estimator_types.h"
// #include "application/estimator/estimator_module.h"
// #include "application/estimator/pad_filter.h"
// #include "application/fsm/fsm_types.h"
// #include "application/logger/log.h"
// #include "canlib.h"
// #include "drivers/timer/timer.h"
// #include "queue.h"
// #include "task.h"
// #include "third_party/rocketlib/include/common.h"

// DEFINE_FFF_GLOBALS;
// }

// // Define Fakes
// // flight_phase_state_t flight_phase_get_state(void);
// FAKE_VALUE_FUNC(flight_phase_state_t, flight_phase_get_state);
// // bool get_analog_data(const can_msg_t *msg, can_analog_sensor_id_t *sensor_id, uint16_t *value);
// FAKE_VALUE_FUNC(bool, get_analog_data, const can_msg_t *, can_analog_sensor_id_t *, uint16_t *);
// FAKE_VOID_FUNC(proc_handle_fatal_error, const char *);
// // w_status_t can_handler_transmit(const can_msg_t *msg);
// FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
// FAKE_VALUE_FUNC(w_status_t, flight_phase_get_act_allowed_ms, uint32_t *);
// FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
// FAKE_VALUE_FUNC(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);

// TODO: add unit test for these new can functions
// FAKE_VALUE_FUNC(w_status_t, can_encode_scaled_float, can_scaling_types_t, float32_t, void*);
// FAKE_VOID_FUNC(build_analog_sensor_16bit_msg, can_msg_prio_t, uint16_t, can_analog_sensor_id_t, uint16_t, can_msg_t *);

// // TODO: confirm with Finn and Tristan with actual tolerance
// static double CMD_TOLERANCE_RAD = M_PI / (180.0 * 1000.0); // this is based on the error being a single millidegree since rounding to integer

// static uint32_t test_act_allowed_ms_value = 0;

// // uint16_t rad_to_can_cmd(float rad) {
// //     // Convert radians to millidegrees and shift to unsigned 16-bit
// //     int16_t res_int16 = (int16_t)(rad / M_PI * 180.0 * 1000.0) + 32768;
// //     return (uint16_t)(res_int16); 
// // }

// class ControllerTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//         RESET_FAKE(flight_phase_get_state);
//         RESET_FAKE(get_analog_data);
//         RESET_FAKE(proc_handle_fatal_error);
//         RESET_FAKE(can_handler_transmit);
//         RESET_FAKE(log_text);
//         RESET_FAKE(log_data);
//         RESET_FAKE(flight_phase_get_act_allowed_ms);
//         RESET_FAKE(xQueueReceive);
//         FFF_RESET_HISTORY();
//         // default to everything passing
//         xQueueReceive_fake.return_val = pdTRUE;
//         xQueueReceive_fake.custom_fake = xQueueReceive_custom;
//         can_handler_transmit_fake.return_val = W_SUCCESS;
//         // Register the custom fake for flight_phase_get_act_allowed_ms
//         flight_phase_get_act_allowed_ms_fake.custom_fake = flight_phase_get_act_allowed_custom;
//         flight_phase_get_act_allowed_ms_fake.return_val = W_SUCCESS;
//         test_act_allowed_ms_value = 0;
//     }

//     void TearDown() override {}
// };

// TEST_F(ControllerTest, Init) {
//     xQueueCreate_fake.return_val = (QueueHandle_t)1;
//     w_status_t res = controller_init();
//     EXPECT_EQ(res, W_SUCCESS);
// }

// TEST_F(ControllerTest, RunLoopPadPhase) {
//     // Arrange
//     flight_phase_get_state_fake.return_val = STATE_IDLE;

//     // Act
//     w_status_t actual_res = controller_run_loop();

//     // Assert
//     EXPECT_EQ(actual_res, W_SUCCESS);
//     EXPECT_FALSE(ctx.cmd_output.cmd_updated);
//     EXPECT_EQ(log_text_fake.call_count, 0);
// }

// TEST_F(ControllerTest, StepPadFilterPhase) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_SE_INIT;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_SUCCESS);
//     EXPECT_FALSE(ctx.cmd_output.cmd_updated);
//     EXPECT_EQ(log_text_fake.call_count, 0);
// }

// TEST_F(ControllerTest, StepDataMiss) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = false}};
//     const fsm_state_t curr_fsm_state = STATE_ACT_ALLOWED;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_FAILURE);
//     EXPECT_FALSE(ctx.cmd_output.cmd_updated);
// }

// TEST_F(ControllerTest, StepBoostButNotActAllowed) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_BOOST;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_SUCCESS);
//     EXPECT_FALSE(ctx.cmd_output.cmd_updated);
//     EXPECT_EQ(log_text_fake.call_count, 0);
// }

// TEST_F(ControllerTest, StepActAllowedButTimeWrong) {
//     // Arrange
//     test_act_allowed_ms_value = 2000;

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_ACT_ALLOWED;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_FAILURE);
//     // expect no cmd cuz failure
//     EXPECT_FALSE(ctx.cmd_output.cmd_updated);
// }

// /**
//  * for the successful actuation test cases, numbers come from controller_module_test.cpp
//  */

// TEST_F(ControllerTest, StepActAllowedStep1) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_ACT_ALLOWED;

//     test_act_allowed_ms_value = 7005;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     ctx.new_input_state.roll_state.roll_angle = 0.16345;
//     ctx.new_input_state.roll_state.roll_rate = 0.154534;
//     ctx.new_input_state.roll_state.canard_angle = 0.000134;
//     ctx.new_input_state.canard_coeff = 0.55234;
//     ctx.new_input_state.pressure_dynamic = 12093;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_SUCCESS);
//     EXPECT_EQ(can_handler_transmit_fake.call_count, 1);
//     // TODO redefine how to test the outputs to match the expected output
// }

// TEST_F(ControllerTest, StepActAllowedStep2) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_ACT_ALLOWED;

//     test_act_allowed_ms_value = 12034;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     ctx.new_input_state.roll_state.roll_angle = 0.16345;
//     ctx.new_input_state.roll_state.roll_rate = 0.154534;
//     ctx.new_input_state.roll_state.canard_angle = 0.000134;
//     ctx.new_input_state.canard_coeff = 0.55234;
//     ctx.new_input_state.pressure_dynamic = 12093;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_SUCCESS);
//     EXPECT_EQ(can_handler_transmit_fake.call_count, 1);
//     // TODO redefine how to test the outputs to match the expected output
// }

// TEST_F(ControllerTest, StepActAllowedOutOfBounds) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_ACT_ALLOWED;

//     test_act_allowed_ms_value = 15034;
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     ctx.new_input_state.roll_state.roll_angle = 0.16345;
//     ctx.new_input_state.roll_state.roll_rate = 0.154534;
//     ctx.new_input_state.roll_state.canard_angle = 0.000134;
//     ctx.new_input_state.canard_coeff = 0.55234;
//     ctx.new_input_state.pressure_dynamic = 709097;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_FAILURE);
//     // expect no cmd if interp fails
//     EXPECT_FALSE(ctx.cmd_output.cmd_updated);
// }

// // expect controller to compute and run during recovery phase too
// TEST_F(ControllerTest, StepRecovery) {
//     // Arrange

//     // initalize context
//     controller_ctx_t ctx = {.cmd_output = {.cmd_updated = false}, .new_input_state = {.state_updated = true}};
//     const fsm_state_t curr_fsm_state = STATE_RECOVERY;

//     test_act_allowed_ms_value = 12034; // step 2
//     const uint32_t act_allowed_timestamp_ms = 1000;
//     const uint32_t curr_timestamp_ms = test_act_allowed_ms_value + act_allowed_timestamp_ms;

//     ctx.new_input_state.roll_state.roll_angle = 0.16345;
//     ctx.new_input_state.roll_state.roll_rate = 0.154534;
//     ctx.new_input_state.roll_state.canard_angle = 0.000134;
//     ctx.new_input_state.canard_coeff = 0.55234;
//     ctx.new_input_state.pressure_dynamic = 12093;

//     // Act
//     w_status_t actual_res = controller_step(&ctx, curr_fsm_state, act_allowed_timestamp_ms, curr_timestamp_ms);

//     // Assert
//     EXPECT_EQ(actual_res, W_SUCCESS);
//     EXPECT_EQ(can_handler_transmit_fake.call_count, 1);
//     // TODO redefine how to test the outputs to match the expected output
// }
