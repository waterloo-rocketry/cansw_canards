/**
 * orientation correction from matlab commit e20e5d1
 */

#include "fff.h"
#include "utils/math_testing_helpers.hpp"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/estimator/estimator.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"
#include "task.h"
#include "third_party/rocketlib/include/common.h"

// Forward declare imu_handler_run
extern w_status_t imu_handler_run(uint32_t loop_count);

// Define all fake functions for IMUs using FFF
FAKE_VALUE_FUNC(w_status_t, altimu_init);
// FAKE_VALUE_FUNC(w_status_t, altimu_get_acc_data, vector3d_t *, altimu_raw_imu_data_t *);
// FAKE_VALUE_FUNC(w_status_t, altimu_get_gyro_data, vector3d_t *, altimu_raw_imu_data_t *);
FAKE_VALUE_FUNC(
    w_status_t, altimu_get_gyro_acc_data, vector3d_t *, vector3d_t *, altimu_raw_imu_data_t *,
    altimu_raw_imu_data_t *
);
FAKE_VALUE_FUNC(w_status_t, altimu_get_mag_data, vector3d_t *, altimu_raw_imu_data_t *);
FAKE_VALUE_FUNC(
    w_status_t, altimu_get_baro_data, altimu_barometer_data_t *, altimu_raw_baro_data_t *
);
FAKE_VALUE_FUNC(w_status_t, altimu_check_sanity);

FAKE_VALUE_FUNC(w_status_t, movella_init);
FAKE_VALUE_FUNC(w_status_t, movella_get_data, movella_data_t *, uint32_t);

FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
FAKE_VALUE_FUNC(w_status_t, estimator_init);
FAKE_VALUE_FUNC(w_status_t, estimator_update_imu_data, estimator_all_imus_input_t *);

// Fakes for logging
FAKE_VALUE_FUNC(w_status_t, log_init);
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);
FAKE_VALUE_FUNC(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);
FAKE_VOID_FUNC(log_task, void *);

// fake can stuff
// w_status_t can_handler_transmit(const can_msg_t *msg);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
FAKE_VALUE_FUNC(
    bool, build_baro_data_msg, can_msg_prio_t, uint16_t, can_imu_id_t, uint32_t, uint16_t,
    can_msg_t *
);
FAKE_VALUE_FUNC(
    bool, build_imu_data_msg, can_msg_prio_t, uint16_t, char, can_imu_id_t, uint16_t, uint16_t,
    can_msg_t *
);
FAKE_VALUE_FUNC(
    bool, build_mag_data_msg, can_msg_prio_t, uint16_t, char, can_imu_id_t, uint16_t, can_msg_t *
);

// Static buffer for IMU data capture in tests
static estimator_all_imus_input_t captured_data;
}

// Define input IMU vectors (ACC, GYRO, MAG)
static const vector3d_t INPUT_ACC = {1.0, 2.0, 3.0};
static const vector3d_t INPUT_GYRO = {4.0, 5.0, 6.0};
static const vector3d_t INPUT_MAG = {7.0, 8.0, 9.0};
static const vector3d_t INPUT_EULER = {10.0, 20.0, 30.0};
static const double INPUT_BARO = 101325.0; // Standard atmospheric pressure in Pa

static const vector3d_t EXPECTED_ACC_MOVELLA = {3.0, 1.0, 2.0};
// expect imu handler convert pololu from g to m/s^2 before orientation correction
static const vector3d_t EXPECTED_ACC_POLOLU = {-3.0 * 9.81f, -1.0 * 9.81f, 2.0 * 9.81f};
// expect imu handler converts pololu from deg to rad before orientation correction
static const vector3d_t EXPECTED_GYRO_MOVELLA = {6.0, 4.0, 5.0};
static const vector3d_t EXPECTED_GYRO_POLOLU = {
    -6.0 * M_PI / 180, -4.0 * M_PI / 180, 5.0 * M_PI / 180
};
static const vector3d_t EXPECTED_MAG_MOVELLA = {9.0, 7.0, 8.0};
static const vector3d_t EXPECTED_MAG_POLOLU = {-9.0, -7.0, 8.0};
// static const vector3d_t EXPECTED_EULER = {10.0, 20.0, 30.0}; // ahrs not used rn
static const double EXPECTED_BARO = 101325.0; // Standard atmospheric pressure in Pa

// Define tolerance for comparisons
static const double tolerance = 0.00005;

// Helper functions for setting up test data
static w_status_t timer_get_ms_custom_fake(float *time_ms) {
    *time_ms = 1000.0;
    return W_SUCCESS;
}

static w_status_t altimu_get_acc_data_success(vector3d_t *acc, altimu_raw_imu_data_t *raw_acc) {
    *acc = INPUT_ACC;
    raw_acc->x = 100;
    raw_acc->y = 200;
    raw_acc->z = 300;
    return W_SUCCESS;
}

static w_status_t altimu_get_gyro_data_success(vector3d_t *gyro, altimu_raw_imu_data_t *raw_gyro) {
    *gyro = INPUT_GYRO;
    raw_gyro->x = 400;
    raw_gyro->y = 500;
    raw_gyro->z = 600;
    return W_SUCCESS;
}

static w_status_t altimu_get_mag_data_success(vector3d_t *mag, altimu_raw_imu_data_t *raw_mag) {
    *mag = INPUT_MAG;
    raw_mag->x = 700;
    raw_mag->y = 800;
    raw_mag->z = 900;
    return W_SUCCESS;
}

static w_status_t altimu_get_gyro_acc_data_success(
    vector3d_t *acc_data, vector3d_t *gyro_data, altimu_raw_imu_data_t *raw_acc,
    altimu_raw_imu_data_t *raw_gyro
) {
    *acc_data = INPUT_ACC;
    *gyro_data = INPUT_GYRO;
    raw_acc->x = 100;
    raw_acc->y = 200;
    raw_acc->z = 300;
    raw_gyro->x = 400;
    raw_gyro->y = 500;
    raw_gyro->z = 600;
    return W_SUCCESS;
}

static w_status_t
altimu_get_baro_data_success(altimu_barometer_data_t *baro, altimu_raw_baro_data_t *raw_baro) {
    baro->pressure = INPUT_BARO;
    baro->temperature = 25.0;
    raw_baro->pressure = 101325;
    raw_baro->temperature = 33;
    return W_SUCCESS;
}

static w_status_t movella_get_data_success(movella_data_t *data, uint32_t timeout_ms) {
    data->acc = INPUT_ACC;
    data->gyr = INPUT_GYRO;
    data->mag = INPUT_MAG;
    data->euler = INPUT_EULER;
    data->pres = INPUT_BARO;
    data->temp = 25.0;
    return W_SUCCESS;
}

static w_status_t estimator_update_capture(estimator_all_imus_input_t *data) {
    memcpy(&captured_data, data, sizeof(estimator_all_imus_input_t));
    return W_SUCCESS;
}

class ImuHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test
        RESET_FAKE(altimu_init);
        // RESET_FAKE(altimu_get_acc_data);
        // RESET_FAKE(altimu_get_gyro_data);
        RESET_FAKE(altimu_get_gyro_acc_data);
        RESET_FAKE(altimu_get_mag_data);
        RESET_FAKE(altimu_get_baro_data);
        RESET_FAKE(altimu_check_sanity);
        RESET_FAKE(build_imu_data_msg);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(build_baro_data_msg);
        RESET_FAKE(movella_init);
        RESET_FAKE(movella_get_data);

        RESET_FAKE(estimator_update_imu_data);
        RESET_FAKE(timer_get_ms);

        // Reset FreeRTOS mocks
        RESET_FAKE(vTaskDelayUntil);

        // Default successful returns
        altimu_init_fake.return_val = W_SUCCESS;
        altimu_check_sanity_fake.return_val = W_SUCCESS;
        movella_init_fake.return_val = W_SUCCESS;
        timer_get_ms_fake.return_val = W_SUCCESS;
        estimator_update_imu_data_fake.return_val = W_SUCCESS;

        // Clear captured data
        memset(&captured_data, 0, sizeof(captured_data));

        // Initialize IMU handler before each test
        imu_handler_init();
    }
};

// Tests for initialization
TEST_F(ImuHandlerTest, InitSuccess) {
    EXPECT_EQ(W_SUCCESS, imu_handler_init());
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, RunSuccessful) {
    // Set up all mocks for successful readings
    // altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    // altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
    altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;

    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

    // Run the function under test with loop_count = 1
    w_status_t result = imu_handler_run(1);

    // Verify function returned success
    EXPECT_EQ(W_SUCCESS, result);

    // Verify IMU read calls were made
    // EXPECT_EQ(1, altimu_get_acc_data_fake.call_count);
    // EXPECT_EQ(1, altimu_get_gyro_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_gyro_acc_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_mag_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_baro_data_fake.call_count);
    EXPECT_EQ(1, movella_get_data_fake.call_count);

    // Verify timestamps
    EXPECT_EQ(1000, captured_data.pololu.timestamp_imu);
    EXPECT_EQ(1000, captured_data.movella.timestamp_imu);

    // Verify data values for Pololu
    assert_vec_eq(EXPECTED_ACC_POLOLU, captured_data.pololu.accelerometer, tolerance);
    assert_vec_eq(EXPECTED_GYRO_POLOLU, captured_data.pololu.gyroscope, tolerance);
    assert_vec_eq(EXPECTED_MAG_POLOLU, captured_data.pololu.magnetometer, tolerance);
    EXPECT_NEAR(captured_data.pololu.barometer, EXPECTED_BARO, abs(EXPECTED_BARO * tolerance));

    // Verify Movella data
    assert_vec_eq(EXPECTED_ACC_MOVELLA, captured_data.movella.accelerometer, tolerance);
    assert_vec_eq(EXPECTED_GYRO_MOVELLA, captured_data.movella.gyroscope, tolerance);
    assert_vec_eq(EXPECTED_MAG_MOVELLA, captured_data.movella.magnetometer, tolerance);
    EXPECT_NEAR(captured_data.movella.barometer, EXPECTED_BARO, abs(EXPECTED_BARO * tolerance));

    // Verify is_dead flags
    EXPECT_FALSE(captured_data.pololu.is_dead);
    EXPECT_FALSE(captured_data.movella.is_dead);
}

// Test with failed Polulu IMU
TEST_F(ImuHandlerTest, RunWithPoluluFailure) {
    // Set Polulu to fail
    // altimu_get_acc_data_fake.return_val = W_FAILURE;
    // altimu_get_gyro_data_fake.return_val = W_FAILURE;
    altimu_get_gyro_acc_data_fake.return_val = W_FAILURE;
    altimu_get_mag_data_fake.return_val = W_FAILURE;
    altimu_get_baro_data_fake.return_val = W_FAILURE;

    // Set Movella to succeed
    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

    // Run the function under test with loop_count = 1
    w_status_t result = imu_handler_run(1);

    // Function should return success since Movella is still working
    EXPECT_EQ(W_SUCCESS, result);

    // Verify Polulu data is marked as dead. doesnt matter what the data is
    EXPECT_TRUE(captured_data.pololu.is_dead);

    // Verify Movella data is still correct and not dead
    assert_vec_eq(EXPECTED_ACC_MOVELLA, captured_data.movella.accelerometer, tolerance);
    assert_vec_eq(EXPECTED_GYRO_MOVELLA, captured_data.movella.gyroscope, tolerance);
    assert_vec_eq(EXPECTED_MAG_MOVELLA, captured_data.movella.magnetometer, tolerance);
    EXPECT_NEAR(captured_data.movella.barometer, EXPECTED_BARO, abs(EXPECTED_BARO * tolerance));
    EXPECT_FALSE(captured_data.movella.is_dead);
}

// Test with failed Movella IMU
TEST_F(ImuHandlerTest, RunWithMovellaFailure) {
    // Set Polulu to succeed
    // altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    // altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    // Set Movella to fail
    movella_get_data_fake.return_val = W_FAILURE;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

    // Run the function under test with loop_count = 1
    w_status_t result = imu_handler_run(1);

    // Function should return success since Polulu is still working
    EXPECT_EQ(W_SUCCESS, result);

    // Verify Movella data is marked as dead. data doesnt matter if dead
    EXPECT_TRUE(captured_data.movella.is_dead);

    // Verify Polulu data is still correct and not dead
    assert_vec_eq(EXPECTED_ACC_POLOLU, captured_data.pololu.accelerometer, tolerance);
    assert_vec_eq(EXPECTED_GYRO_POLOLU, captured_data.pololu.gyroscope, tolerance);
    assert_vec_eq(EXPECTED_MAG_POLOLU, captured_data.pololu.magnetometer, tolerance);
    EXPECT_NEAR(captured_data.pololu.barometer, EXPECTED_BARO, abs(EXPECTED_BARO * tolerance));
    EXPECT_FALSE(captured_data.pololu.is_dead);
}

// Test with all IMUs failing
TEST_F(ImuHandlerTest, RunWithAllImusFailure) {
    // Set all IMUs to fail
    // altimu_get_acc_data_fake.return_val = W_FAILURE;
    // altimu_get_gyro_data_fake.return_val = W_FAILURE;
    altimu_get_gyro_acc_data_fake.return_val = W_FAILURE;
    altimu_get_mag_data_fake.return_val = W_FAILURE;
    altimu_get_baro_data_fake.return_val = W_FAILURE;
    movella_get_data_fake.return_val = W_FAILURE;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

    // Run the function under test with loop_count = 1
    w_status_t result = imu_handler_run(1);

    // Function should return failure since both IMUs failed
    EXPECT_EQ(W_FAILURE, result);

    // Verify all IMU data is marked as dead. data doesnt matter
    EXPECT_TRUE(captured_data.pololu.is_dead);
    EXPECT_TRUE(captured_data.movella.is_dead);
}

// Test behavior when timer fails
TEST_F(ImuHandlerTest, RunWithTimerFailure) {
    // Set up all IMUs for success
    // altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    // altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
    movella_get_data_fake.custom_fake = movella_get_data_success;

    // Set timer to fail
    timer_get_ms_fake.return_val = W_FAILURE;
    estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

    // Run the function under test with loop_count = 1
    w_status_t result = imu_handler_run(1);

    // Function should return success since IMUs are working
    EXPECT_EQ(W_SUCCESS, result);

    // Verify timestamps are zero
    EXPECT_EQ(0, captured_data.pololu.timestamp_imu);
    EXPECT_EQ(0, captured_data.movella.timestamp_imu);

    // But IMU data should still be valid and not dead
    assert_vec_eq(EXPECTED_ACC_POLOLU, captured_data.pololu.accelerometer, tolerance);
    EXPECT_FALSE(captured_data.pololu.is_dead);
    assert_vec_eq(EXPECTED_ACC_MOVELLA, captured_data.movella.accelerometer, tolerance);
    EXPECT_FALSE(captured_data.movella.is_dead);
}

// Test behavior when estimator update fails
TEST_F(ImuHandlerTest, RunWithEstimatorFailure) {
    // Set up all IMUs for success
    // altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    // altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // Set estimator update to fail
    estimator_update_imu_data_fake.return_val = W_FAILURE;

    // Run the function under test with loop_count = 1
    w_status_t result = imu_handler_run(1);

    // Function should return the failure from estimator
    EXPECT_EQ(W_FAILURE, result);

    // Verify IMU data was collected normally despite estimator failure
    EXPECT_EQ(1, altimu_get_gyro_acc_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_mag_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_baro_data_fake.call_count);
    EXPECT_EQ(1, movella_get_data_fake.call_count);
}

// Test CAN logging respects rate limit
TEST_F(ImuHandlerTest, ImuHandlerRunLoop_CanRateLimit) {
    // Arrange
    const uint32_t can_tx_rate = 16; // period is now 6 ms, so 100/6=16
    const uint32_t num_loops = 120; // Run for enough loops to cover multiple send cycles
    uint32_t expected_log_loops = 0;

    // Set up mocks for successful readings
    // altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    // altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

    build_imu_data_msg_fake.return_val = true; // Simulate successful CAN message build
    can_handler_transmit_fake.return_val = W_SUCCESS; // Simulate successful CAN transmission

    // Act
    for (uint32_t i = 0; i < num_loops; ++i) {
        imu_handler_run(i);
        if (i % can_tx_rate == 0) {
            expected_log_loops++;
        }
    }

    // Assert
    // Check that CAN-related functions were called the correct number of times
    EXPECT_EQ(build_imu_data_msg_fake.call_count, expected_log_loops * 3); // 3 imu msgs per cycle
    EXPECT_EQ(build_baro_data_msg_fake.call_count, expected_log_loops * 1); // 1 baro msg per cycle
    // 7 transmissions per cycle
    EXPECT_EQ(can_handler_transmit_fake.call_count, expected_log_loops * 7);
}

TEST_F(ImuHandlerTest, ImuHandlerRun_CanLogNominal) {
    // Arrange
    const uint32_t loop_count = 16; // Trigger CAN logging at this loop count
    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // Set up mocks for successful readings
    // altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    // altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
    movella_get_data_fake.custom_fake = movella_get_data_success;

    build_imu_data_msg_fake.return_val = true; // Simulate successful CAN message build
    build_baro_data_msg_fake.return_val = true; // Simulate successful CAN message build
    can_handler_transmit_fake.return_val = W_SUCCESS; // Simulate successful CAN transmission

    // Act
    w_status_t result = imu_handler_run(loop_count);

    // Assert
    EXPECT_EQ(result, W_SUCCESS); // Expect overall success

    // Verify CAN message build and transmit calls
    EXPECT_EQ(build_imu_data_msg_fake.call_count, 3); // 3 IMU messages (X, Y, Z)
    EXPECT_EQ(build_baro_data_msg_fake.call_count, 1); // 1 barometer message
    EXPECT_EQ(can_handler_transmit_fake.call_count, 7); // Total 7 CAN transmissions

    // Verify arguments for the first IMU message (X-axis)
    EXPECT_EQ(build_imu_data_msg_fake.arg0_history[0], PRIO_LOW);
    EXPECT_EQ(build_imu_data_msg_fake.arg2_history[0], 'X');
    EXPECT_EQ(build_imu_data_msg_fake.arg3_history[0], IMU_PROC_ALTIMU10);
    EXPECT_EQ(build_imu_data_msg_fake.arg4_history[0], 100); // Raw accelerometer X
    EXPECT_EQ(build_imu_data_msg_fake.arg5_history[0], 400); // Raw gyroscope X

    // Verify arguments for the barometer message
    EXPECT_EQ(build_baro_data_msg_fake.arg0_history[0], PRIO_LOW);
    EXPECT_EQ(build_baro_data_msg_fake.arg2_history[0], IMU_PROC_ALTIMU10);
    EXPECT_EQ(build_baro_data_msg_fake.arg3_history[0], 101325); // Raw pressure
    EXPECT_EQ(build_baro_data_msg_fake.arg4_history[0], 33); // Raw temperature
}
