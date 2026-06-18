/**
 * orientation correction from matlab commit e20e5d1
 */

#include "fff.h"
#include "utils/math_testing_helpers.hpp"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include "application/can_handler/can_handler.h"
#include "FreeRTOS.h"
#include "application/estimator/estimator_types.h"
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
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/IIS2MDC/IIS2MDC.h"


    // Define all fake functions for IMUs using FFF
    FAKE_VALUE_FUNC(w_status_t, movella_get_data, movella_data_t*, uint32_t);

    FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t*);

	FAKE_VALUE_FUNC(w_status_t, lsm6dsv32x_get_gyro_acc_data, vector3d_t*, vector3d_t*, lsm6dsv32x_raw_imu_data_t*,
										lsm6dsv32x_raw_imu_data_t*, bool*);

	FAKE_VALUE_FUNC(w_status_t, iis2mdc_get_data, vector3d_t*, iis2mdc_raw_data_t*, uint32_t*, bool*);

    // Fakes for logging
    FAKE_VALUE_FUNC(w_status_t, log_init);
    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char*, const char*, ...);
    FAKE_VALUE_FUNC(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t*);

// fake can stuff
// w_status_t can_handler_transmit(const can_msg_t *msg);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
// TODO: add unit tests for these new canlib message builders
// FAKE_VALUE_FUNC(w_status_t, can_encode_scaled_int, can_scaling_types_t, int64_t, void*);
FAKE_VOID_FUNC(build_analog_sensor_16bit_msg, can_msg_prio_t, uint16_t, can_analog_sensor_id_t, uint16_t, can_msg_t *);
FAKE_VOID_FUNC(build_3d_analog_sensor_16bit_msg, can_msg_prio_t, uint16_t, can_dem_3d_sensor_id_t, uint16_t, uint16_t, uint16_t, can_msg_t *);
FAKE_VOID_FUNC(build_2d_analog_sensor_24bit_msg, can_msg_prio_t, uint16_t, can_dem_2d_sensor_id_t, uint32_t, uint32_t, can_msg_t *);
// FAKE_VALUE_FUNC(
//     bool, build_baro_data_msg, can_msg_prio_t, uint16_t, can_imu_id_t, uint32_t, uint16_t,
//     can_msg_t *
// );
// FAKE_VALUE_FUNC(
//     bool, build_imu_data_msg, can_msg_prio_t, uint16_t, char, can_imu_id_t, uint16_t, uint16_t,
//     can_msg_t *
// );
// FAKE_VALUE_FUNC(
//     bool, build_mag_data_msg, can_msg_prio_t, uint16_t, char, can_imu_id_t, uint16_t, can_msg_t *
// );

    // Static buffer for IMU data capture in tests
    static all_sensors_data_t captured_data;
}

// Define tolerance for comparisons
static const double tolerance = 0.00005;

typedef struct {
	uint32_t timestamp_ms;
	w_status_t return_val;
} timer_get_values_t;

static timer_get_values_t g_timer_get_return_val = {0};

// Helper functions for setting up test data
static w_status_t timer_get_ms_custom_fake(uint32_t* time_ms) {
    *time_ms = g_timer_get_return_val.timestamp_ms;
    return g_timer_get_return_val.return_val;
}

typedef struct {
	vector3d_t acc_data;
	vector3d_t gyro_data;
	lsm6dsv32x_raw_imu_data_t raw_acc;
	lsm6dsv32x_raw_imu_data_t raw_gyro;
	bool is_dead;
	w_status_t return_val;
} lsm6_get_values_t;

static lsm6_get_values_t g_lsm6_get_return_val = {0};

static w_status_t set_lsm6dsv32x_get_gyro_acc_data(vector3d_t *acc_data, vector3d_t *gyro_data,
										lsm6dsv32x_raw_imu_data_t *raw_acc,
										lsm6dsv32x_raw_imu_data_t *raw_gyro, bool *is_dead) {
	*acc_data = g_lsm6_get_return_val.acc_data;
	*gyro_data = g_lsm6_get_return_val.gyro_data;
	*raw_acc = g_lsm6_get_return_val.raw_acc;
	*raw_gyro = g_lsm6_get_return_val.raw_gyro;
	*is_dead = g_lsm6_get_return_val.is_dead;
	return g_lsm6_get_return_val.return_val;
}

typedef struct {
	vector3d_t data;
	iis2mdc_raw_data_t raw_data;
	uint32_t timestamp_ms;
	bool is_dead;
	w_status_t return_val;
} iis2_get_values_t;

static iis2_get_values_t g_iis2_get_return_val = {0};

static w_status_t set_iis2mdc_get_data(vector3d_t *data, iis2mdc_raw_data_t *raw_data, uint32_t *timestamp_ms,
							bool *is_dead) {
	*data = g_iis2_get_return_val.data;
	*raw_data = g_iis2_get_return_val.raw_data;
	*timestamp_ms = g_iis2_get_return_val.timestamp_ms;
	*is_dead = g_iis2_get_return_val.is_dead;
	return g_iis2_get_return_val.return_val;
}

typedef struct {
	movella_data_t data;
	w_status_t return_val;
} mti_get_values_t;

static mti_get_values_t g_mti_get_return_val = {0};

static w_status_t set_movella_get_data(movella_data_t *data, uint32_t timeout_ms) {
	*data = g_mti_get_return_val.data;
	return g_mti_get_return_val.return_val;
}

class ImuHandlerTest : public ::testing::Test {
protected:
	void SetUp() override {
		// Reset all fakes before each test
		RESET_FAKE(lsm6dsv32x_get_gyro_acc_data);
		RESET_FAKE(iis2mdc_get_data);
		RESET_FAKE(can_handler_transmit);
		RESET_FAKE(movella_get_data);

		RESET_FAKE(timer_get_ms);

		// Reset FreeRTOS mocks
		RESET_FAKE(vTaskDelayUntil);

		// Default successful returns
		timer_get_ms_fake.return_val = W_SUCCESS;

		// Clear captured data
		memset(&captured_data, 0, sizeof(captured_data));

		// Initialize IMU handler before each test
		imu_handler_init();

      
		// reset log test and queuecreate as this point so all of the logs that are captured are from this point on
		RESET_FAKE(log_text);

	}
};

// Tests for initialization
TEST_F(ImuHandlerTest, InitSuccess) {
	EXPECT_EQ(W_SUCCESS, imu_handler_init());
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasAllSensorNew) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasComDeadLSM6) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = true;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_FALSE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewLSM6) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 997;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 997;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_FALSE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasFailGetLSM6) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_FAILURE;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_FALSE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasComDeadMag) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = true;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_FALSE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewMag) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 995;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_FALSE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasFailGetMag) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_FAILURE;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_FALSE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasComDeadMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = true;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_FALSE(output.mti_meas.mti_accel.is_new);
	EXPECT_FALSE(output.mti_meas.mti_gyro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_baro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewAccelMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 997;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_FALSE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewGyroMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 997;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_FALSE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewBaroMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 997;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewMagMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 997;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasNotNewOtherMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 0;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 0;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_SUCCESS;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_TRUE(output.mti_meas.mti_accel.is_new);
	EXPECT_TRUE(output.mti_meas.mti_gyro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_baro.is_new);
	EXPECT_TRUE(output.mti_meas.mti_mag.is_new);
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, GetFreshMeasGetFailMTI) {
	// Mock Sensor Readings
	g_lsm6_get_return_val.raw_acc.timestamp_ms = 999;
	g_lsm6_get_return_val.raw_gyro.timestamp_ms = 999;
	g_lsm6_get_return_val.is_dead = false;
	g_lsm6_get_return_val.return_val = W_SUCCESS;

	g_iis2_get_return_val.timestamp_ms = 999;
	g_iis2_get_return_val.is_dead = false;
	g_iis2_get_return_val.return_val = W_SUCCESS;

	g_mti_get_return_val.data.acc_timestamp_ms = 999;
	g_mti_get_return_val.data.gyr_timestamp_ms = 999;
	g_mti_get_return_val.data.euler_timestamp_ms = 999;
	g_mti_get_return_val.data.mag_timestamp_ms = 999;
	g_mti_get_return_val.data.pres_timestamp_ms = 999;
	g_mti_get_return_val.data.temp_timestamp_ms = 999;
	g_mti_get_return_val.data.is_dead = false;
	g_mti_get_return_val.return_val = W_FAILURE;

	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.custom_fake = set_lsm6dsv32x_get_gyro_acc_data;
	iis2mdc_get_data_fake.custom_fake = set_iis2mdc_get_data;

	movella_get_data_fake.custom_fake = set_movella_get_data;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {
		.last_board_imu_timestamp_ms = 997,
		.last_baro_timestamp_ms = 980,
		.last_mag_timestamp_ms = 995,

		.last_ad_accel_timestamp_ms = 997,
		.last_ad_gyro_timestamp_ms = 997,

		.last_mti_acc_timestamp_ms = 997,
		.last_mti_gyr_timestamp_ms = 997,
		.last_mti_mag_timestamp_ms = 997,
		.last_mti_pres_timestamp_ms = 997
	};

	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Verify function returned success
	EXPECT_EQ(W_SUCCESS, result);

	// Verify IMU read calls were made
	EXPECT_EQ(1, lsm6dsv32x_get_gyro_acc_data_fake.call_count);
	EXPECT_EQ(1, iis2mdc_get_data_fake.call_count);

	EXPECT_EQ(1, movella_get_data_fake.call_count);

	// TODO: verify data

	// Verify new flags
	EXPECT_TRUE(output.board_meas.board_imu.is_new);
	EXPECT_TRUE(output.board_meas.board_mag.is_new);

	EXPECT_FALSE(output.mti_meas.mti_accel.is_new);
	EXPECT_FALSE(output.mti_meas.mti_gyro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_baro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_mag.is_new);
}

TEST_F(ImuHandlerTest, GetFreshMeasWithInvalidPtrOutput) {
	// Set up values
	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.return_val = W_SUCCESS;
	iis2mdc_get_data_fake.return_val = W_SUCCESS;

	movella_get_data_fake.return_val = W_SUCCESS;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {0};


	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, NULL);

	// Function should return the failure from estimator
	EXPECT_EQ(W_INVALID_PARAM, result);
}

TEST_F(ImuHandlerTest, GetFreshMeasWithInvalidPtrCtx) {
	// Set up values
	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_SUCCESS;

	lsm6dsv32x_get_gyro_acc_data_fake.return_val = W_SUCCESS;
	iis2mdc_get_data_fake.return_val = W_SUCCESS;

	movella_get_data_fake.return_val = W_SUCCESS;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {0};


	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(NULL, &output);

	// Function should return the failure from estimator
	EXPECT_EQ(W_INVALID_PARAM, result);
}

TEST_F(ImuHandlerTest, GetFreshMeasFailWithTimerFail) {
	// Set up values
	g_timer_get_return_val.timestamp_ms = 1000;
	g_timer_get_return_val.return_val = W_FAILURE;

	lsm6dsv32x_get_gyro_acc_data_fake.return_val = W_SUCCESS;
	iis2mdc_get_data_fake.return_val = W_SUCCESS;

	movella_get_data_fake.return_val = W_SUCCESS;

	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // set up output ptr
	all_sensors_data_t output = {0};
	imu_handler_ctx_t ctx = {0};


	// Run the function under test with loop_count = 1
	w_status_t result = imu_handler_get_fresh_meas(&ctx, &output);

	// Function should return the failure from estimator
	EXPECT_EQ(W_FAILURE, result);

	// Verify new flags
	EXPECT_FALSE(output.board_meas.board_imu.is_new);
	EXPECT_FALSE(output.board_meas.board_mag.is_new);

	EXPECT_FALSE(output.mti_meas.mti_accel.is_new);
	EXPECT_FALSE(output.mti_meas.mti_gyro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_baro.is_new);
	EXPECT_FALSE(output.mti_meas.mti_mag.is_new);
}

// Revive once this has been reimplmented
// // Test CAN logging respects rate limit
// TEST_F(ImuHandlerTest, ImuHandlerRunLoop_CanRateLimit) {
// 	// Arrange
// 	const uint32_t can_tx_rate = 16; // period is now 6 ms, so 100/6=16
// 	const uint32_t num_loops = 120; // Run for enough loops to cover multiple send cycles
// 	uint32_t expected_log_loops = 0;

// 	// Set up mocks for successful readings
// 	// altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
// 	// altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
// 	altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
// 	altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
// 	altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
// 	movella_get_data_fake.custom_fake = movella_get_data_success;

// 	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
// 	estimator_update_imu_data_fake.custom_fake = estimator_update_capture;

// 	build_imu_data_msg_fake.return_val = true; // Simulate successful CAN message build
// 	can_handler_transmit_fake.return_val = W_SUCCESS; // Simulate successful CAN transmission


//    // set up output ptr
//	  all_sensors_data_t output = {0};
	
// 	// Act
// 	for (uint32_t i = 0; i < num_loops; ++i) {
// 		imu_handler_get_fresh_meas(&output);
// 		if (i % can_tx_rate == 0) {
// 			expected_log_loops++;
// 		}
// 	}

// 	// Assert
// 	// Check that CAN-related functions were called the correct number of times
// 	EXPECT_EQ(build_imu_data_msg_fake.call_count, expected_log_loops * 3); // 3 imu msgs per cycle
// 	EXPECT_EQ(build_baro_data_msg_fake.call_count, expected_log_loops * 1); // 1 baro msg per cycle
// 	// 7 transmissions per cycle
// 	EXPECT_EQ(can_handler_transmit_fake.call_count, expected_log_loops * 7);
// }

// TEST_F(ImuHandlerTest, ImuHandlerRun_CanLogNominal) {
// 	// Arrange
// 	const uint32_t loop_count = 16; // Trigger CAN logging at this loop count
// 	timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

// 	// Set up mocks for successful readings
// 	// altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
// 	// altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
// 	altimu_get_gyro_acc_data_fake.custom_fake = altimu_get_gyro_acc_data_success;
// 	altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
// 	altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;
// 	movella_get_data_fake.custom_fake = movella_get_data_success;

// 	build_imu_data_msg_fake.return_val = true; // Simulate successful CAN message build
// 	build_baro_data_msg_fake.return_val = true; // Simulate successful CAN message build
// 	can_handler_transmit_fake.return_val = W_SUCCESS; // Simulate successful CAN transmission

//    // set up output ptr
//    all_sensors_data_t output = {0};

// 	// Act
// 	w_status_t result = imu_handler_run(loop_count, &output);

// 	// Assert
// 	EXPECT_EQ(result, W_SUCCESS); // Expect overall success

// 	// Verify CAN message build and transmit calls
// 	// TODO: revive these with the new associated messages
//     // EXPECT_EQ(build_imu_data_msg_fake.call_count, 3); // 3 IMU messages (X, Y, Z)
// 	// EXPECT_EQ(build_baro_data_msg_fake.call_count, 1); // 1 barometer message

//     // TODO: double check this is the new standard
// 	EXPECT_EQ(can_handler_transmit_fake.call_count, 4); // Total 7 CAN transmissions

// 	// Verify arguments for the first IMU message (X-axis)
// 	EXPECT_EQ(build_imu_data_msg_fake.arg0_history[0], PRIO_LOW);
// 	EXPECT_EQ(build_imu_data_msg_fake.arg2_history[0], 'X');
// 	EXPECT_EQ(build_imu_data_msg_fake.arg3_history[0], IMU_PROC_ALTIMU10);
// 	EXPECT_EQ(build_imu_data_msg_fake.arg4_history[0], 100); // Raw accelerometer X
// 	EXPECT_EQ(build_imu_data_msg_fake.arg5_history[0], 400); // Raw gyroscope X

// 	// Verify arguments for the barometer message
// 	EXPECT_EQ(build_baro_data_msg_fake.arg0_history[0], PRIO_LOW);
// 	EXPECT_EQ(build_baro_data_msg_fake.arg2_history[0], IMU_PROC_ALTIMU10);
// 	EXPECT_EQ(build_baro_data_msg_fake.arg3_history[0], 101325); // Raw pressure
// 	EXPECT_EQ(build_baro_data_msg_fake.arg4_history[0], 33); // Raw temperature
// }

TEST_F(ImuHandlerTest, ImuHandlerRun_CalibrationWarning) {
	// Arrange
	// Simulate uncalibrated orientation by setting the flag to failure
	bool orientation_calibrated = false;

	// Act
	w_status_t result = imu_handler_init();

	// Assert
	EXPECT_EQ(result, W_SUCCESS); // Initialization should still succeed
	EXPECT_STREQ(log_text_fake.arg1_history[0], "IMUHandler");
	EXPECT_STREQ(log_text_fake.arg2_history[0],
				 "WARN: IMU orientation correction matrices not calibrated yet, using default "
				 "orientation.");
}