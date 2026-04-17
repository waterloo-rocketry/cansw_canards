#include "../mocks/fff/fff.h"
#include <gtest/gtest.h>

// Define error codes to match those in health_checks.c
#define E_NOMINAL 0x00
#define E_WATCHDOG_TIMEOUT 0x81

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "application/health_checks/health_checks.h"
#include "canlib.h"
#include "drivers/adc/adc.h"
#include "message_types.h"
#include "rocketlib/include/common.h"

// all the functions that are being tested
extern w_status_t health_check_exec();
extern w_status_t get_adc_current(uint32_t *adc_current_mA);
extern w_status_t check_watchdog_tasks(void);

FAKE_VALUE_FUNC(w_status_t, adc_get_raw_volts, adc_channel_t, uint32_t *, uint32_t);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, uint32_t *);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, can_msg_t *);
FAKE_VALUE_FUNC(bool, build_general_board_status_msg, can_msg_prio_t, uint16_t, uint32_t, uint16_t,
				can_msg_t *);
// FAKE_VALUE_FUNC(TaskHandle_t, xTaskGetCurrentTaskHandle);
FAKE_VOID_FUNC(log_text, uint32_t, const char *, const char *, void *);
FAKE_VALUE_FUNC(bool, build_analog_data_msg, can_msg_prio_t, uint16_t, can_analog_sensor_id_t,
				uint16_t, can_msg_t *);

// Mock implementations for all the module get_status functions
FAKE_VALUE_FUNC(uint32_t, i2c_get_status);
FAKE_VALUE_FUNC(uint32_t, adc_get_status);
FAKE_VALUE_FUNC(uint32_t, can_handler_get_status);
FAKE_VALUE_FUNC(uint32_t, estimator_get_status);
FAKE_VALUE_FUNC(uint32_t, controller_get_status);
FAKE_VALUE_FUNC(uint32_t, sd_card_get_status);
FAKE_VALUE_FUNC(uint32_t, timer_get_status);
FAKE_VALUE_FUNC(uint32_t, logger_get_status);
FAKE_VALUE_FUNC(uint32_t, gpio_get_status);
FAKE_VALUE_FUNC(uint32_t, flight_phase_get_status);
FAKE_VALUE_FUNC(uint32_t, imu_handler_get_status);
FAKE_VALUE_FUNC(uint32_t, uart_get_status);

// Mocked global variables
static uint32_t timer_ms_value_mock;
static uint32_t adc_value_mock;
}

DEFINE_FFF_GLOBALS;

// Mocked functions for helping with unit testing
// check if having extra static functions is good or not for unit testing
static w_status_t timer_get_ms_mock(uint32_t *out_time) {
	*out_time = timer_ms_value_mock;
	return W_SUCCESS;
}

static w_status_t adc_get_raw_volts_mock(adc_channel_t channel, uint32_t *out_value,
										 uint32_t timeout_ms) {
	*out_value = adc_value_mock;
	return W_SUCCESS;
}

class HealthChecksTest : public ::testing::Test {
protected:
	void SetUp() override {
		RESET_FAKE(adc_get_raw_volts);
		RESET_FAKE(timer_get_ms);
		RESET_FAKE(can_handler_transmit);
		RESET_FAKE(build_general_board_status_msg);
		RESET_FAKE(build_analog_data_msg);
		RESET_FAKE(can_handler_transmit);
		RESET_FAKE(xTaskGetCurrentTaskHandle);
		RESET_FAKE(xTaskGetTickCount);
		RESET_FAKE(vTaskDelayUntil);
		RESET_FAKE(log_text);

		// Reset all the module status function fakes
		RESET_FAKE(i2c_get_status);
		RESET_FAKE(adc_get_status);
		RESET_FAKE(can_handler_get_status);
		RESET_FAKE(estimator_get_status);
		RESET_FAKE(controller_get_status);
		RESET_FAKE(sd_card_get_status);
		RESET_FAKE(timer_get_status);
		RESET_FAKE(logger_get_status);
		RESET_FAKE(gpio_get_status);
		RESET_FAKE(flight_phase_get_status);
		RESET_FAKE(imu_handler_get_status);
		RESET_FAKE(uart_get_status);

		FFF_RESET_HISTORY();

		adc_get_raw_volts_fake.return_val = W_SUCCESS;
		timer_get_ms_fake.return_val = W_SUCCESS;
		can_handler_transmit_fake.return_val = W_SUCCESS;
		build_general_board_status_msg_fake.return_val = true;

		// Set default return values for module status functions
		i2c_get_status_fake.return_val = W_SUCCESS;
		adc_get_status_fake.return_val = W_SUCCESS;
		can_handler_get_status_fake.return_val = W_SUCCESS;
		estimator_get_status_fake.return_val = W_SUCCESS;
		controller_get_status_fake.return_val = W_SUCCESS;
		sd_card_get_status_fake.return_val = W_SUCCESS;
		timer_get_status_fake.return_val = W_SUCCESS;
		logger_get_status_fake.return_val = W_SUCCESS;
		gpio_get_status_fake.return_val = W_SUCCESS;
		flight_phase_get_status_fake.return_val = W_SUCCESS;
		imu_handler_get_status_fake.return_val = W_SUCCESS;
		uart_get_status_fake.return_val = W_SUCCESS;
	}

	void TearDown() override {}

	// Helper functions to set up the test environment
	void SetTimerMs(uint32_t time_ms) {
		timer_ms_value_mock = time_ms;
		timer_get_ms_fake.custom_fake = timer_get_ms_mock;
	}

	// Set the ADC value to be returned by the fake function
	void SetAdcValue(uint32_t adc_val) {
		adc_value_mock = adc_val;
		adc_get_raw_volts_fake.custom_fake = adc_get_raw_volts_mock;
	}

	// Constants for mocking tests
	static constexpr float ADC_VREF = 3.3f;
	static constexpr float INA180A3_GAIN = 100.0f;
	static constexpr float R_SENSE = 0.033f;
	static constexpr uint16_t MAX_CURRENT_mA = 400;
	static constexpr uint32_t ADC_MAX_COUNTS_MOCK = 65535; // need another local definition for this
};

TEST_F(HealthChecksTest, GetAdcCurrentPassing) {
	// Arrange
	uint32_t adc_current_mA;
	uint32_t adc_value = 1000; // Example ADC value
	SetAdcValue(adc_value);

	float voltage_mV = ((float)(adc_value) / ADC_MAX_COUNTS_MOCK) * ADC_VREF * 1000.0f;
	uint32_t expected_current_mA =
		((float)(voltage_mV) / INA180A3_GAIN) / R_SENSE; // doing the math again here

	// Act
	w_status_t result = get_adc_current(&adc_current_mA);

	// Assert
	EXPECT_EQ(W_SUCCESS, result);
	EXPECT_EQ(expected_current_mA, adc_current_mA);
	EXPECT_EQ(adc_get_raw_volts_fake.call_count, 1);
	EXPECT_EQ(adc_get_raw_volts_fake.arg0_val, PROCESSOR_BOARD_VOLTAGE);
}

TEST_F(HealthChecksTest, GetAdcCurrentFailing) {
	// Arrange
	uint32_t adc_current_mA;
	adc_get_raw_volts_fake.return_val = W_IO_TIMEOUT;

	// Act
	w_status_t result = get_adc_current(&adc_current_mA);

	// Assert
	EXPECT_EQ(W_IO_TIMEOUT, result);
	EXPECT_EQ(adc_get_raw_volts_fake.call_count, 1);
}

TEST_F(HealthChecksTest, NominalHealthCheck) {
	// Arrange
	SetTimerMs(1000);
	uint32_t nominal_current = 300; // example current
	SetAdcValue(
		// +1 for integer rounding
		nominal_current * R_SENSE * INA180A3_GAIN / (ADC_VREF * 1000.0f) * ADC_MAX_COUNTS_MOCK + 1);
	build_general_board_status_msg_fake.return_val = true;
	build_analog_data_msg_fake.return_val = true;

	// Act
	w_status_t result = health_check_exec();

	// Assert
	EXPECT_EQ(W_SUCCESS, result);
	EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
	EXPECT_EQ(build_general_board_status_msg_fake.arg0_val, PRIO_LOW);
	EXPECT_EQ(build_general_board_status_msg_fake.arg2_val, E_NOMINAL);
	EXPECT_EQ(build_analog_data_msg_fake.call_count, 1);
	EXPECT_EQ(build_analog_data_msg_fake.arg0_val, PRIO_LOW);
	EXPECT_EQ(build_analog_data_msg_fake.arg2_val, SENSOR_5V_CURR);
	EXPECT_EQ(build_analog_data_msg_fake.arg3_val, nominal_current);
}

TEST_F(HealthChecksTest, OvercurrentHealthCheck) {
	// Arrange
	SetTimerMs(1000);
	uint32_t over_current = 5000; // example too much current
	// +1 for integer rounding
	SetAdcValue(
		over_current * R_SENSE * INA180A3_GAIN / (ADC_VREF * 1000.0f) * ADC_MAX_COUNTS_MOCK + 1);

	build_general_board_status_msg_fake.return_val = true;
	build_analog_data_msg_fake.return_val = true;

	// Act
	w_status_t result = health_check_exec();

	// Assert
	EXPECT_EQ(W_SUCCESS, result);
	EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
	EXPECT_EQ(build_general_board_status_msg_fake.arg0_val, PRIO_LOW);
	EXPECT_EQ(build_general_board_status_msg_fake.arg2_val, 1 << E_5V_OVER_CURRENT_OFFSET);
	EXPECT_EQ(build_analog_data_msg_fake.arg0_val, PRIO_LOW);
	EXPECT_EQ(build_analog_data_msg_fake.arg2_val, SENSOR_5V_CURR);
	EXPECT_EQ(build_analog_data_msg_fake.arg3_val, over_current);
}

TEST_F(HealthChecksTest, FailureHealthCheck) {
	// Arrange
	SetTimerMs(1000);
	uint32_t nominal_current = 300;
	// +1 for integer rounding
	SetAdcValue(
		nominal_current * R_SENSE * INA180A3_GAIN / (ADC_VREF * 1000.0f) * ADC_MAX_COUNTS_MOCK + 1);
	build_general_board_status_msg_fake.return_val = false;
	build_analog_data_msg_fake.return_val = true;

	// Act
	w_status_t result = health_check_exec();

	// Assert
	EXPECT_EQ(W_FAILURE, result);
	EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
	EXPECT_EQ(build_analog_data_msg_fake.arg0_val, PRIO_LOW);
	EXPECT_EQ(build_analog_data_msg_fake.arg2_val, SENSOR_5V_CURR);
	EXPECT_EQ(build_analog_data_msg_fake.arg3_val, nominal_current);
}

TEST_F(HealthChecksTest, WatchdogRegisterAndKick) {
	// Arrange
	TaskHandle_t fake_task = (TaskHandle_t)0x12345678; // bs address
	uint32_t timeout_ticks = pdMS_TO_TICKS(100);
	SetTimerMs(1000);
	xTaskGetCurrentTaskHandle_fake.return_val = fake_task;

	// Act
	watchdog_register_task(fake_task, timeout_ticks);
	watchdog_kick();

	// Assert
	EXPECT_EQ(timer_get_ms_fake.call_count, 2); // Once for register, once for kick
	EXPECT_EQ(xTaskGetCurrentTaskHandle_fake.call_count, 1);

	// Advance time but still within timeout becuase timout was earlier set to 100ms
	SetTimerMs(1050);
	w_status_t result = check_watchdog_tasks();

	// Assert
	EXPECT_EQ(W_SUCCESS, result);
	EXPECT_EQ(build_general_board_status_msg_fake.call_count, 0);
}

TEST_F(HealthChecksTest, WatchdogTimeout) {
	// Arrange
	TaskHandle_t fake_task = (TaskHandle_t)0x12345678;
	uint32_t timeout_ticks = pdMS_TO_TICKS(100);
	SetTimerMs(1000);
	xTaskGetCurrentTaskHandle_fake.return_val = fake_task;

	watchdog_register_task(fake_task, timeout_ticks); // skip kicking to get to the timeout logic

	check_watchdog_tasks();

	build_general_board_status_msg_fake.return_val = true;
	can_handler_transmit_fake.return_val = W_SUCCESS;

	// Advance time beyond timeout
	SetTimerMs(1200); // 200ms later, should trigger timeout

	// Act
	uint32_t result = check_watchdog_tasks();

	// Assert
	EXPECT_EQ(1 << E_WATCHDOG_TIMEOUT_OFFSET, result);
}

TEST_F(HealthChecksTest, WatchdogMaxTasksLimit) {
	// Arrange
	TaskHandle_t fake_tasks[15]; // More than MAX_WATCHDOG_TASKS = 10
	for (uint32_t i = 0; i < 15; i++) {
		fake_tasks[i] = (TaskHandle_t)(uintptr_t)(0x10000 + i); // filling up with bs addresses
	}
	SetTimerMs(1000);

	// Act
	for (int i = 0; i < 15; i++) {
		watchdog_register_task(fake_tasks[i], 100); // never would take more than 10 anyway
	}
	SetTimerMs(1050);

	w_status_t result = check_watchdog_tasks();

	// Assert
	EXPECT_EQ(W_SUCCESS, result); // check watchdog still goes through
	EXPECT_EQ(build_general_board_status_msg_fake.call_count, 0);
}
