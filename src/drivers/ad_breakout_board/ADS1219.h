/*
ACK: https://github.com/binomaiheu/ADS1219
*/

/**
 * @file ADS1219.h
 * @brief ADS1219 24-bit ADC driver for STM32 (C port)
 *
 * C port of the Arduino ADS1219 C++ library, adapted to use the project's
 * I2C bus abstraction and w_status_t error codes.
 *
 * Key changes from the original Arduino C++ version:
 *   - Language: C instead of C++
 *   - I2C layer: Uses the project i2c_read_reg / i2c_write_reg helpers
 *   - Error codes: Returns w_status_t (W_SUCCESS, W_IO_ERROR, W_INVALID_PARAM,
 *     W_IO_TIMEOUT, W_OVERFLOW)
 *   - State: All per-device state lives in an ads1219_handle_t struct
 *   - Delays: Uses vTaskDelay() (FreeRTOS-safe).
 *   - Header guard: #ifndef instead of #pragma once.
 *   - Removed: Arduino architecture-specifics.
 */

#ifndef ADS1219_H
#define ADS1219_H

#include <stdbool.h>
#include <stdint.h>

#include "common/math/math.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"

/* Default I2C address, A0 and A1 both to DGND */
#define ADS1219_I2C_ADDRESS 0x40

/* Command bytes (table 7, p.25 in datasheet) */
#define ADS1219_CMD_RESET 0x06
#define ADS1219_CMD_START_SYNC 0x08
#define ADS1219_CMD_POWERDOWN 0x02
#define ADS1219_CMD_RDATA 0x10
#define ADS1219_CMD_RREG_CONFIG 0x20
#define ADS1219_CMD_RREG_STATUS 0x24
#define ADS1219_CMD_WREG 0x40

/* Config register field masks (1 = untouched, 0 = field bits) */
#define ADS1219_CONFIG_MASK_VREF 0xFE /* bit 0 */
#define ADS1219_CONFIG_MASK_CM 0xFD /* bit 1 */
#define ADS1219_CONFIG_MASK_DR 0xF3 /* bits 2-3 */
#define ADS1219_CONFIG_MASK_GAIN 0xEF /* bit 4 */
#define ADS1219_CONFIG_MASK_MUX 0x1F /* bits 5-7 */

/* MUX field values */ // AINP - AINN
#define ADS1219_MUX_DIFF_0_1 0x00 // AIN0 - AIN1
#define ADS1219_MUX_DIFF_2_3 0x20 // AIN2 - AIN3
#define ADS1219_MUX_DIFF_1_2 0x40 // AIN1 - AIN2
#define ADS1219_MUX_SINGLE_0 0x60 // AIN0 - AGND
#define ADS1219_MUX_SINGLE_1 0x80 // AIN1 - AGND
#define ADS1219_MUX_SINGLE_2 0xA0 // AIN2 - AGND
#define ADS1219_MUX_SINGLE_3 0xC0 // AIN3 - AGND
#define ADS1219_MUX_SHORTED 0xE0 // AVDD/2 - AVDD/2

/* Gain settings */
#define ADS1219_GAIN_ONE 1
#define ADS1219_GAIN_FOUR 4

/* Voltage reference */
#define ADS1219_VREF_INTERNAL 0
#define ADS1219_VREF_EXTERNAL 1

/* Data-rate settings */
#define ADS1219_DATARATE_20SPS 0
#define ADS1219_DATARATE_90SPS 1
#define ADS1219_DATARATE_330SPS 2
#define ADS1219_DATARATE_1000SPS 3

/* Conversion mode */
#define ADS1219_CM_SINGLE_SHOT 0
#define ADS1219_CM_CONTINUOUS 1

/**
 * @brief Run-time handle for one ADS1219 device.
 *
 * Replaces the C++ class -- all per-device state lives here.
 * The caller allocates this (stack or static) and passes it to every function.
 */
typedef struct {
	i2c_bus_t bus; /**< Which project I2C bus the device sits on */
	uint8_t i2c_addr; /**< 7-bit I2C address */
	uint8_t gain;
	float32_t aref_n; /**< Negative reference voltage in mV */
	float32_t aref_p; /**< Positive reference voltage in mV */
	bool initialized; /**< Set true after ads1219_init() succeeds */
} ads1219_handle_t;

/**
 * @brief Initialise the handle and reset the device.
 *
 * @param[out] p_handle  Handle to populate
 * @param[in]  bus     Project I2C bus the device is on
 * @param[in]  addr    7-bit I2C address (use ADS1219_I2C_ADDRESS for default)
 * @return W_SUCCESS or an error code
 */
w_status_t ads1219_init(ads1219_handle_t *p_handle, i2c_bus_t bus, uint8_t addr);

/**
 * @brief Issue a RESET command.
 * @param p_handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_reset(ads1219_handle_t *p_handle);

/**
 * @brief Issue a START / SYNC command (begin conversion).
 * @param p_handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_start(ads1219_handle_t *p_handle);

/**
 * @brief Issue a POWERDOWN command.
 * @param p_handle handle to the ADC data
 * @return status of function
 */
w_status_t ads1219_powerdown(ads1219_handle_t *p_handle);

/**
 * @brief Read the current gain setting from the config register.
 * @param p_handle handle to the ADC data
 * @param[out] gain  ADS1219_GAIN_ONE or ADS1219_GAIN_FOUR
 * @return status of function
 */
w_status_t ads1219_get_gain(ads1219_handle_t *p_handle, uint8_t *gain);

/**
 * @brief Set the gain.
 * @param handle handle to the ADC data
 * @param[in] gain  ADS1219_GAIN_ONE or ADS1219_GAIN_FOUR
 * @return status of function
 */
w_status_t ads1219_set_gain(ads1219_handle_t *handle, uint8_t gain);

/**
 * @brief Read the current voltage-reference setting.
 * @param p_handle handle to the ADC data
 * @param[out] vref  ADS1219_VREF_INTERNAL or ADS1219_VREF_EXTERNAL
 * @return status of function
 */
w_status_t ads1219_get_vref(ads1219_handle_t *p_handle, uint8_t *vref);

/**
 * @brief Set the voltage reference and ref voltages.
 * @param p_handle handle to the ADC data
 * @param[in] vref    ADS1219_VREF_INTERNAL or ADS1219_VREF_EXTERNAL
 * @param[in] aref_n  Negative reference in mV (ignored for internal, 0 used)
 * @param[in] aref_p  Positive reference in mV (ignored for internal, 2048 used)
 * @return status of function
 */
w_status_t ads1219_set_vref(ads1219_handle_t *p_handle, uint8_t vref, float64_t aref_n,
							float64_t aref_p);

/**
 * @brief Read the current data-rate setting.
 * @param p_handle handle to the ADC data
 * @param[out] rate  0-3 (ADS1219_DATARATE_*)
 * @return status of function
 */
w_status_t ads1219_get_data_rate(ads1219_handle_t *p_handle, uint8_t *rate);

/**
 * @brief Set the data rate.
 * @param p_handle handle to the ADC data
 * @param[in] rate  0-3 (ADS1219_DATARATE_*)
 * @return status of function
 */
w_status_t ads1219_set_data_rate(ads1219_handle_t *p_handle, uint8_t rate);

/**
 * @brief Read the current conversion-mode setting.
 * @param p_handle handle to the ADC data
 * @param[out] mode  ADS1219_CM_SINGLE_SHOT or ADS1219_CM_CONTINUOUS
 * @return status of function
 */
w_status_t ads1219_get_conversion_mode(ads1219_handle_t *p_handle, uint8_t *mode);

/**
 * @brief Set the conversion mode.
 * @param p_handle handle to the ADC data
 * @param[in] mode  ADS1219_CM_SINGLE_SHOT or ADS1219_CM_CONTINUOUS
 * @return status of function
 */
w_status_t ads1219_set_conversion_mode(ads1219_handle_t *p_handle, uint8_t mode);

/**
 * @brief Check whether a conversion result is ready.
 * @param p_handle handle to the ADC data
 * @param[out] ready  true when a new result is available
 * @return status of function
 */
w_status_t ads1219_conversion_ready(ads1219_handle_t *p_handle, bool *ready);

/**
 * @brief Set the MUX Channel that is used
 * @param p_handle handle to the ADC data
 * @param channel the channel that is used (Macros defined above)
 * @return status of function
 */
w_status_t ads1219_set_channel(ads1219_handle_t *p_handle, uint8_t channel);

/**
 * @brief reads the value of the ads1219 and return the raw value
 * @param p_handle handle to the ADC data
 * @param value is a pointer to where the raw value will be stored
 * @return status of function
 */
w_status_t ads1219_read_value(ads1219_handle_t *p_handle, uint32_t *value);

/**
 * @brief Convert a raw ADC count to millivolts.
 * @param[in]  p_handle    Device handle (carries reference voltages)
 * @param[in]  adc_count Raw signed count from a read function
 * @param[out] mv        Result in millivolts
 * @return status of function
 */
w_status_t ads1219_millivolts(ads1219_handle_t *p_handle, int32_t adc_count, float64_t *mv);

/**
 * @brief Return the converted ADC voltage (millivolts)
 * @param p_handle handle to the ADC data
 * @param data this is pointer to the data return which would be in terms of mV
 * @return status of function
 */
w_status_t ads1219_get_millivolts(ads1219_handle_t *p_handle, float64_t *data);

/**
 * @brief performs a sanity check by reading back the configurations
 * @param p_handle handle to the ADC data
 * @param config_setting is the configuration that is being checked agaisnt
 * @return if the sanity check was successful
 */
w_status_t ads1219_sanity_check(ads1219_handle_t *p_handle, uint8_t config_setting);

#endif /* ADS1219_H */
