#ifndef ADC_H
#define ADC_H

#include "application/health_checks/health_checks.h"
#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
	VSENS_BAT1 = 0,
	VSENS_BAT2,
	VSENS_RKT,
	ISENS_BAT2,
	ISENS_BAT1,
	VSENS_CHG,
	VSENS_USB,
	ISENS_3V3,
	ISENS_5V,
	ADC_CHANNEL_COUNT
} adc_channel_t;

/**
 * @brief Structure to track ADC errors and status
 */
typedef struct {
	bool is_init; /**< Initialization status flag */
	uint32_t conversion_timeouts; /**< Count of ADC conversion timeouts */
	uint32_t invalid_channels; /**< Count of attempts to read invalid channels */
	uint32_t overflow_errors; /**< Count of ADC value overflow errors */
} adc_error_data_t;

/**
 * @brief Initialize the ADC driver
 * @param hadc Pointer to the HAL ADC handle
 * @return Status of the initialization
 */
w_status_t adc_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, ADC_HandleTypeDef *hadc3);

/**
 * @brief Convert the raw voltage into actual value it measures (empty stub for now)
 * Units of conversion
 *   - VSENS_ channels & PROCESSOR_BOARD_VOLTAGE: volts (V)
 *   - ISENS_ channels: milliamps (mA)
 * @param channel The adc channel to read from
 * @param output Pointer to store converted value
 * @return Status of the read operation
 */
w_status_t adc_get_converted_val(adc_channel_t channel, float *output);

/**
 * @brief Report ADC module health status
 *
 * Retrieves and reports ADC error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
health_status_t adc_get_status(void);

#endif
