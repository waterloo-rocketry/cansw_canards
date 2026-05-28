#include "drivers/adc/adc.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "semphr.h"

#define ADC_CONV_TIMEOUT_TICKS pdMS_TO_TICKS(1)
#define V_REF 3.3f // check this later
#define ADC1_NUM_CHANNELS 5
#define ADC2_NUM_CHANNELS 2
#define ADC3_NUM_CHANNELS 2

// DMA buffers
static uint32_t adc1_raw_counts[ADC1_NUM_CHANNELS];
static uint32_t adc2_raw_counts[ADC2_NUM_CHANNELS];
static uint32_t adc3_raw_counts[ADC3_NUM_CHANNELS];

static ADC_HandleTypeDef *adc1_handle;
static ADC_HandleTypeDef *adc2_handle;
static ADC_HandleTypeDef *adc3_handle;

static adc_error_data_t adc_error_stats = {0};

static const uint32_t channel_to_dma_index[ADC_CHANNEL_COUNT] = {[VSENS_BAT1] = 0,
																 [VSENS_BAT2] = 1,
																 [VSENS_RKT] = 2,
																 [ISENS_BAT2] = 3,
																 [ISENS_BAT1] = 4,
																 [VSENS_CHG] = 0,
																 [VSENS_USB] = 1,
																 [ISENS_3V3] = 0,
																 [ISENS_5V] = 1};

static const uint32_t channel_to_adc[ADC_CHANNEL_COUNT] = {[VSENS_BAT1] = 1,
														   [VSENS_BAT2] = 1,
														   [VSENS_RKT] = 1,
														   [ISENS_BAT2] = 1,
														   [ISENS_BAT1] = 1,
														   [VSENS_CHG] = 2,
														   [VSENS_USB] = 2,
														   [ISENS_3V3] = 3,
														   [ISENS_5V] = 3};

static const float conversion_table[ADC_CHANNEL_COUNT] = {
	// temporarily values for scaling multiplier
	[VSENS_BAT1] = 0,
	[VSENS_BAT2] = 0,
	[VSENS_RKT] = 0,
	[ISENS_BAT2] = 0,
	[ISENS_BAT1] = 0,
	[VSENS_CHG] = 0,
	[VSENS_USB] = 0,
	[ISENS_3V3] = 0,
	[ISENS_5V] = 0,
};

w_status_t adc_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, ADC_HandleTypeDef *hadc3) {
	if (NULL == hadc1 || NULL == hadc2 || NULL == hadc3) {
		return W_INVALID_PARAM;
	}

	adc1_handle = hadc1;
	adc2_handle = hadc2;
	adc3_handle = hadc3;

	if (HAL_OK != HAL_ADCEx_Calibration_Start(adc1_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) ||
		HAL_OK != HAL_ADCEx_Calibration_Start(adc2_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) ||
		HAL_OK != HAL_ADCEx_Calibration_Start(adc3_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)) {
		log_text(1, "adc", "initfailcal");
		return W_FAILURE;
	}

	// Start continuous DMA
	if (HAL_OK != HAL_ADC_Start_DMA(adc1_handle, adc1_raw_counts, ADC1_NUM_CHANNELS) ||
		HAL_OK != HAL_ADC_Start_DMA(adc2_handle, adc2_raw_counts, ADC2_NUM_CHANNELS) ||
		HAL_OK != HAL_ADC_Start_DMA(adc3_handle, adc3_raw_counts, ADC3_NUM_CHANNELS)) {
		log_text(1, "adc", "initfaildma");
		return W_FAILURE;
	}

	// Initialize error tracking
	adc_error_stats.is_init = true;
	adc_error_stats.conversion_timeouts = 0;
	adc_error_stats.invalid_channels = 0;
	adc_error_stats.overflow_errors = 0;

	return W_SUCCESS;
}

static w_status_t adc_get_raw_counts(adc_channel_t channel, uint32_t *output) {
	if (channel >= ADC_CHANNEL_COUNT) {
		adc_error_stats.invalid_channels++;
		return W_INVALID_PARAM;
	}

	uint32_t index = channel_to_dma_index[channel];
	uint32_t adc = channel_to_adc[channel];

	if (1 == adc) {
		*output = adc1_raw_counts[index];
	} else if (2 == adc) {
		*output = adc2_raw_counts[index];
	} else {
		*output = adc3_raw_counts[index];
	}

	if (*output > ADC_MAX_COUNTS) {
		adc_error_stats.overflow_errors++;
		return W_OVERFLOW;
	}

	return W_SUCCESS;
}

w_status_t adc_get_raw_volts(adc_channel_t channel, uint32_t *output) {
	uint32_t counts = 0;
	w_status_t status = adc_get_raw_counts(channel, &counts);
	if (status != W_SUCCESS) {
		return status;
	}

	*output = ((float)counts / ADC_MAX_COUNTS) * V_REF;
	return W_SUCCESS;
}

w_status_t adc_get_converted_val(adc_channel_t channel, uint32_t *output) {
	uint32_t raw_volts = 0;
	w_status_t status = adc_get_raw_volts(channel, &raw_volts);
	if (status != W_SUCCESS) {
		return status;
	}

	*output = raw_volts * conversion_table[channel];
	return W_SUCCESS;
}

uint32_t adc_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log error statistics
	log_text(0,
			 "adc",
			 "%s conv_timeouts=%lu, mutex_timeouts=%lu, invalid_channels=%lu, "
			 "overflows=%lu",
			 adc_error_stats.is_init ? "true" : "false",
			 adc_error_stats.conversion_timeouts,
			 adc_error_stats.invalid_channels,
			 adc_error_stats.overflow_errors);

	return status_bitfield;
}

