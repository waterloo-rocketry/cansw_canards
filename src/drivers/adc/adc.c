#include "drivers/adc/adc.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "semphr.h"

#define ADC_CONV_TIMEOUT_TICKS pdMS_TO_TICKS(1)
#define V_REF 2.5f
#define ADC1_MAX_COUNTS 0xFFFF // 16 bit full scale, configured in ioc
#define ADC1_NUM_CHANNELS 5
#define ADC2_MAX_COUNTS 0xFFFF // 16 bit full scale, configured in ioc
#define ADC2_NUM_CHANNELS 2
#define ADC3_MAX_COUNTS 0x0FFF // 12 bit full scale, configured in ioc
#define ADC3_NUM_CHANNELS 2

// Active DMA buffers
static uint16_t adc1_dma_counts[ADC1_NUM_CHANNELS];
static uint16_t adc2_dma_counts[ADC2_NUM_CHANNELS];
static uint16_t adc3_dma_counts[ADC3_NUM_CHANNELS];

typedef struct {
	uint16_t *buffer; // DMA buffer
	uint8_t rank; // rank index (see cubemx)
	float conv_factor; // circuit scaling factor
	uint32_t max_counts;
} adc_channel_desc_t;

static adc_channel_desc_t adc_map[ADC_CHANNEL_COUNT] = {
	[VSENS_BAT1] = {adc1_dma_counts, 0, 11.0f, ADC1_MAX_COUNTS},
	[VSENS_BAT2] = {adc1_dma_counts, 1, 11.0f, ADC1_MAX_COUNTS},
	[VSENS_RKT] = {adc1_dma_counts, 2, 6.2356f, ADC1_MAX_COUNTS},
	[ISENS_BAT2] = {adc1_dma_counts, 3, 11111.1f, ADC1_MAX_COUNTS},
	[ISENS_BAT1] = {adc1_dma_counts, 4, 11111.1f, ADC1_MAX_COUNTS},
	[VSENS_CHG] = {adc2_dma_counts, 0, 6.2356f, ADC2_MAX_COUNTS},
	[VSENS_USB] = {adc2_dma_counts, 1, 2.0f, ADC2_MAX_COUNTS},
	[ISENS_3V3] = {adc3_dma_counts, 0, 2000.0f, ADC3_MAX_COUNTS},
	[ISENS_5V] = {adc3_dma_counts, 1, 2000.0f, ADC3_MAX_COUNTS},
};

static ADC_HandleTypeDef *adc1_handle;
static ADC_HandleTypeDef *adc2_handle;
static ADC_HandleTypeDef *adc3_handle;

static adc_error_data_t adc_error_stats = {0};

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

	// Start ADC continuous conversion and circular DMA
	if (HAL_OK != HAL_ADC_Start_DMA(adc1_handle, (uint32_t *)adc1_dma_counts, ADC1_NUM_CHANNELS) ||
		HAL_OK != HAL_ADC_Start_DMA(adc2_handle, (uint32_t *)adc2_dma_counts, ADC2_NUM_CHANNELS) ||
		HAL_OK != HAL_ADC_Start_DMA(adc3_handle, (uint32_t *)adc3_dma_counts, ADC3_NUM_CHANNELS)) {
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

/**
 * @brief Get the raw ADC counts of a channel from DMA buffer
 * @param channel The adc channel to read from
 * @param output Pointer to store raw counts value
 * @return Status of the read operation
 */
static w_status_t adc_get_raw_counts(adc_channel_t channel, uint32_t *output) {
	if (channel >= ADC_CHANNEL_COUNT) {
		adc_error_stats.invalid_channels++;
		return W_INVALID_PARAM;
	}

	const adc_channel_desc_t *adc_desc = &adc_map[channel];

	if (NULL == adc_desc->buffer) {
		adc_error_stats.invalid_channels++;
		return W_INVALID_PARAM;
	}

	*output = adc_desc->buffer[adc_desc->rank];

	if (*output > adc_desc->max_counts) {
		adc_error_stats.overflow_errors++;
		return W_OVERFLOW;
	}

	return W_SUCCESS;
}

/**
 * @brief Convert the raw counts of a channel into voltage
 * @param channel The adc channel to read from
 * @param output Pointer to store output value of the ADC channel
 * @return Status of the read operation
 */
static w_status_t adc_get_raw_volts(adc_channel_t channel, float *output) {
	uint32_t counts = 0;
	w_status_t status = adc_get_raw_counts(channel, &counts);
	if (status != W_SUCCESS) {
		return status;
	}

	const adc_channel_desc_t *adc_desc = &adc_map[channel];

	*output = ((float)counts / adc_desc->max_counts) * V_REF;

	return W_SUCCESS;
}

w_status_t adc_get_converted_val(adc_channel_t channel, float *output) {
	float raw_volts = 0;
	w_status_t status = adc_get_raw_volts(channel, &raw_volts);
	if (status != W_SUCCESS) {
		return status;
	}

	const adc_channel_desc_t *adc_desc = &adc_map[channel];

	*output = raw_volts * adc_desc->conv_factor;
	return W_SUCCESS;
}

health_status_t adc_get_status(void) {
	uint32_t status_bitfield = 0;

	// Log error statistics
	log_text(0,
			 "adc",
			 "%s conv_timeouts=%lu, invalid_channels=%lu, "
			 "overflows=%lu",
			 adc_error_stats.is_init ? "true" : "false",
			 adc_error_stats.conversion_timeouts,
			 adc_error_stats.invalid_channels,
			 adc_error_stats.overflow_errors);

	health_status_t status = {.severity = HEALTH_OK, .module_id = MODULE_ADC, .error_bitfield = 0};

	return status;
}

