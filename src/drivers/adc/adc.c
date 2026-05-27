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

static ADC_HandleTypeDef *adc_handle;
static SemaphoreHandle_t adc_conversion_semaphore = NULL;
static SemaphoreHandle_t adc_mutex = NULL;
static adc_error_data_t adc_error_stats = {0};

static void ADC1_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	(void)hadc;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* We have not woken a task at the start of the ISR. */
	xSemaphoreGiveFromISR(adc_conversion_semaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static const uint8_t channel_to_dma_index[ADC_CHANNEL_COUNT-1] = {
    [VSENS_BAT1]  = 0, 
    [VSENS_BAT2]  = 1, 
    [VSENS_RKT]   = 2, 
    [ISENS_BAT2]  = 3, 
    [ISENS_BAT1]  = 4, 
    [VSENS_CHG]   = 0, 
    [VSENS_USB]   = 1, 
    [ISENS_3V3]   = 0,
    [ISENS_5V]    = 1,
};

static const uint8_t channel_to_adc[ADC_CHANNEL_COUNT-1] = {
    [VSENS_BAT1]  = 1,
    [VSENS_BAT2]  = 1,
    [VSENS_RKT]   = 1,
    [ISENS_BAT2]  = 1,
    [ISENS_BAT1]  = 1,
    [VSENS_CHG]   = 2,
    [VSENS_USB]   = 2,
    [ISENS_3V3]   = 3,
    [ISENS_5V]    = 3,
};

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
	[PROCESSOR_BOARD_VOLTAGE] = 0};

w_status_t adc_init(ADC_HandleTypeDef *hadc) {
	if (NULL == hadc) {
		return W_INVALID_PARAM;
	}
	adc_handle = hadc;

	adc_conversion_semaphore = xSemaphoreCreateBinary();
	adc_mutex = xSemaphoreCreateMutex();

	if ((NULL == adc_mutex) || (NULL == adc_conversion_semaphore)) {
		if (adc_mutex) {
			vSemaphoreDelete(adc_mutex);
		}
		if (adc_conversion_semaphore) {
			vSemaphoreDelete(adc_conversion_semaphore);
		}
		log_text(1, "adc", "initfailmtx");
		return W_FAILURE;
	}

	if (HAL_OK != HAL_ADC_RegisterCallback(
					  adc_handle, HAL_ADC_CONVERSION_COMPLETE_CB_ID, ADC1_ConvCpltCallback)) {
		vSemaphoreDelete(adc_mutex);
		vSemaphoreDelete(adc_conversion_semaphore);
		log_text(1, "adc", "initfailcb");
		return W_FAILURE;
	}

	// Run ADC hardware auto-calibration
	if (HAL_OK != HAL_ADCEx_Calibration_Start(adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)) {
		vSemaphoreDelete(adc_mutex);
		vSemaphoreDelete(adc_conversion_semaphore);
		log_text(1, "adc", "initfailcal");
		return W_FAILURE;
	}

	// Initialize error tracking
	adc_error_stats.is_init = true;
	adc_error_stats.conversion_timeouts = 0;
	adc_error_stats.mutex_timeouts = 0;
	adc_error_stats.invalid_channels = 0;
	adc_error_stats.overflow_errors = 0;

	return W_SUCCESS;
}

static w_status_t adc_get_raw_counts(adc_channel_t channel, uint32_t *output, uint32_t timeout_ms) {
	if (channel >= ADC_CHANNEL_COUNT-1) {
		adc_error_stats.invalid_channels++;
		return W_INVALID_PARAM;
	}

	uint32_t index = channel_to_dma_index[channel];
	uint32_t adc = channel_to_adc[channel];

	if (1 == adc) {
		*output = adc1_raw_counts[index];
	}
	else if (2 == adc) {
		*output = adc2_raw_counts[index];
	}
	else {
		*output = adc3_raw_counts[index];
	}

	if (*output > ADC_MAX_COUNTS) {
		adc_error_stats.overflow_errors++;
		return W_OVERFLOW;
	}

	return W_SUCCESS;
}

w_status_t adc_get_raw_volts(adc_channel_t channel, uint32_t *output, uint32_t timeout_ms) {
	uint32_t counts = 0;
	w_status_t status = adc_get_raw_counts(channel, &counts, 0);
	if (status != W_SUCCESS) {
		return status;
	}

	*output = ((float)counts / ADC_MAX_COUNTS) * V_REF;
	return W_SUCCESS;
}

w_status_t adc_get_converted_val(adc_channel_t channel, uint32_t *output) {
	uint32_t raw_volts = 0;
	w_status_t status = adc_get_raw_volts(channel, &raw_volts, 0);
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
			 adc_error_stats.mutex_timeouts,
			 adc_error_stats.invalid_channels,
			 adc_error_stats.overflow_errors);

	return status_bitfield;
}

