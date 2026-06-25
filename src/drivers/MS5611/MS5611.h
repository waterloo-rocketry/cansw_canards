#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>

#include "rocketlib/include/common.h"

typedef struct {
	int32_t temperature_centideg; /* °C × 100, e.g. 2007 = 20.07°C  */
	int32_t pressure_centimbar; /* mbar × 100, e.g. 100009 = 1000.09 mbar */
} ms5611_raw_result_t;

w_status_t ms5611_init(void);
void ms5611_deinit(void);

/**
 * @brief Non-blocking getter. Returns the most recent pressure/temperature sample collected by
 * ms5611_task. Does not perform any I2C or conversion delay.
 * @param result Pointer to store the latest raw pressure and temperature results
 * @param timestamp_ms Pointer to store the timestamp of the latest sample
 * @return W_SUCCESS if a valid sample was copied, W_FAILURE otherwise (no sample yet, last read
 * failed, or sampling task currently updating)
 */
w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result, uint32_t *timestamp_ms);

/**
 * @brief FreeRTOS task that periodically performs the blocking pressure/temperature read and caches
 * the latest result for ms5611_get_raw_pressure.
 */
void ms5611_task(void *argument);

#endif // MS5611_H
