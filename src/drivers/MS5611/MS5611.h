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
w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result, uint32_t *timestamp_ms);

#endif // MS5611_H
