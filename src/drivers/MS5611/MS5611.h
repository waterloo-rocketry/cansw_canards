#ifndef MS5611_H
#define MS5611_H

#include <stdbool.h>
#include <stdint.h>

#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"
#include "timers.h"

/* IIC address: CSB pin low = 0x77, CSB pin high = 0x76 */
typedef enum {
	MS5611_ADDRESS_CSB_LOW = 0x77,
	MS5611_ADDRESS_CSB_HIGH = 0x76
} ms5611_address_t;

// reset command
#define MS5611_CMD_RESET 0x1E

// pressure conversion commands (D1)
#define MS5611_CMD_CONVERT_D1_OSR256 0x40
#define MS5611_CMD_CONVERT_D1_OSR512 0x42
#define MS5611_CMD_CONVERT_D1_OSR1024 0x44
#define MS5611_CMD_CONVERT_D1_OSR2048 0x46
#define MS5611_CMD_CONVERT_D1_OSR4096 0x48

// temperature conversion commands (D2)
#define MS5611_CMD_CONVERT_D2_OSR256 0x50
#define MS5611_CMD_CONVERT_D2_OSR512 0x52
#define MS5611_CMD_CONVERT_D2_OSR1024 0x54
#define MS5611_CMD_CONVERT_D2_OSR2048 0x56
#define MS5611_CMD_CONVERT_D2_OSR4096 0x58

// ADC read command
#define MS5611_CMD_ADC_READ 0x00
#define MS5611_CMD_PROM_READ_BASE 0xA0 /* OR with (addr << 1) for addr 0..7 */

// calibration coefficient indices in PROM readout
#define MS5611_COEFF_SENS 1
#define MS5611_COEFF_OFF 2
#define MS5611_COEFF_TCS 3
#define MS5611_COEFF_TCO 4
#define MS5611_COEFF_TREF 5
#define MS5611_COEFF_TEMPSENS 6

// osr index in command arrays
typedef enum {
	MS5611_OSR_256 = 0,
	MS5611_OSR_512 = 1,
	MS5611_OSR_1024 = 2,
	MS5611_OSR_2048 = 3,
	MS5611_OSR_4096 = 4
} ms5611_osr_t;

typedef enum {
	MS5611_STATE_IDLE, // Ready for new measurement
	MS5611_STATE_TEMP_CMD_SENT, // Awaiting timer before temp read
	MS5611_STATE_TEMP_DMA_PENDING, // DMA reading temp ADC
	MS5611_STATE_PRESSURE_CMD_SENT, // Awaiting timer before pressure read
	MS5611_STATE_PRESSURE_DMA_PENDING, // DMA reading pressure ADC
	MS5611_STATE_COMPLETE, // All data ready, math done
	MS5611_STATE_ERROR // Failed transfer
} ms5611_async_state_t;

typedef struct {
	int32_t temperature_centideg; // compensated temperature in centidegrees C
	int32_t pressure_centimbar; // compensated pressure in centimbar
} ms5611_raw_result_t;

typedef struct {
	/* Calibration coefficients read from PROM */
	uint16_t prom_coef[8]; /* C[1]..C[6] used; C[0] = factory reserved empty space */

	i2c_bus_t bus;
	ms5611_address_t addr;

	/* Selected OSR for pressure and temperature conversions */
	ms5611_osr_t osr;

	/* Set true once init succeeds */
	bool initialized;

	/* Async read stuff */
	ms5611_async_state_t async_state;
	uint32_t d1, d2; // Raw ADC buffers
	uint8_t adc_buf[3]; // DMA target buffer for I2C reads
	DMA_HandleTypeDef dma_h; // DMA configuration
	TimerHandle_t conv_timer; // FreeRTOS timer for conversion delays
	ms5611_raw_result_t pending_result; // Compensation result
} ms5611_handle_t;

w_status_t ms5611_init(void);
void ms5611_deinit(void); // for testing purposes, not intended for regular use
w_status_t
ms5611_get_raw_pressure(ms5611_raw_result_t *result); // blocking read of pressure and temperature
													  // with compensation, for testing and fallback

/* Async (non-blocking) APIs for MS5611 measurements */
w_status_t ms5611_start_async_read(void);
bool ms5611_is_ready(void);
w_status_t ms5611_get_async_result(ms5611_raw_result_t *result);
SemaphoreHandle_t ms5611_get_completion_sem(void);

// example async usage pattern:
// void barometer_task(void *pvParameters) {
//     ms5611_raw_result_t result;
//     SemaphoreHandle_t completion_sem = ms5611_get_completion_sem();

//     while (1) {
//         /* Start measurement (returns immediately, <1ms) */
//         if (ms5611_start_async_read() == W_SUCCESS) {
//             /* suspend this task until measurement completes */
//             /* Other tasks continue running during this time */
//             if (xSemaphoreTake(completion_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
//                 /* Get result (microsecond latency) */
//                 if (ms5611_get_async_result(&result) == W_SUCCESS) {
//                     log_text(0, "baro", "Temp: %d centiC, Press: %d centiMb",
//                              result.temperature_centideg,
//                              result.pressure_centimbar);
//                 }
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));  /* Take reading every second */
//     }
// }

#endif // MS5611_H
