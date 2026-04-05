#ifndef MS5611_H
#define MS5611_H

#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

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

typedef struct {
	/* Calibration coefficients read from PROM */
	uint16_t C[8]; /* C[1]..C[6] used; C[0] = factory reserved empty space */

	i2c_bus_t bus;
	ms5611_address_t addr;

	/* Selected OSR for pressure and temperature conversions */
	ms5611_osr_t osr;

	/* Set true once init succeeds */
	bool initialized;
} ms5611_handle_t;

typedef struct {
	int32_t temperature_centideg; /* °C × 100, e.g. 2007 = 20.07°C  */
	int32_t pressure_centimbar; /* mbar × 100, e.g. 100009 = 1000.09 mbar */
} ms5611_raw_result_t;

w_status_t ms5611_init(void);
w_status_t ms5611_check_sanity(void);
w_status_t ms5611_get_raw_pressure(ms5611_raw_result_t *result);

#endif // MS5611_H
