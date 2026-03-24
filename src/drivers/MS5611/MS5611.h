#ifndef MS5611_H
#define MS5611_H

#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include <stdbool.h>
#include <stdint.h>
// MS5611 I2C address
#define MS5611_ADDR 0x77

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

// i2c interface struct
typedef struct {
	void (*iic_write)(uint8_t byte);
	uint8_t (*iic_read)(void);
	void (*delay_us)(uint32_t us);
} ms5611_i2c_interface_t;

// osr options
typedef enum {
	MS5611_OSR_256 = 0,
	MS5611_OSR_512 = 1,
	MS5611_OSR_1024 = 2, // we are using 1024 OSR for now
	MS5611_OSR_2048 = 3,
	MS5611_OSR_4096 = 4
} ms5611_osr_t;

/* Conversion time in microseconds - max values from datasheet for safety */
static const uint32_t MS5611_CONV_TIME_US[] = {
	600, /* OSR 256  — datasheet max 0.60ms */
	1170, /* OSR 512  — datasheet max 1.17ms */
	2280,
	/* OSR 1024 — datasheet max 2.28ms */ // this one is picked
	4540, /* OSR 2048 — datasheet max 4.54ms */
	9040, /* OSR 4096 — datasheet max 9.04ms */
};

typedef struct {
	/* Calibration coefficients read from PROM */
	uint16_t C[7]; /* C[1]..C[6] used; C[0] = factory reserved empty space */

	// /* HAL function pointers */
	// void     (*spi_cs_low)(void);
	// void     (*spi_cs_high)(void);
	ms5611_i2c_interface_t i2c_interface;

	/* Selected OSR for pressure and temperature conversions */
	ms5611_osr_t osr;

	/* Set true once init succeeds */
	bool initialized;
} ms5611_t;

typedef struct {
	int32_t temperature_centideg; /* °C × 100, e.g. 2007 = 20.07°C  */
	int32_t pressure_centimbar; /* mbar × 100, e.g. 100009 = 1000.09 mbar */
} ms5611_raw_result_t;

w_status_t ms5611_init(ms5611_t *dev, ms5611_osr_t osr);
w_status_t ms5611_read(ms5611_t *dev, ms5611_raw_result_t *out);

#endif