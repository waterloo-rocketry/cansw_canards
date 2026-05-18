#ifndef IIS2MDC_H
#define IIS2MDC_H

#include "rocketlib/include/common.h"
#include <stdint.h>

// I2C slave address (7 bit)
#define IIS2MDC_I2C_ADDR 0x1E

// Expected WHO_AM_I value for sanity checking
#define IIS2MDC_WHO_AM_I_VAL 0x40

// Register addresses
#define IIS2MDC_REG_OFFSET_X_L 0x45 // add other axes later
#define IIS2MDC_REG_WHO_AM_I 0x4F
#define IIS2MDC_REG_CFG_A 0x60
#define IIS2MDC_REG_CFG_B 0x61
#define IIS2MDC_REG_CFG_C 0x62
#define IIS2MDC_REG_STATUS 0x67
#define IIS2MDC_REG_OUTX_L 0x68 // add other axes later

// Status register for when a new set of data available
#define IIS2MDC_STATUS_ZYXDA (1 << 3)

// Resets config registers
#define IIS2MDC_CFG_A_SOFT_RESET (1 << 5)

/* Init configuration:
 CFG_REG_A = 0x8C  COMP_TEMP_EN=1, LP=0(high-res), ODR=11 (100 Hz), MD=00 (continuous)
 CFG_REG_B = 0x02  OFF_CANC=1 (continuous offset cancellatioon)
 CFG_REG_C = 0x10  BDU=1 (block data update, DRDY pin unused, host polls status register)
 */
#define IIS2MDC_INIT_CFG_A 0x8C
#define IIS2MDC_INIT_CFG_B 0x02
#define IIS2MDC_INIT_CFG_C 0x10

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} iis2mdc_raw_data_t;

w_status_t iis2mdc_init(void);
w_status_t iis2mdc_check_sanity(void);
w_status_t iis2mdc_get_data(iis2mdc_raw_data_t *out);

#endif // IIS2MDC_H
