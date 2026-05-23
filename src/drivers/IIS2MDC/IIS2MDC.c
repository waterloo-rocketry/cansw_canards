#include "drivers/IIS2MDC/IIS2MDC.h"
#include "drivers/i2c/i2c.h"

// Register addresses
#define IIS2MDC_REG_OFFSET_X_L 0x45 // add other axes later
#define IIS2MDC_REG_WHO_AM_I 0x4F
#define IIS2MDC_REG_CFG_A 0x60
#define IIS2MDC_REG_CFG_B 0x61
#define IIS2MDC_REG_CFG_C 0x62
#define IIS2MDC_REG_STATUS 0x67
#define IIS2MDC_REG_OUTX_L 0x68

w_status_t iis2mdc_init(void){
	return W_SUCCESS;
}
w_status_t iis2mdc_check_sanity(void){
	return W_SUCCESS;
}
w_status_t iis2mdc_get_data(iis2mdc_data_t *data, iis2mdc_raw_data_t *raw_data){
	return W_SUCCESS;
}