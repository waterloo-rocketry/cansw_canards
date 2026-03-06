
#include "example_module.h"
#include "application/logger/log.h"

health_status_t example_module_get_status(void) {
    uint16_t example_condition = 0;

    //Fatal errors
	if (!example_condition) {
		return HEALTH_STATUS(HEALTH_FATAL, 
		                     MODULE_EXAMPLE,
		                     EXAMPLE_ERR_NOT_INITIALIZED);
	}
	
	if (!example_condition) {
		return HEALTH_STATUS(HEALTH_FATAL,
		                     MODULE_EXAMPLE,
		                     EXAMPLE_ERR_HARDWARE_FAULT);
	}
	
	//Warnings
	if (!example_condition) {
		return HEALTH_STATUS(HEALTH_WARN,
		                     MODULE_EXAMPLE,
		                     EXAMPLE_ERR_TIMEOUT);
	}
	
	return HEALTH_STATUS_OK(MODULE_EXAMPLE); 
}