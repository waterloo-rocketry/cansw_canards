#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
    #include "FreeRTOS.h"
    #include "application/logger/log.h"
    #include "drivers/MS5611/MS5611.h"
    #include "drivers/gpio/gpio.h"
    #include "drivers/i2c/i2c.h"
}
