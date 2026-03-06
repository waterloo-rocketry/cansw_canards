#include "application/logger/log.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/imus/LSM6DSV32X_regmap.h"
#include <limits.h>
#include <stdio.h>

static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief Flips and checks the bit register on that allows access to the controll registers
 * @note Must be called before any bit registers are configured
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_config_open();

/**
 * @brief Flips and checks the bit register off that allows access to the controll registers
 * @note Must be called after bit registers are configured, called before flight!!!
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_config_close();

/**
 * @brief Initializes the bit registers for lsm6dsv32x
 * @note Must be called to wake up imu
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init();

