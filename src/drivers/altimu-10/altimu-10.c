#include "drivers/altimu-10/altimu-10.h"
#include "application/logger/log.h"
#include "drivers/altimu-10/LIS3MDL_regmap.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/LSM6DSO_regmap.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include <limits.h>
#include <stdio.h>

// AltIMU device addresses and configuration
#define LSM6DSO_ADDR 0x6B // addr sel pin HIGH IMU
#define LIS3MDL_ADDR 0x1E // addr sel pin HIGH Mag
#define LPS22DF_ADDR 0x5D // addr sel pin HIGH Baro

// sensor ranges. these must be selected using the i2c init regs
static const double ALTIMU_ACC_MAX = 16.0; // g
static const double ALTIMU_GYRO_MAX = 2000.0; // dps
static const double ALTIMU_MAG_MAX = 16.0; // gauss

// AltIMU conversion factors - based on config settings below
static const double ACC_FS = ALTIMU_ACC_MAX / INT16_MAX; // g / LSB
static const double GYRO_FS = ALTIMU_GYRO_MAX / INT16_MAX; // dps / LSB
static const double MAG_FS = ALTIMU_MAG_MAX / INT16_MAX; // gauss / LSB
static const double BARO_FS = 100.0 / 4096.0; // fixed scale: 100 Pa / 4096 LSB
static const double TEMP_FS = 1.0 / 100.0; // fixed scale: 1 deg C / 100 LSB

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    return i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
}

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return Status of the operation.
 * TODO: improve error code verbosity?
 */
w_status_t altimu_check_sanity(void) {
    w_status_t i2c_status = W_SUCCESS;
    w_status_t device_status = W_SUCCESS;

    const uint8_t expected_lis3mdl = 0x3D;
    const uint8_t expected_lps22df = 0xB4;
    const uint8_t expected_lsm6dso = 0x6C;

    uint8_t who_am_i;
    i2c_status |= i2c_read_reg(I2C_BUS_4, LIS3MDL_ADDR, LIS3_WHO_AM_I, &who_am_i, 1);
    if (expected_lis3mdl != who_am_i) {
        device_status |= W_FAILURE;
    }

    i2c_status |= i2c_read_reg(I2C_BUS_4, LPS22DF_ADDR, LPS_WHO_AM_I, &who_am_i, 1);
    if (expected_lps22df != who_am_i) {
        device_status |= W_FAILURE;
    }

    i2c_status |= i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, WHO_AM_I_REG, &who_am_i, 1);
    if (expected_lsm6dso != who_am_i) {
        device_status |= W_FAILURE;
    }

    if ((i2c_status != W_SUCCESS) || (device_status != W_SUCCESS)) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

/**
 * @brief Initializes the Pololu AltIMU incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t altimu_init() {
    w_status_t status = W_SUCCESS;

    // Drive addr sel pin HIGH to use each device's "default" i2c addr
    status |= gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);

    // LSM6DSO: https://www.st.com/resource/en/datasheet/lsm6dso.pdf

    // Accel ODR: 208 Hz
    // Accel Fs: max  (+-16g)
    // LPF2_XL_EN: disable 2nd lowpass stage
    // Note: need to set XL_FS_MODE = 0
    status |= write_1_byte(LSM6DSO_ADDR, CTRL1_XL, 0x54);

    // Gyro ODR: 208 Hz
    // Gyro Fs: max (+-2000 dps)
    status |= write_1_byte(LSM6DSO_ADDR, CTRL2_G, 0x5C);

    // BDU Enable
    status |= write_1_byte(LSM6DSO_ADDR, CTRL3_C, 0x44);

    // Gyro LPF1 Bw: 67 Hz
    status |= write_1_byte(LSM6DSO_ADDR, CTRL6_C, 0x00);

    // XL_FS_MODE = 0
    // Accel Lowpass Bandwidth = ODR/2 = 104 Hz
    status |= write_1_byte(LSM6DSO_ADDR, CTRL8_XL, 0x0);

    // Set bit 1 to 1 when not using I3C, recommended by datasheet
    status |= write_1_byte(LSM6DSO_ADDR, CTRL9_XL, 0x02);

    // LIS3MDL: https://www.pololu.com/file/0J1089/LIS3MDL.pdf

    // Enable Fast ODR (>80 Hz)
    // Mag ODR: 155 Hz
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG1, 0x62);

    // Mag Fs: max (+- 16 gauss)
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG2, 0x60);

    // Continuous conversion mode
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG3, 0x0);

    // BDU enable
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG5, 0x40);

    // LPS22DF: https://www.pololu.com/file/0J1961/lps22df.pdf

    // ODR: max (200 Hz)
    //  Averaging: 8 samples
    status |= write_1_byte(LPS22DF_ADDR, LPS22DF_CTRL_REG1, 0x41);

    // LPF EN
    // LPF cutoff = ODR/4 = 50 Hz
    // BDU enable
    status |= write_1_byte(LPS22DF_ADDR, LPS22DF_CTRL_REG2, 0x18);

    if (status != W_SUCCESS) {
        log_text(1, "altimu", "initfail");
    }

    return status;
}

/**
 * @brief Retrieves accelerometer data.
 * @return Accelerometer data (gravities)
 */
w_status_t altimu_get_acc_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data) {
    uint8_t raw_bytes[6];
    w_status_t status = i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, OUTX_L_A, raw_bytes, 6);
    if (W_SUCCESS == status) {
        raw_data->x = (uint16_t)(((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
        raw_data->y = (uint16_t)(((uint16_t)raw_bytes[3] << 8) | raw_bytes[2]);
        raw_data->z = (uint16_t)(((uint16_t)raw_bytes[5] << 8) | raw_bytes[4]);
        data->x = (int16_t)raw_data->x * ACC_FS;
        data->y = (int16_t)raw_data->y * ACC_FS;
        data->z = (int16_t)raw_data->z * ACC_FS;
    }
    return status;
}

/**
 * @brief Retrieves gyroscope data.
 * @return Gyroscope data (deg/s)
 */
w_status_t altimu_get_gyro_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data) {
    uint8_t raw_bytes[6];
    w_status_t status = i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, OUTX_L_G, raw_bytes, 6);
    if (W_SUCCESS == status) {
        raw_data->x = (uint16_t)(((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
        raw_data->y = (uint16_t)(((uint16_t)raw_bytes[3] << 8) | raw_bytes[2]);
        raw_data->z = (uint16_t)(((uint16_t)raw_bytes[5] << 8) | raw_bytes[4]);
        data->x = (int16_t)raw_data->x * GYRO_FS;
        data->y = (int16_t)raw_data->y * GYRO_FS;
        data->z = (int16_t)raw_data->z * GYRO_FS;
    }
    return status;
}

/**
 * @brief Retrieves both gyroscope and accelerometer data in one I2C transaction.
 * @param[out] acc_data    Processed accelerometer data (gravities)
 * @param[out] gyro_data   Processed gyroscope data (deg/s)
 * @param[out] raw_acc     Raw accelerometer data
 * @param[out] raw_gyro    Raw gyroscope data
 * @return Status of I2C read
 */
w_status_t altimu_get_gyro_acc_data(
    vector3d_t *acc_data, vector3d_t *gyro_data, altimu_raw_imu_data_t *raw_acc,
    altimu_raw_imu_data_t *raw_gyro
) {
    // Drive addr sel pin HIGH to use each device's "default" i2c addr
    w_status_t status = gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);

    uint8_t raw_bytes[12];
    status |= i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, OUTX_L_G, raw_bytes, 12);
    if (W_SUCCESS == status) {
        // Parse gyroscope raw data (first 6 bytes)
        raw_gyro->x = (uint16_t)(((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
        raw_gyro->y = (uint16_t)(((uint16_t)raw_bytes[3] << 8) | raw_bytes[2]);
        raw_gyro->z = (uint16_t)(((uint16_t)raw_bytes[5] << 8) | raw_bytes[4]);

        // Parse accelerometer raw data (next 6 bytes)
        raw_acc->x = (uint16_t)(((uint16_t)raw_bytes[7] << 8) | raw_bytes[6]);
        raw_acc->y = (uint16_t)(((uint16_t)raw_bytes[9] << 8) | raw_bytes[8]);
        raw_acc->z = (uint16_t)(((uint16_t)raw_bytes[11] << 8) | raw_bytes[10]);

        // Convert to physical units
        gyro_data->x = (int16_t)raw_gyro->x * GYRO_FS;
        gyro_data->y = (int16_t)raw_gyro->y * GYRO_FS;
        gyro_data->z = (int16_t)raw_gyro->z * GYRO_FS;

        acc_data->x = (int16_t)raw_acc->x * ACC_FS;
        acc_data->y = (int16_t)raw_acc->y * ACC_FS;
        acc_data->z = (int16_t)raw_acc->z * ACC_FS;
    }

    // lsm6dso gyro range: +- 2000 dps. accel range: +- 16g
    if (fabs(gyro_data->x) > 2000.0 || fabs(gyro_data->y) > 2000.0 || fabs(gyro_data->z) > 2000.0 ||
        fabs(acc_data->x) > 16.0 || fabs(acc_data->y) > 16.0 || fabs(acc_data->z) > 16.0) {
        status = W_IO_ERROR;
    }
    return status;
}

/**
 * @brief Retrieves magnetometer data.
 * @return Magnetometer data (gauss)
 */
w_status_t altimu_get_mag_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data) {
    // Drive addr sel pin HIGH to use each device's "default" i2c addr
    w_status_t status = gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);

    uint8_t raw_bytes[6];
    status |= i2c_read_reg(I2C_BUS_4, LIS3MDL_ADDR, LIS3_OUT_X_L, raw_bytes, 6);
    if (W_SUCCESS == status) {
        raw_data->x = (uint16_t)(((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
        raw_data->y = (uint16_t)(((uint16_t)raw_bytes[3] << 8) | raw_bytes[2]);
        raw_data->z = (uint16_t)(((uint16_t)raw_bytes[5] << 8) | raw_bytes[4]);
        data->x = (int16_t)raw_data->x * MAG_FS;
        data->y = (int16_t)raw_data->y * MAG_FS;
        data->z = (int16_t)raw_data->z * MAG_FS;
    }

    // lis3mdl mag range: +- 16 gauss
    if (fabs(data->x) > ALTIMU_MAG_MAX || fabs(data->y) > ALTIMU_MAG_MAX ||
        fabs(data->z) > ALTIMU_MAG_MAX) {
        status = W_IO_ERROR;
    }
    return status;
}

/**
 * @brief Retrieves barometer data.
 * @return Barometer data (pascal, celsius)
 */
w_status_t altimu_get_baro_data(altimu_barometer_data_t *data, altimu_raw_baro_data_t *raw_data) {
    // Drive addr sel pin HIGH to use each device's "default" i2c addr
    w_status_t status = gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);

    uint8_t raw_bytes[5];
    status |= i2c_read_reg(I2C_BUS_4, LPS22DF_ADDR, LPS_PRESS_OUT_XL, raw_bytes, 5);
    if (W_SUCCESS == status) {
        raw_data->pressure = (uint32_t)(((uint32_t)raw_bytes[2] << 16) |
                                        ((uint16_t)raw_bytes[1] << 8) | raw_bytes[0]);
        raw_data->temperature = (uint16_t)(((uint16_t)raw_bytes[4] << 8) | raw_bytes[3]);
        data->pressure = (int32_t)raw_data->pressure * BARO_FS;
        data->temperature = (int16_t)raw_data->temperature * TEMP_FS;
    }
    return status;
}
