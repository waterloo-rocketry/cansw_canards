#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"

// i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len);
FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *, uint8_t)

// i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len);
FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)

// w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);

#define LSM6DSO_ADDR 0x6B // default (addr sel pin vdd) IMU
#define LIS3MDL_ADDR 0x1E // default (addr sel pin vdd) Mag
#define LPS22DF_ADDR 0x5D // default (addr sel pin vdd) Baro

// Successful WHOMAI read
static w_status_t i2c_read_reg_custom_fake1(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    if (device_addr == LIS3MDL_ADDR) {
        *data = 0x3D;
    } else if (device_addr == LPS22DF_ADDR) {
        *data = 0xB4;
    } else if (device_addr == LSM6DSO_ADDR) {
        *data = 0x6C;
    }
    return W_SUCCESS;
};

// Valid WHOAMI values but invalid i2c read
static w_status_t i2c_read_reg_custom_fake2(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    if (device_addr == LIS3MDL_ADDR) {
        *data = 0x3D;
    } else if (device_addr == LPS22DF_ADDR) {
        *data = 0xB4;
    } else if (device_addr == LSM6DSO_ADDR) {
        *data = 0x6C;
    }
    return W_FAILURE;
};

// Invalid WHOAMI values but valid i2c read
static w_status_t i2c_read_reg_custom_fake3(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    *data = 0x0;
    return W_SUCCESS;
};

// Return fake acceleration data
static w_status_t i2c_read_reg_custom_fake4(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    // 1.1g in X, 2.2g in Y, -3.3g in Z
    data[0] = 0xCD; // X-axis low byte
    data[1] = 0x08; // X-axis high byte
    data[2] = 0x99; // Y-axis low byte
    data[3] = 0x11; // Y-axis high byte
    data[4] = 0x9A; // Z-axis low byte
    data[5] = 0xE5; // Z-axis high byte
    return W_SUCCESS;
}

// Return fake gyro data
static w_status_t i2c_read_reg_custom_fake5(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    // 550 dps in X, 1050 dps in Y, and -550 dps in Z
    data[0] = 0x33; // X-axis low byte
    data[1] = 0x23; // X-axis high byte
    data[2] = 0x33; // Y-axis low byte
    data[3] = 0x43; // Y-axis high byte
    data[4] = 0xCD; // Z-axis low byte
    data[5] = 0xDC; // Z-axis high byte
    return W_SUCCESS;
}

// Return fake magnetometer data
static w_status_t i2c_read_reg_custom_fake6(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    // 2 Gauss in X, 4 Gauss in Y, -6 Gauss in Z
    data[0] = 0x00; // X-axis low byte
    data[1] = 0x10; // X-axis high byte
    data[2] = 0x00; // Y-axis low byte
    data[3] = 0x20; // Y-axis high byte
    data[4] = 0x00; // Z-axis low byte
    data[5] = 0xD0; // Z-axis high byte (negative value)
    return W_SUCCESS;
}
static w_status_t i2c_read_reg_custom_fake7(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    // Pressure: 420 Pa
    // Datasheet: 1 LSB = 1/4096 hPa = 0.0244140625 Pa
    // Raw value = 420 / 0.0244140625 ≈ 17184 counts
    int32_t pres_raw = 17184;
    data[0] = (uint8_t)(pres_raw & 0xFF); // XL
    data[1] = (uint8_t)((pres_raw >> 8) & 0xFF); // L
    data[2] = (uint8_t)((pres_raw >> 16) & 0xFF); // H

    // Temperature: 69 C
    // Datasheet: 1 LSB = 1/100 C
    int16_t temp_raw = (int16_t)(69 * 100);
    data[3] = (uint8_t)(temp_raw & 0xFF); // L
    data[4] = (uint8_t)((temp_raw >> 8) & 0xFF); // H

    return W_SUCCESS;
}

static w_status_t i2c_read_reg_custom_fake8(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    // Pressure: 100000 Pa
    // Datasheet: 1 LSB = 1/4096 hPa = 0.0244140625 Pa
    // Raw value = 100000 / 0.0244140625 ≈ 4096000 counts
    int32_t pres_raw = 4096000;

    data[0] = (uint8_t)(pres_raw & 0xFF); // XL
    data[1] = (uint8_t)((pres_raw >> 8) & 0xFF); // L
    data[2] = (uint8_t)((pres_raw >> 16) & 0xFF); // H

    // Temperature: 25 C
    // Datasheet: 1 LSB = 1/100 C
    int16_t temp_raw = (int16_t)(25 * 100);

    data[3] = (uint8_t)(temp_raw & 0xFF); // L
    data[4] = (uint8_t)((temp_raw >> 8) & 0xFF); // H

    return W_SUCCESS;
}
}

class AltimuTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(i2c_write_reg);
        RESET_FAKE(i2c_read_reg);
        RESET_FAKE(gpio_write);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Kind of a proxy for testing write_1_byte
TEST_F(AltimuTest, InitCallsI2CWriteNTimes) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = altimu_init();
    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // There are currently 12 config registers we care about writing to when we initialize
    // Checking that we do 12 writes is probably enough to detect if smth changes, I'm not going to
    // copy the current config into a test
    EXPECT_EQ(i2c_write_reg_fake.call_count, 12);
    EXPECT_EQ(i2c_write_reg_fake.arg0_val, I2C_BUS_4);
    EXPECT_EQ(i2c_write_reg_fake.arg4_val, 1); // write length
    EXPECT_EQ(gpio_write_fake.call_count, 1);
    EXPECT_EQ(gpio_write_fake.arg0_val, GPIO_PIN_ALTIMU_SA0);
    EXPECT_EQ(gpio_write_fake.arg1_val, GPIO_LEVEL_HIGH); // we want to use the high addr sel always
}

// Init tests
TEST_F(AltimuTest, InitFailsIfI2CWriteFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = altimu_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// gpio init fail
TEST_F(AltimuTest, InitFailsIfGpioWriteFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = altimu_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AltimuTest, SanityCheckPasses) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake1;

    // Act
    w_status_t status = altimu_check_sanity();

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(AltimuTest, SanityCheckFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake2;

    // Act
    w_status_t status = altimu_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AltimuTest, SanityCheckFailsIfWHOAMIInvalid) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake3;

    // Act
    w_status_t status = altimu_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// Data read failure

TEST_F(AltimuTest, GetAccDataFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    vector3d_t data;
    altimu_raw_imu_data_t raw_data;
    w_status_t status = altimu_get_acc_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AltimuTest, GetGyroDataFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    vector3d_t data;
    altimu_raw_imu_data_t raw_data;
    w_status_t status = altimu_get_gyro_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(AltimuTest, GetMagDataFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    vector3d_t data;
    altimu_raw_imu_data_t raw_data;
    w_status_t status = altimu_get_mag_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_IO_ERROR);
}

TEST_F(AltimuTest, GetBaroDataFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    altimu_barometer_data_t data;
    altimu_raw_baro_data_t raw_data;
    w_status_t status = altimu_get_baro_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_IO_ERROR);
}

// Data read conversion

TEST_F(AltimuTest, GetAccDataConversion) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake4;

    // Act
    vector3d_t data;
    altimu_raw_imu_data_t raw_data;
    w_status_t status = altimu_get_acc_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    // Needs to be within 16 bit int rounding tolerance -> FS/2^15 ~> 0.000488296152 plus float
    // tolerance plus me rounding when I write these test cases
    EXPECT_NEAR(data.x, 1.1f, 0.000488296152);
    EXPECT_NEAR(data.y, 2.2f, 0.000488296152);
    EXPECT_NEAR(data.z, -3.3f, 0.000488296152);

    // Check raw data
    EXPECT_EQ(raw_data.x, 0x08CD); // X-axis raw value
    EXPECT_EQ(raw_data.y, 0x1199); // Y-axis raw value
    EXPECT_EQ(raw_data.z, 0xE59A); // Z-axis raw value
}

TEST_F(AltimuTest, GetGyroDataConversion) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake5;

    // Act
    vector3d_t data;
    altimu_raw_imu_data_t raw_data;
    w_status_t status = altimu_get_gyro_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    // tolerance -> FS/2^15 ~> 0.06103701895f
    EXPECT_NEAR(data.x, 550.0f, 0.06103701895);
    EXPECT_NEAR(data.y, 1050.0f, 0.06103701895);
    EXPECT_NEAR(data.z, -550.0f, 0.06103701895);

    // Check raw data
    EXPECT_EQ(raw_data.x, 0x2333); // X-axis raw value
    EXPECT_EQ(raw_data.y, 0x4333); // Y-axis raw value
    EXPECT_EQ(raw_data.z, 0xDCCD); // Z-axis raw value
}

TEST_F(AltimuTest, GetMagDataConversion) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake6;

    // Act
    vector3d_t data;
    altimu_raw_imu_data_t raw_data;
    w_status_t status = altimu_get_mag_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    //  tolerance -> FS/2^15 ~> 0.000488296152
    EXPECT_NEAR(data.x, 2.0f, 0.000488296152);
    EXPECT_NEAR(data.y, 4.0f, 0.000488296152);
    EXPECT_NEAR(data.z, -6.0f, 0.000488296152);

    // Check raw data
    EXPECT_EQ(raw_data.x, 0x1000); // X-axis raw value
    EXPECT_EQ(raw_data.y, 0x2000); // Y-axis raw value
    EXPECT_EQ(raw_data.z, 0xD000); // Z-axis raw value
}

TEST_F(AltimuTest, GetBaroDataConversionInRange) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake8;

    // Act
    altimu_barometer_data_t data;
    altimu_raw_baro_data_t raw_data;
    w_status_t status = altimu_get_baro_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_NEAR(data.pressure, 100000, 0.0244140625);
    // tolerance -> 0.01
    EXPECT_NEAR(data.temperature, 25, 0.01);

    // Check raw data
    EXPECT_EQ(raw_data.pressure, 4096000); // Pressure raw value
    EXPECT_EQ(raw_data.temperature, (int16_t)(25 * 100)); // Temperature raw value
}

TEST_F(AltimuTest, GetBaroDataConversionSaturateLow) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake7;

    // Act
    altimu_barometer_data_t data;
    altimu_raw_baro_data_t raw_data;
    w_status_t status = altimu_get_baro_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    // expect saturate to mininum 6000 Pa
    EXPECT_NEAR(data.pressure, 6000, 0.0244140625);
    // tolerance -> 0.01
    EXPECT_NEAR(data.temperature, 69.0f, 0.01);

    // Check raw data
    EXPECT_EQ(raw_data.pressure, 0x004320); // Pressure raw value
    EXPECT_EQ(raw_data.temperature, 0x1AF4); // Temperature raw value
}
