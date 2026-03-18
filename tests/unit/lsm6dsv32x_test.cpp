#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/imus/LSM6DSV32X.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"

// i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len);
FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *, uint8_t)

// i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len);
FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)

// w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t);

FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);

}

#define LSM6DSV32X_ADDR 0x6B// addr sel pin HIGH IMU

// Successful WHOMAI read
static w_status_t i2c_read_reg_custom_fake1(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len) {
    if (device_addr == LSM6DSV32X_ADDR) {
        *data = 0x70;
    }
    return W_SUCCESS;
    
};

// Valid WHOAMI values but invalid i2c read
static w_status_t i2c_read_reg_custom_fake2(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    if (device_addr == LSM6DSV32X_ADDR) {
        *data = 0x70;
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

class lsm6dsv32xTest : public ::testing::Test {
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
TEST_F(lsm6dsv32xTest, InitCallsI2CWriteNTimes) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = lsm6dsv32x_init();
    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // There are currently 10 config registers we care about writing to when we initialize
    // Checking that we do 10 writes is probably enough to detect if smth changes, I'm not going to
    // copy the current config into a test
    EXPECT_EQ(i2c_write_reg_fake.call_count, 10);
    EXPECT_EQ(i2c_write_reg_fake.arg0_val, I2C_BUS_4);
    EXPECT_EQ(i2c_write_reg_fake.arg4_val, 1); // write length
    EXPECT_EQ(gpio_write_fake.call_count, 1);
    EXPECT_EQ(gpio_write_fake.arg0_val, GPIO_PIN_7);
    EXPECT_EQ(gpio_write_fake.arg1_val, GPIO_LEVEL_HIGH); // we want to use the high addr sel always
}

// Init tests
TEST_F(lsm6dsv32xTest, InitFailsIfI2CWriteFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = lsm6dsv32x_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// gpio init fail
TEST_F(lsm6dsv32xTest, InitFailsIfGpioWriteFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = lsm6dsv32x_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(lsm6dsv32xTest, SanityCheckPasses) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake1;

    // Act
    w_status_t status = lsm6dsv32x_check_sanity();

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(lsm6dsv32xTest, SanityCheckFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake2;

    // Act
    w_status_t status = lsm6dsv32x_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(lsm6dsv32xTest, SanityCheckFailsIfWHOAMIInvalid) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake3;

    // Act
    w_status_t status = lsm6dsv32x_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// Data read failure

TEST_F(lsm6dsv32xTest, GetAccDataFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    vector3d_t data;
    lsm6dsv32x_raw_imu_data_t raw_data;
    w_status_t status = lsm6dsv32x_get_acc_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(lsm6dsv32xTest, GetGyroDataFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    vector3d_t data;
    lsm6dsv32x_raw_imu_data_t raw_data;
    w_status_t status = lsm6dsv32x_get_gyro_data(&data, &raw_data);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// Data read conversion

TEST_F(lsm6dsv32xTest, GetAccDataConversion) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake4;

    // Act
    vector3d_t data;
    lsm6dsv32x_raw_imu_data_t raw_data;
    w_status_t status = lsm6dsv32x_get_acc_data(&data, &raw_data);

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

TEST_F(lsm6dsv32xTest, GetGyroDataConversion) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake5;

    // Act
    vector3d_t data;
    lsm6dsv32x_raw_imu_data_t raw_data;
    w_status_t status = lsm6dsv32x_get_gyro_data(&data, &raw_data);

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




