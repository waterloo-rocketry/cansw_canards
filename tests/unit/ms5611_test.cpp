#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
    #include "FreeRTOS.h"
    #include "application/logger/log.h"
    #include "drivers/MS5611/MS5611.h"
    #include "drivers/gpio/gpio.h"
    #include "drivers/i2c/i2c.h"

    
    // i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len);
    FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *, uint8_t)

    // i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len);
    FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)

    // w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);
    FAKE_VALUE_FUNC(w_status_t, gpio_write, gpio_pin_t, gpio_level_t, uint32_t);

    FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...);

    #define MS5611_ADDR 0x77 // default (addr sel pin high) address for MS5611

     // Successful PROM read
    static w_status_t i2c_read_reg_custom_fake1(
        i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
    ) {
        if (device_addr == MS5611_ADDR) {
            // prom coeff - values don't matter, just need to be consistent and pass crc check
            data[1] = 0xFF; // dummy data, not used in sanity check
            data[2] = 0xFF; // dummy data, not used in sanity check
            data[3] = 0xFF; // dummy data, not used in sanity check
            data[4] = 0xFF; // dummy data, not used in sanity
            data[5] = 0xFF; // dummy data, not used in sanity check
            data[6] = 0xFF; // dummy data, not used in sanity
        }

        return W_SUCCESS;
    };

    // Valid PROM read but i2c read fails
    static w_status_t i2c_read_reg_custom_fake2(
        i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
    ) {
        if (device_addr == MS5611_ADDR) {
            *data = 0x00; // dummy data, not used in sanity check
        }

        return W_FAILURE;
    };

    // Invalid PROM values but valid i2c read
    static w_status_t i2c_read_reg_custom_fake3(
        i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
    ) {
        *data = 0x0;
        
        return W_SUCCESS;
    };

    // Return fake pressure and temperature data
    static w_status_t i2c_read_reg_custom_fake4(
        i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
    ) {
        // Pressure: 420 Pa
        // Datasheet: 1 LSB = 1/4096 hPa = 0.0244140625 Pa
        // Raw value = 420 / 0.0244140625 ≈ 17184 counts
        int32_t pres_raw = 17184;
        data[0] = (uint8_t)(pres_raw & 0xFF); // XL
        data[1] = (uint8_t)((pres_raw >> 8) & 0xFF); // L

        return W_SUCCESS;
    }

    static w_status_t i2c_read_reg_custom_fake5(
        i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
    ) {
        

        return W_SUCCESS;
    }
}

class MS5611Test : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(i2c_write_reg);
        RESET_FAKE(i2c_read_reg);
        RESET_FAKE(gpio_write);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// check i2c
TEST_F(MS5611Test, SanityCheckFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake2;

    // Act
    w_status_t status = ms5611_init();
    w_status_t status = ms5611_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// init tests
TEST_F(MS5611Test, InitCallsI2CWriteNTimes) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = ms5611_init();

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(MS5611Test, InitFailsIfI2CWriteFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = ms5611_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// gpio init fail
TEST_F(MS5611Test, InitFailsIfGpioWriteFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = ms5611_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// prom read fail
TEST_F(MS5611Test, InitFailsIfPromReadFails) {
    // Arrange
    i2c_write_reg_fake.return_val = W_SUCCESS;
    gpio_write_fake.return_val = W_SUCCESS;
    i2c_read_reg_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = ms5611_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// sanity check tests
TEST_F(MS5611Test, SanityCheckPasses) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake1;

    // Act
    w_status_t status = ms5611_check_sanity();

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(MS5611Test, SanityCheckFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake2;

    // Act
    w_status_t status = ms5611_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(MS5611Test, SanityCheckFailsIfPromValuesInvalid) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake3;

    // Act
    w_status_t status = ms5611_check_sanity();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(MS5611Test, GetRawPressureFailsIfI2CFails) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake4;

    // Act
    ms5611_raw_result_t result;
    w_status_t status = ms5611_get_raw_pressure(&result);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// data conversion fails
TEST_F(MS5611Test, ReportFailureIfDataConversionFails) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake4;

    // Act
    ms5611_raw_result_t result;
    w_status_t status = ms5611_get_raw_pressure(&result);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

// data conversion success
TEST_F(MS5611Test, GetRawPressureDataConversion) {
    // Arrange
    i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake5;

    // Act
    ms5611_raw_result_t result;
    w_status_t status = ms5611_get_raw_pressure(&result);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(result.temperature_centideg, 2007); // 20.07°C in centidegrees
    EXPECT_EQ(result.pressure_centimbar, 100009); // 1000.09 mbar in centimbar
}
