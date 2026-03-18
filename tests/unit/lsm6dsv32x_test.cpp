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

// w_status_t timer_get_ms(float *ms);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float*);

//void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
FAKE_VALUE_FUNC(void, HAL_GPIO_EXTI_Callback, uint16_t);

//void HAL_I2C_Mem_Read_DMA
FAKE_VALUE_FUNC(void, HAL_I2C_Mem_Read_DMA, I2C_HandleTypeDef, uint16_t, uint16_t, uint8_t, uint16_t);

//void HAL_I2C_MemRxCpltCallback
FAKE_VALUE_FUNC(void, HAL_I2C_MemRxCpltCallback, I2C_HandleTypeDef);



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

static w_status_t timer_get_ms_fake4(float *ms)
{
    *ms = 212;
    return W_SUCCESS;
}

static void HAL_GPIO_EXTI_Callback_fake5(uint16_t gpio_pin){
    if (GPIO_Pin == IMU_INT1_PIN) {
		lsm6dsv32x_int1_isr_handler();
	}

}

static void HAL_I2C_Mem_Read_DMA_fake6(I2C_HandleTypeDef *h2ic,
								   uint8_t device_addr,
								   uint8_t mem_addr,
								   uint8_t mem_size,
								   uint8_t *data,
								   uint8_t write_size){

    data[0] = 0xCD; // X-axis low byte
    data[1] = 0x08; // X-axis high byte
    data[2] = 0x99; // Y-axis low byte

    data[3] = 0x11; // Y-axis high byte
    data[4] = 0x9A; // Z-axis low byte
    data[5] = 0xE5; // Z-axis high byte
                                                                        
    data[6] = 0xCD; // X-axis low byte
    data[7] = 0x08; // X-axis high byte
    data[8] = 0x99; // Y-axis low byte

    data[9] = 0x11; // Y-axis high byte
    data[10] = 0x9A; // Z-axis low byte
    data[11] = 0xE5; // Z-axis high byte

    HAL_I2C_MemRxCpltCallback(h2ic);

    }

static void HAL_I2C_MemRxCpltCallback_fake7(I2C_HandleTypeDef *hi2c) {
	if (lsm6dsv32x_ctx.hi2c == hi2c) {
		lsm6dsv32x_dma_complete_handle();
	}
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

TEST_F(lsm6dsv32xTest, GetAllDataConversion) {

    // Act
    vector3d_t acc_data;
    vector3d_t gyro_data; 
    lsm6dsv32x_raw_imu_data_t raw_acc;
    lsm6dsv32x_raw_imu_data_t raw_gyro;

    HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
    w_status_t status = lsm6dsv32x_get_gyro_acc_data(acc_data,gyro_data,raw_acc,raw_gyro);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    // Needs to be within 16 bit int rounding tolerance -> FS/2^15 ~> 0.000488296152 plus float
    // tolerance plus me rounding when I write these test cases
    EXPECT_NEAR(acc_data.x, 1.1f, 0.000488296152);
    EXPECT_NEAR(acc_data.y, 2.2f, 0.000488296152);
    EXPECT_NEAR(acc_data.z, -3.3f, 0.000488296152);

    // Check raw data
    EXPECT_EQ(raw_acc.x, 0x08CD); // X-axis raw value
    EXPECT_EQ(raw_acc.y, 0x1199); // Y-axis raw value
    EXPECT_EQ(raw_acc.z, 0xE59A); // Z-axis raw value

    // Needs to be within 16 bit int rounding tolerance -> FS/2^15 ~> 0.000488296152 plus float
    // tolerance plus me rounding when I write these test cases
    EXPECT_NEAR(gyro_data.x, 1.1f, 0.000488296152);
    EXPECT_NEAR(gryo_data.y, 2.2f, 0.000488296152);
    EXPECT_NEAR(gyro_data.z, -3.3f, 0.000488296152);

    // Check raw data
    EXPECT_EQ(raw_gyro.x, 0x08CD); // X-axis raw value
    EXPECT_EQ(raw_gyro.y, 0x1199); // Y-axis raw value
    EXPECT_EQ(raw_gyro.z, 0xE59A); // Z-axis raw value
}






