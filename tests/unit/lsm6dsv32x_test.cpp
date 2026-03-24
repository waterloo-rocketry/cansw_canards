#include "fff.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits.h>

extern "C" {
#include "FreeRTOS.h"
#include "drivers/imus/LSM6DSV32X.h"
#include "drivers/imus/LSM6DSV32X_regmap.h"
#include "drivers/i2c/i2c.h"
#include "application/logger/log.h"

// Needed only so lsm6dsv32x_init() can run in test setup
FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *, uint8_t)
FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...)
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *)
FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_DMA,
    I2C_HandleTypeDef*, uint16_t, uint16_t,
    uint16_t, uint8_t*, uint16_t)
}

#define LSM6DSV32X_ADDR 0x6B

static w_status_t timer_get_ms_custom_fake(float *ms) {
    if (ms == NULL) {
        return W_INVALID_PARAM;
    }

    *ms = 1234.5f; // arbitrary deterministic timestamp
    return W_SUCCESS;
}

static w_status_t i2c_read_reg_custom_fake_whoami(
    i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len
) {
    (void)bus;
    (void)device_addr;
    (void)reg;
    (void)len;
    *data = 0x70; // expected WHO_AM_I for LSM6DSV32X
    return W_SUCCESS;
}

class Lsm6dsv32xTest : public ::testing::Test {
protected:
    imu_ctx_t ctx{};

    void SetUp() override {
        RESET_FAKE(i2c_write_reg);
        RESET_FAKE(i2c_read_reg);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(log_text);
        FFF_RESET_HISTORY();

        std::memset(&ctx, 0, sizeof(ctx));

        i2c_write_reg_fake.return_val = W_SUCCESS;
        i2c_read_reg_fake.custom_fake = i2c_read_reg_custom_fake_whoami;
        timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

        ASSERT_EQ(lsm6dsv32x_init(&ctx), W_SUCCESS);
        ctx.stale_data = IMU_DATA_STALE;

    }

    void TearDown() override {}
};

TEST_F(Lsm6dsv32xTest, GetGyroAccDataFailsIfDataIsNotReady) {
    // Arrange
    ctx.stale_data = IMU_DATA_STALE;

    // Act
    vector3d_t acc_data;
    vector3d_t gyro_data;
    lsm6dsv32x_raw_imu_data_t raw_acc;
    lsm6dsv32x_raw_imu_data_t raw_gyro;

    w_status_t status = lsm6dsv32x_get_gyro_acc_data(&acc_data, &gyro_data, &raw_acc, &raw_gyro);

    // Assert
    EXPECT_EQ(status, W_IO_ERROR);
}

TEST_F(Lsm6dsv32xTest, GetGyroAccDataConvertsFakeFIFOData) {
    // Arrange
    // FIFO layout in driver:
    //   bytes 0..5   = gyro X/Y/Z (little-endian)
    //   bytes 6..11  = accel X/Y/Z (little-endian)
    //
    // Gyro raw values:
    //   X = 0x2333  -> 1100.0091555 dps
    //   Y = 0x4333  -> 2100.0396740 dps
    //   Z = 0xDCCD  -> -1100.0091555 dps
    //
    // Acc raw values:
    //   X = 0x08CD  ->  2.200262459 g
    //   Y = 0x1199  ->  4.399548326 g
    //   Z = 0xE59A  -> -6.599810785 g

    uint8_t *buf = ctx.dual_buffer[IMU_READ_BUFFER];
    buf[0]  = 0x33; buf[1]  = 0x23; // gyro X
    buf[2]  = 0x33; buf[3]  = 0x43; // gyro Y
    buf[4]  = 0xCD; buf[5]  = 0xDC; // gyro Z
    buf[6]  = 0xCD; buf[7]  = 0x08; // acc X
    buf[8]  = 0x99; buf[9]  = 0x11; // acc Y
    buf[10] = 0x9A; buf[11] = 0xE5; // acc Z

    ctx.stale_data = IMU_DATA_READY;

    // Act
    vector3d_t acc_data;
    vector3d_t gyro_data;
    lsm6dsv32x_raw_imu_data_t raw_acc;
    lsm6dsv32x_raw_imu_data_t raw_gyro;

    w_status_t status = lsm6dsv32x_get_gyro_acc_data(&acc_data, &gyro_data, &raw_acc, &raw_gyro);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);

    EXPECT_EQ(raw_gyro.x, 0x2333);
    EXPECT_EQ(raw_gyro.y, 0x4333);
    EXPECT_EQ(raw_gyro.z, 0xDCCD);

    EXPECT_EQ(raw_acc.x, 0x08CD);
    EXPECT_EQ(raw_acc.y, 0x1199);
    EXPECT_EQ(raw_acc.z, 0xE59A);

    const float gyro_fs = 4000.0f / INT16_MAX;
    const float acc_fs  = 32.0f / INT16_MAX;

    EXPECT_NEAR(gyro_data.x, (int16_t)0x2333 * gyro_fs, 0.001f);
    EXPECT_NEAR(gyro_data.y, (int16_t)0x4333 * gyro_fs, 0.001f);
    EXPECT_NEAR(gyro_data.z, (int16_t)0xDCCD * gyro_fs, 0.001f);

    EXPECT_NEAR(acc_data.x, (int16_t)0x08CD * acc_fs, 0.0001f);
    EXPECT_NEAR(acc_data.y, (int16_t)0x1199 * acc_fs, 0.0001f);
    EXPECT_NEAR(acc_data.z, (int16_t)0xE59A * acc_fs, 0.0001f);

    EXPECT_EQ(ctx.stale_data, IMU_DATA_STALE);
}