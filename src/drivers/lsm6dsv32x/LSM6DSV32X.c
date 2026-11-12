#include "FreeRTOS.h"
#include "i2c.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/i2c/i2c.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X.h"
#include "drivers/lsm6dsv32x/LSM6DSV32X_regmap.h"
#include "drivers/timer/timer.h"

typedef enum {
  LSM6DSV32X_READ_BUFFER = 0,
  LSM6DSV32X_WRITE_BUFFER = 1
} lsm6dsv32x_buffer_t;

typedef enum {
  LSM6DSV32X_DATA_STALE = 1,
  LSM6DSV32X_DATA_READY = 0
} lsm6dsv32x_data_state_t;

typedef enum {
  LSM6DSV32X_BUS_BUSY = 0,
  LSM6DSV32X_BUS_FREE = 1
} lsm6dsv32x_bus_state;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t dev_addr;
  uint8_t dual_buffer[2][LSM6DSV32X_FIFO_THRESHOLD][7];
  uint32_t timestamp_ms[2];
  volatile lsm6dsv32x_bus_state bus_status;
  bool switched_callback;
  w_status_t latest_status;

} lsm6dsv32x_ctx_t;

/**
 * health check information
 */
typedef struct {
  bool is_init;
  uint32_t init_failed_write;
  uint32_t init_failed_register_callback;
  uint32_t is_insane;
  uint32_t post_init_sanity_checks;
  uint32_t dma_handle_wrong_i2c;
  uint32_t dma_data_transfer_unswitched_callback;
  uint32_t dma_data_transfer_failed_timer;
  uint32_t dma_data_transfer_failed_mem_read;
  uint32_t get_gyro_acc_data_unswitched_callback;
  uint32_t get_gyro_acc_data_lastest_status_failure;
  uint32_t wrong_fifo_tag;
  uint32_t same_sensor_samples;
} lsm6dsv32x_health_t;

// device addresses and configuration
static const uint16_t LSM6DSV32X_ADDR = 0x6a; // addr sel pin LOW

// external definition of h2ic for the i2c bus 1;
extern I2C_HandleTypeDef hi2c1;

// sensor ranges. these must be selected using the i2c init regs
static const float64_t ACC_RANGE = 32.0;    // g
static const float64_t GYRO_RANGE = 4000.0; // dps

// LSM6DSV32X conversion factors based on datasheet
static const float64_t ACC_FS = 0.000976; // g / LSB
static const float64_t GYRO_FS = 0.140;   // dps / LSB

static const uint8_t GYRO_TAG = 0x01;
static const uint8_t ACC_TAG = 0x02;

static const size_t LSM6DSV32X_EXPECTED_WHO_AM_I = 0x70;

static lsm6dsv32x_ctx_t lsm6dsv32x_ctx = {.switched_callback = false};

static const uint8_t CTX_BUFFER_SIZE =
    sizeof(lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER]);

static lsm6dsv32x_health_t lsm6dsv32x_health = {0};

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
  return i2c_write_reg(I2C_BUS_1, addr, reg, &data, 1);
}

/**
 * @brief Sanity check for the IMU
 * @note just checks the who am i bit
 * @return Status of the operation
 */
static w_status_t lsm6dsv32x_check_sanity() {
  if (lsm6dsv32x_ctx.switched_callback) {
    log_text(1, LOG_LVL_WARN, "LSM6DSV32X",
             "Attempting to reinitialize after switching callback.");
    lsm6dsv32x_health.post_init_sanity_checks++;
    return W_FAILURE;
  }

  w_status_t i2c_status = W_SUCCESS;
  w_status_t device_status = W_SUCCESS;

  uint8_t who_am_i = 0;

  i2c_status |= i2c_read_reg(I2C_BUS_1, LSM6DSV32X_ADDR, LSM6DSV32X_WHO_AM_I,
                             &who_am_i, 1);
  if (LSM6DSV32X_EXPECTED_WHO_AM_I != who_am_i) {
    device_status |= W_FAILURE;
  }

  if ((W_SUCCESS == device_status) && (W_SUCCESS == i2c_status)) {
    return W_SUCCESS;
  } else {
    lsm6dsv32x_health.is_insane++;
    return W_FAILURE;
  }
}

// not static to allow to be called in unit test
/**
 * @brief handler for after the DMA is completed
 */
void lsm6dsv32x_dma_complete_handle(I2C_HandleTypeDef *hi2c) {
  if (hi2c != lsm6dsv32x_ctx.hi2c) {
    lsm6dsv32x_health.dma_handle_wrong_i2c++;
    return;
  }

  lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_FREE;

  memcpy(lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER],
         lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_WRITE_BUFFER], CTX_BUFFER_SIZE);
  lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_READ_BUFFER] =
      lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_WRITE_BUFFER];
}

/**
 * @brief i2c dma error handler
 */
static void lsm6dsv32x_dma_error_handle(I2C_HandleTypeDef *hi2c) {
  if (hi2c != lsm6dsv32x_ctx.hi2c) {
    return;
  }

  lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_FREE;
  lsm6dsv32x_ctx.latest_status = W_IO_ERROR;
}

/**
 * @brief Initializes the bit registers for
 * @note Must be called after bit registers are configured, called before
 * flight!!!
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_init() {
  if (lsm6dsv32x_ctx.switched_callback) {
    log_text(1, LOG_LVL_FATAL, "LSM6DSV32X",
             "Attempting to reinitialize after switching callback.");
    return W_FAILURE;
  }
  lsm6dsv32x_ctx.hi2c = &hi2c1;

  w_status_t status = W_SUCCESS;

  // LSM6DSV32X: https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf

  // Accel ODR: 7.68 KHz
  // Accel mode: high performance
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL1_XL, 0x0C);

  // Gyro ODR: 7.68 KHz
  // Gyro mode: high performance
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL2_G, 0x0C);

  // set FIFO to FIFO mode (fill until LSM6DSV32X_FIFO_THRESHOLD)
  status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL1, 0x01);
  status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL2, 0x00);
  // set accel and gyro ODR to 7.6Khz
  status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL3, 0xCC);
  status |= write_1_byte(LSM6DSV32X_ADDR, FIFO_CTRL4, 0x01);

  // set gyro range to +-4000dps
  // set LPF bandwith to 241 (ODR/4)
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL6_G, 0x0C);

  // enable gryro LPF
  // dont touch input impedance
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL7_G, 0x01);

  // Disable 2 channel mode
  // Accel full scale range +-32g
  // Accel low pass cutoff frequency ODR/4 (240hz)
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL8_XL, 0x07);

  // accel slope filter: low pass
  // accel user offset bit weight: 2^-10 g/LSB
  // disable offset, until imu calibration
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL9_XL, 0x29);

  // pulse drdy, so we can get rising edge
  status |= write_1_byte(LSM6DSV32X_ADDR, CTRL4, 0x02);

  status |= lsm6dsv32x_check_sanity();

  // Trigger INT1 when the FIFO is filled to LSM6DSV32X_FIFO_THRESHOLD
  status |= write_1_byte(LSM6DSV32X_ADDR, INT1_CTRL, 0x20);

  // load all settings and end use of all i2c driver before switch to dma
  vTaskDelay(1);

  lsm6dsv32x_ctx.switched_callback = true;

  HAL_StatusTypeDef i2c_register_status = HAL_OK;
  // register the I2C callback for the end of the DMA read to call the dma
  // complete handler function
  i2c_register_status |= HAL_I2C_RegisterCallback(
      lsm6dsv32x_ctx.hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID,
      lsm6dsv32x_dma_complete_handle);
  i2c_register_status |= HAL_I2C_RegisterCallback(
      lsm6dsv32x_ctx.hi2c, HAL_I2C_ERROR_CB_ID, lsm6dsv32x_dma_error_handle);

  // AFTER THIS POINT NEVER USE OLD I2C OR VERY BAD ERRORS

  if (HAL_OK != i2c_register_status) {
    lsm6dsv32x_health.init_failed_register_callback++;
    log_text(1, LOG_LVL_FATAL, "LSM6DSV32X",
             "Failed to reregister I2C callbacks");
    return W_FAILURE;
  }

  if (W_SUCCESS == status) {
    lsm6dsv32x_health.is_init = 1;
  } else {
    lsm6dsv32x_health.init_failed_write++;
    log_text(1, LOG_LVL_FATAL, "LSM6DSV32X", "initfail");
  }

  // only open the i2c bus once fully ready
  lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_FREE;

  return status;
}

/**
 * @brief ISR for the interrupt pin that begins DMA data transfer
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_int1_isr_handler() {
  if (!lsm6dsv32x_health.is_init) {
    return W_SUCCESS;
  }

  if (!lsm6dsv32x_ctx.switched_callback) {
    lsm6dsv32x_health.dma_data_transfer_unswitched_callback++;
    return W_FAILURE;
  }

  // store the timestamp when the data is received
  if (timer_get_ms(&lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_WRITE_BUFFER]) !=
      W_SUCCESS) {
    lsm6dsv32x_health.dma_data_transfer_failed_timer++;
    return W_FAILURE;
  }

  lsm6dsv32x_ctx.latest_status = W_SUCCESS;

  // set the bus to occupied meanining that data is
  // actively being streamed via DMA into the Buffer from the IMU
  lsm6dsv32x_ctx.bus_status = LSM6DSV32X_BUS_BUSY;

  const uint8_t shifted_address = (LSM6DSV32X_ADDR << 1);

  HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_DMA(
      lsm6dsv32x_ctx.hi2c, shifted_address, FIFO_DATA_OUT_TAG,
      I2C_MEMADD_SIZE_8BIT,
      lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_WRITE_BUFFER][0], CTX_BUFFER_SIZE);

  if (hal_status != HAL_OK) {
    lsm6dsv32x_ctx.bus_status =
        LSM6DSV32X_BUS_FREE; // so that we can attempt send again
    lsm6dsv32x_ctx.latest_status = W_IO_ERROR;
    lsm6dsv32x_health.dma_data_transfer_failed_mem_read++;
  }
  return lsm6dsv32x_ctx.latest_status;
}

/**
 * @brief Retrieves the latest 12 bytes of imu data
 * @param[out] acc_data    Processed accelerometer data (gravities)
 * @param[out] gyro_data   Processed gyroscope data (deg/s)
 * @param[out] raw_acc     Raw accelerometer data
 * @param[out] raw_gyro    Raw gyroscope data
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_get_gyro_acc_data(vector3d_t *acc_data,
                                        vector3d_t *gyro_data,
                                        lsm6dsv32x_raw_imu_data_t *raw_acc,
                                        lsm6dsv32x_raw_imu_data_t *raw_gyro) {
  w_status_t status = W_SUCCESS;
  uint8_t
      raw_bytes[LSM6DSV32X_FIFO_THRESHOLD]
               [7]; // copy the bytes so they are safe while doing calculations

  if (lsm6dsv32x_ctx.switched_callback) 
  {
    taskENTER_CRITICAL();

    // enter a critical section while copying the data
    memcpy(raw_bytes, lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER],
           CTX_BUFFER_SIZE);
    raw_acc->timestamp_ms = lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_READ_BUFFER];
    // get status
    status = lsm6dsv32x_ctx.latest_status;

    taskEXIT_CRITICAL();

    // find the latest samples
    uint8_t *latest_gyro_bytes = NULL;
    uint8_t *latest_acc_bytes = NULL;
    size_t i = LSM6DSV32X_FIFO_THRESHOLD - 1;
    do 
    {
      uint8_t tag = raw_bytes[i][0] >> 3;
      if ((GYRO_TAG == tag) && (NULL == latest_gyro_bytes)) 
      {
        latest_gyro_bytes = &raw_bytes[i][1];
      } 
      else if ((ACC_TAG == tag) && (NULL == latest_acc_bytes)) 
      {
        latest_acc_bytes = &raw_bytes[i][1];
      }
      i--;
    } 
    while ((0 < i) && ((NULL == latest_gyro_bytes) || (NULL == latest_acc_bytes)));

    if ((0 == i) && ((NULL == latest_gyro_bytes) || (NULL == latest_acc_bytes))) 
    {
      // can't read, we got only samples of one type
      lsm6dsv32x_health.same_sensor_samples++;
      lsm6dsv32x_health.get_gyro_acc_data_lastest_status_failure++;
      return W_FAILURE;
    }

    // Parse gyroscope raw data (first 6 bytes)
    raw_gyro->x = (uint16_t)(((uint16_t)latest_gyro_bytes[1] << 8) |
                             latest_gyro_bytes[0]);
    raw_gyro->y = (uint16_t)(((uint16_t)latest_gyro_bytes[3] << 8) |
                             latest_gyro_bytes[2]);
    raw_gyro->z = (uint16_t)(((uint16_t)latest_gyro_bytes[5] << 8) |
                             latest_gyro_bytes[4]);

    raw_gyro->timestamp_ms =
        raw_acc->timestamp_ms; // grab time from accel since it's the smae
                               // timestamp

    // Parse accelerometer raw data (next 6 bytes)
    raw_acc->x =
        (uint16_t)(((uint16_t)latest_acc_bytes[1] << 8) | latest_acc_bytes[0]);
    raw_acc->y =
        (uint16_t)(((uint16_t)latest_acc_bytes[3] << 8) | latest_acc_bytes[2]);
    raw_acc->z =
        (uint16_t)(((uint16_t)latest_acc_bytes[5] << 8) | latest_acc_bytes[4]);

    // Convert to physical units
    gyro_data->x = ((float64_t)((int16_t)raw_gyro->x)) * GYRO_FS;
    gyro_data->y = ((float64_t)((int16_t)raw_gyro->y)) * GYRO_FS;
    gyro_data->z = ((float64_t)((int16_t)raw_gyro->z)) * GYRO_FS;

    acc_data->x = ((float64_t)((int16_t)raw_acc->x)) * ACC_FS;
    acc_data->y = ((float64_t)((int16_t)raw_acc->y)) * ACC_FS;
    acc_data->z = ((float64_t)((int16_t)raw_acc->z)) * ACC_FS;

    if (W_FAILURE == status) 
    {
      lsm6dsv32x_health.get_gyro_acc_data_lastest_status_failure++;
    }
  }
  else 
  {
    status = W_FAILURE;
    lsm6dsv32x_health.get_gyro_acc_data_unswitched_callback++;
  }

  return status;
}

/**
 * @brief Retrieves all samples of gyro and acc data stored in the FIFO
 * @param[out] acc_data          Processed accelerometer data (gravities)
 * @param[out] gyro_data         Processed gyroscope data (deg/s)
 * @param[out] raw_acc           Raw accelerometer data
 * @param[out] raw_gyro          Raw gyroscope data
 * @param[out] gyro_sample_count The number of gyro samples in the FIFO
 * @param[out] acc_sample_count  The number of acc samples in the FIFO
 * @return Status of the operation
 */
w_status_t lsm6dsv32x_get_full_gyro_acc_data(
    vector3d_t acc_data[LSM6DSV32X_FIFO_THRESHOLD],
    vector3d_t gyro_data[LSM6DSV32X_FIFO_THRESHOLD],
    lsm6dsv32x_raw_imu_data_t raw_accs[LSM6DSV32X_FIFO_THRESHOLD],
    lsm6dsv32x_raw_imu_data_t raw_gyros[LSM6DSV32X_FIFO_THRESHOLD],
    size_t *gyro_sample_count, size_t *acc_sample_count) {
  w_status_t status = W_SUCCESS;
  uint8_t raw_bytes[LSM6DSV32X_FIFO_THRESHOLD]
                   [7]; // copy the bytes so they are safe while doing
                        // calculations

  if (lsm6dsv32x_ctx.switched_callback) {
    taskENTER_CRITICAL();

    // enter a critical section while copying the data
    memcpy(raw_bytes, lsm6dsv32x_ctx.dual_buffer[LSM6DSV32X_READ_BUFFER],
           CTX_BUFFER_SIZE);
    uint32_t timestamp_ms = lsm6dsv32x_ctx.timestamp_ms[LSM6DSV32X_READ_BUFFER];
    // get status
    status = lsm6dsv32x_ctx.latest_status;

    taskEXIT_CRITICAL();

    size_t accel_index = 0, gyro_index = 0;
    for (size_t i = 0; i < LSM6DSV32X_FIFO_THRESHOLD; i++) {
      uint8_t tag = raw_bytes[i][0] >> 3;
      uint16_t x = (uint16_t)(raw_bytes[i][2] << 8) | raw_bytes[i][1];
      uint16_t y = (uint16_t)(raw_bytes[i][4] << 8) | raw_bytes[i][3];
      uint16_t z = (uint16_t)(raw_bytes[i][6] << 8) | raw_bytes[i][5];

      if (GYRO_TAG == tag) 
      {
        // gyro sample
        raw_gyros[gyro_index].x = x;
        raw_gyros[gyro_index].y = y;
        raw_gyros[gyro_index].z = z;
        raw_gyros[gyro_index].timestamp_ms = timestamp_ms;

        gyro_data[gyro_index].x = ((int16_t)x) * GYRO_FS;
        gyro_data[gyro_index].y = ((int16_t)y) * GYRO_FS;
        gyro_data[gyro_index].z = ((int16_t)z) * GYRO_FS;

        gyro_index++;
      } 
      else if (ACC_TAG == tag) 
      {
        // accel sample
        raw_accs[accel_index].x = x;
        raw_accs[accel_index].y = y;
        raw_accs[accel_index].z = z;
        raw_accs[accel_index].timestamp_ms = timestamp_ms;

        acc_data[accel_index].x = ((int16_t)x) * ACC_FS;
        acc_data[accel_index].y = ((int16_t)y) * ACC_FS;
        acc_data[accel_index].z = ((int16_t)z) * ACC_FS;

        accel_index++;
      } 
      else 
      {
        status = W_FAILURE;
        lsm6dsv32x_health.wrong_fifo_tag++;
      }
      *gyro_sample_count = gyro_index;
      *acc_sample_count = accel_index;
    }

    if (W_FAILURE == status) 
    {
      lsm6dsv32x_health.get_gyro_acc_data_lastest_status_failure++;
    }
  } 
  else 
  {
    status = W_FAILURE;
    lsm6dsv32x_health.get_gyro_acc_data_unswitched_callback++;
  }

  return status;
}

health_status_t lsm6dsv32x_get_status(void) {
  health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
                            .module_id = CANARDS_MODULE_ID_LSM6DSV32X,
                            .error_bitfield = 0};

  // I2C errors
  if (W_IO_ERROR == lsm6dsv32x_ctx.latest_status) {
    status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
    status.error_bitfield |= 1 << CANARDS_MODULE_E_COMM_FAILURE_OFFSET;
  }

  // Failed initialization on critical component
  if (!lsm6dsv32x_health.is_init) {
    status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
    status.error_bitfield |= 1 << CANARDS_MODULE_E_NOT_INIT_OFFSET;
  }

  log_text(10, LOG_LVL_INFO, "LSM6DSV32X",
           "init=%d, init_failed_write=%d, init_failed_register_cb=%d, "
           "is_insane=%d, sanity_checks=%d",
           lsm6dsv32x_health.is_init, lsm6dsv32x_health.init_failed_write,
           lsm6dsv32x_health.init_failed_register_callback,
           lsm6dsv32x_health.is_insane,
           lsm6dsv32x_health.post_init_sanity_checks);

  // expect the unswitched callback count to be high but most importantly
  // constant. it is called about 10000 times before initialization completes
  log_text(10, LOG_LVL_INFO, "LSM6DSV32X",
           "wrong_i2c=%d, unswitched_cb=%d, failed_timer=%d, fail_mem_read=%d",
           lsm6dsv32x_health.dma_handle_wrong_i2c,
           lsm6dsv32x_health.dma_data_transfer_unswitched_callback,
           lsm6dsv32x_health.dma_data_transfer_failed_timer,
           lsm6dsv32x_health.dma_data_transfer_failed_mem_read);

  log_text(10, LOG_LVL_INFO, "LSM6DSV32X",
           "get_data_unswitched_cb=%d, get_data_sts_fail=%d, latest_status=%d, "
           "bus_sts=%d",
           lsm6dsv32x_health.get_gyro_acc_data_unswitched_callback,
           lsm6dsv32x_health.get_gyro_acc_data_lastest_status_failure,
           lsm6dsv32x_ctx.latest_status, lsm6dsv32x_ctx.bus_status);

  log_text(10, LOG_LVL_INFO, "LSM6DSV32X",
           "wrong_fifo_tag=%d, same_sensor_samples=%d",
           lsm6dsv32x_health.wrong_fifo_tag,
           lsm6dsv32x_health.same_sensor_samples);

  return status;
}
