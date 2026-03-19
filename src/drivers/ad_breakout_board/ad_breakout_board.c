#include "rocketlib/include/common.h"
#include "drivers/ad_breakout_board/ad_breakout_board.h"
#include "drivers/ad_breakout_board/ADXRS649.h"
#include "application/flight_phase/flight_phase.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"


typedef enum {
    DATA_READY = 0,
    DATA_STALE,
    NO_DATA
} data_state_t;

// struct to hold task context
typedef struct {
	ad_breakout_board_mesurement_t dual_buffer[2];
	ad_breakout_board_raw_mesurement_t dual_buffer_raw[2];
	data_state_t data_state;

} task_ctx_t;


static const uint8_t AD_BREAKOUT_BOARD_PERIOD_MS = 2;
static const size_t TASK_CTX_SIZE = sizeof(task_ctx_t);

/* ADXRS */
// SD Card Log
static const uint16_t IDLE_RECOVERY_ADXRS_SD_LOG_PERIOD_MS= 1000;
static const uint16_t PAD_ADXRS_SD_LOG_PERIOD_MS = 1000/20;
static const uint16_t FLIGHT_ADXRS_SD_LOG_PERIOD_MS = 1000/200;

static const uint16_t IDLE_RECOVERY_ADXRS_SD_LOG_RATE = IDLE_RECOVERY_ADXRS_SD_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXRS_SD_LOG_RATE = PAD_ADXRS_SD_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXRS_SD_LOG_RATE = FLIGHT_ADXRS_SD_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;

// Flash Log
static const uint16_t IDLE_RECOVERY_ADXRS_FLASH_LOG_PERIOD_MS= 1000;
static const uint16_t PAD_ADXRS_FLASH_LOG_PERIOD_MS = 1000/5;
static const uint16_t FLIGHT_ADXRS_FLASH_LOG_PERIOD_MS = 1000/20;

static const uint16_t IDLE_RECOVERY_ADXRS_FLASH_LOG_RATE = IDLE_RECOVERY_ADXRS_FLASH_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXRS_FLASH_LOG_RATE = PAD_ADXRS_FLASH_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXRS_FLASH_LOG_RATE = FLIGHT_ADXRS_FLASH_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;

// CAN (Telemetry)
static const uint16_t IDLE_ADXRS_CAN_LOG_PERIOD_MS= 1000;
static const uint16_t PAD_ADXRS_CAN_LOG_PERIOD_MS = 1000/10;
static const uint16_t FLIGHT_ADXRS_CAN_LOG_PERIOD_MS = 1000/10;

static const uint16_t IDLE_ADXRS_CAN_LOG_RATE = IDLE_ADXRS_CAN_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXRS_CAN_LOG_RATE = PAD_ADXRS_CAN_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXRS_CAN_LOG_RATE = FLIGHT_ADXRS_CAN_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;

/* ADXL */
// SD Card Log
static const uint16_t IDLE_RECOVERY_ADXL_SD_LOG_PERIOD_MS= 1000;
static const uint16_t PAD_ADXL_SD_LOG_PERIOD_MS = 1000/20;
static const uint16_t FLIGHT_ADXL_SD_LOG_PERIOD_MS = 1000/50;

static const uint16_t IDLE_RECOVERY_ADXL_SD_LOG_RATE = IDLE_RECOVERY_ADXL_SD_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXL_SD_LOG_RATE = PAD_ADXL_SD_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXL_SD_LOG_RATE = FLIGHT_ADXL_SD_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;

// Flash Log
static const uint16_t IDLE_RECOVERY_ADXL_FLASH_LOG_PERIOD_MS= 1000;
static const uint16_t PAD_ADXL_FLASH_LOG_PERIOD_MS = 1000/5;
static const uint16_t FLIGHT_ADXL_FLASH_LOG_PERIOD_MS = 1000/20;

static const uint16_t IDLE_RECOVERY_ADXL_FLASH_LOG_RATE = IDLE_RECOVERY_ADXL_FLASH_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXL_FLASH_LOG_RATE = PAD_ADXL_FLASH_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXL_FLASH_LOG_RATE = FLIGHT_ADXL_FLASH_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;

// CAN (Telemetry)
static const uint16_t IDLE_ADXL_CAN_LOG_PERIOD_MS= 1000;
static const uint16_t PAD_ADXL_CAN_LOG_PERIOD_MS = 1000/10;
static const uint16_t FLIGHT_ADXL_CAN_LOG_PERIOD_MS = 1000/10;

static const uint16_t IDLE_ADXL_CAN_LOG_RATE = IDLE_ADXL_CAN_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t PAD_ADXL_CAN_LOG_RATE = PAD_ADXL_CAN_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;
static const uint16_t FLIGHT_ADXL_CAN_LOG_RATE = FLIGHT_ADXL_CAN_LOG_PERIOD_MS/AD_BREAKOUT_BOARD_PERIOD_MS;

static task_ctx_t task_ctx = {};



/**
 * @brief initalize both the breakout board sensor drivers
 * @return the status of initalization
 */
w_status_t ad_beakout_board_init() {
    w_status_t status = W_SUCCESS;

    status |= adxrs649_init();
    // TODO: add ADXL init

    task_ctx.data_state = NO_DATA;
}

static w_status_t ad_breakout_board_data_logging(uint32_t loop_count){
    flight_phase_state_t flight_state = flight_phase_get_state();
    
        // TODO: logging function
        switch (flight_state) {
            case STATE_IDLE:
            case STATE_RECOVERY:


                if ((loop_count % IDLE_RECOVERY_ADXRS_SD_LOG_RATE) == 0) {
                    
                }
                if ((loop_count % IDLE_RECOVERY_ADXRS_SD_LOG_RATE) == 0) {
                    
                }
                if ((loop_count % IDLE_RECOVERY_ADXRS_SD_LOG_RATE) == 0) {
                    
                }
                break;
            default:
                break;
        }
}

/**
 * @brief the FreeRTOS Task for getting the sensor data
 */
void ad_breakout_board_task(void *argument) {
    (void)argument;

	// Variables for precise timing control
	TickType_t last_wake_time = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(AD_BREAKOUT_BOARD_PERIOD_MS);
    uint32_t loop_count = 0;

    w_status_t sensor_status = W_SUCCESS;

    while(1) {
        // reset dead state
        task_ctx.dual_buffer[0].is_dead = false;

        sensor_status |= adxrs649_get_gyro_data(&(task_ctx.dual_buffer[0].z_rate), &(task_ctx.dual_buffer_raw[0].z_rate));
        // TODO: add ADXL get acceleration

        if (W_SUCCESS != sensor_status) {
            task_ctx.dual_buffer[0].is_dead = true;
            // TODO: add error logging
        }

        taskENTER_CRITICAL();
        memcpy(&(task_ctx.dual_buffer[1]), &(task_ctx.dual_buffer[0]), TASK_CTX_SIZE);
        taskEXIT_CRITICAL();

        task_ctx.data_state = DATA_READY;


        // LOG/TELEMETRY
        ad_breakout_board_data_logging(loop_count);

        loop_count++;
           
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief to read both the accelerometer and gyro data
 * @param data this is a pointer to converted data
 * @param raw_data pointer to raw data
 * @return the status of getting data
 */
w_status_t ad_breakout_board_get_data(ad_breakout_board_mesurement_t *data, altimu_raw_imu_data_t *raw_data) {
    
}

