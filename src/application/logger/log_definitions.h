#ifndef LOG__DEFINITIONS_H
#define LOG__DEFINITIONS_H

/* BEGIN GENERATED ERROR ENUM
    A standard error value is split in two parts:
    First two digits represent the file name
    Next two digits count up from 0x00
*/
typedef enum __attribute__((packed)) {
	LOG_ERROR_ERROR = 0x00,
	LOG_ERROR_MAX_SIZE = 0xffff
} log_error_id_t;

STATIC_ASSERT(sizeof(log_error_id_t) == 2, "log_file_error_t must be 2 bytes");
/* END GENERATED ERROR ENUM */

/* BEGIN GENERATED FILE ENUM */
typedef enum __attribute__((packed)) {
	LOG_FILES_ERROR = 0x00,
	LOG_FILES_CAN_HANDLER_C = 0x01,
	LOG_FILES_CONTROLLER_C = 0x02,
	LOG_FILES_FLIGHT_PHASE_C = 0x03,
	LOG_FILES_FSM_C = 0x04,
	LOG_FILES_HEALTH_CHECKS_C = 0x05,
	LOG_FILES_HIL_C = 0x06,
	LOG_FILES_INIT_C = 0x07,
	LOG_FILES_LOG_C = 0x08,
	LOG_FILES_NAVIGATOR_C = 0x09,
	LOG_FILES_POWER_HANDLER_C = 0x0a,
	LOG_FILES_SENSOR_HANDLER_C = 0x0b,
	LOG_FILES_TELEMETRY_C = 0x0c,
	LOG_FILES_ADXL380_C = 0x0d,
	LOG_FILES_ADXRS649_C = 0x0e,
	LOG_FILES_ADC_C = 0x0f,
	LOG_FILES_AK45_DRIVER_C = 0x10,
	LOG_FILES_ALTIMU_10_C = 0x11,
	LOG_FILES_GPIO_C = 0x12,
	LOG_FILES_I2C_C = 0x13,
	LOG_FILES_IIS2MDC_C = 0x14,
	LOG_FILES_LSM6DSV32X_C = 0x15,
	LOG_FILES_MOVELLA_C = 0x16,
	LOG_FILES_MS5611_C = 0x17,
	LOG_FILES_RAW_LOGGER_C = 0x18,
	LOG_FILES_SD_CARD_C = 0x19,
	LOG_FILES_TIMER_C = 0x1a,
	LOG_FILES_UART_C = 0x1b,
	LOG_FILES_SIZE_PIN = 0xff
} log_file_id_t;

STATIC_ASSERT(sizeof(log_file_id_t) == 1, "log_file_id_t must be 1 byte");
/* END GENERATED FILE ENUM */

/* BEGIN GENERATED TASK ENUM
    Tasks increment by one starting from 0
*/
typedef enum __attribute__((packed)) {
	LOG_TASK_ERROR = 0x00,
	LOG_TASK_INIT = 0x01,
	LOG_TASK_FSM = 0x02,
	LOG_TASK_HEALTH_CHECKS = 0x03,
	LOG_TASK_CAN_HANDLER_RX = 0x04,
	LOG_TASK_CAN_HANDLER_TX = 0x05,
	LOG_TASK_MOVELLA = 0x06,
	LOG_TASK_MS5611 = 0x07,
	LOG_TASK_LOGGER = 0x08,
	LOG_TASK_AD_BREAKOUT = 0x09,
	LOG_TASK_TELEMETRY = 0x0a,
	LOG_TASK_MAX_SIZE = 0xff
} log_task_id_t;

STATIC_ASSERT(sizeof(log_task_id_t) <= 1, "log_task_id_t must be at 1 byte");
/* END GENERATED TASK ENUM */

#endif
