<<<<<<<< HEAD:src/drivers/ak45-10/ak45-10.h
#ifndef AK45_10_H
#define AK45_10_H
========
#ifndef AK45_DRIVER_H
#define AK45_DRIVER_H
>>>>>>>> 964ac1757a2855629744abc9a9b7c84c00a2e59e:src/drivers/ak45_driver/ak45_driver.h

#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

<<<<<<<< HEAD:src/drivers/ak45-10/ak45-10.h
// Motor CAN driver ID configurable on the servo, default 1
#define AK45_10_ID 1

// Servo fault codes
typedef enum {
	AK45_FAULT_NONE = 0,
	AK45_FAULT_OVERTEMP = 1,
	AK45_FAULT_OVERCURRENT = 2,
	AK45_FAULT_OVERVOLTAGE = 3,
	AK45_FAULT_UNDERVOLTAGE = 4,
	AK45_FAULT_ENCODER = 5,
	AK45_FAULT_OVERTEMP_MOSFET = 6,
	AK45_FAULT_MOTOR_LOCKUP = 7
========
// Servo fault codes
typedef enum {
	AK45_FAULT_NONE = 0,
	AK45_FAULT_OVERVOLTAGE = 1,
	AK45_FAULT_UNDERVOLTAGE = 2,
	AK45_FAULT_DRV_FAULT = 3,
	AK45_FAULT_ABS_OVERCURRENT = 4,
	AK45_FAULT_OVERTEMP_FET = 5,
	AK45_FAULT_OVERTEMP_MOTOR = 6
>>>>>>>> 964ac1757a2855629744abc9a9b7c84c00a2e59e:src/drivers/ak45_driver/ak45_driver.h
} ak45_fault_code_t;

/**
 * @brief Decoded feedback from the servo
 */
typedef struct {
	float32_t position_deg;
	float32_t speed_erpm; // RPM counted electrically
	float32_t current_a;
	int8_t temperature_c;
	ak45_fault_code_t fault_code;
	uint32_t timestamp_ms; // Time of last received feedback
} ak45_feedback_t;

/**
 * @brief Initialize the servo driver
 *
 * Sets up internal queues and configures the FDCAN RX filter for the servo's extended ID.
 *
 * @param[in] hfdcan Pointer to FDCAN handle
 * @param[in] can_init_timeout_ms timeout in ms for waiting for AK45's CAN to initialize
 * @return W_SUCCESS on success, W_FAILURE on error
 */
<<<<<<<< HEAD:src/drivers/ak45-10/ak45-10.h
w_status_t ak45_init(FDCAN_HandleTypeDef *hfdcan);
========
w_status_t ak45_driver_init(FDCAN_HandleTypeDef *hfdcan, const uint32_t can_init_timeout_ms);
>>>>>>>> 964ac1757a2855629744abc9a9b7c84c00a2e59e:src/drivers/ak45_driver/ak45_driver.h

/**
 * @brief Send a position command to the servo
 *
 * @param[in] angle_deg  Target position in degrees
 * @return W_SUCCESS on success, W_FAILURE on error
 */
<<<<<<<< HEAD:src/drivers/ak45-10/ak45-10.h
w_status_t ak45_send_position_cmd(float angle_deg);
========
w_status_t ak45_send_position_cmd(float32_t angle_deg);
>>>>>>>> 964ac1757a2855629744abc9a9b7c84c00a2e59e:src/drivers/ak45_driver/ak45_driver.h

/**
 * @brief Send a disable command to the servo
 *
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t ak45_send_disable_cmd(void);

/**
 * @brief Get the latest motor feedback
 *
 * @param[out] fb Pointer to store data
 * @return W_SUCCESS if feedback is available, W_FAILURE otherwise
 */
w_status_t ak45_get_latest_feedback(ak45_feedback_t *fb);

/**
 * @brief Get FDCAN transmit error count
 *
 * @return Number of transmission failures
 */
uint32_t ak45_get_tx_errors(void);

<<<<<<<< HEAD:src/drivers/ak45-10/ak45-10.h
#endif // AK45_10_H
========
#endif // AK45_DRIVER_H
>>>>>>>> 964ac1757a2855629744abc9a9b7c84c00a2e59e:src/drivers/ak45_driver/ak45_driver.h
