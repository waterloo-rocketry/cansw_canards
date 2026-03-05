#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "FreeRTOS.h"
#include "third_party/rocketlib/include/common.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// Motor CAN driver ID configurable on the servo, default 1
#define MOTOR_DRIVER_ID 1

// CAN command IDs
typedef enum {
	CAN_PACKET_SET_DUTY         = 0, // Duty cycle mode
	CAN_PACKET_SET_CURRENT      = 1, // Current loop mode
	CAN_PACKET_SET_CURRENT_BRAKE = 2, // Current brake mode
	CAN_PACKET_SET_RPM          = 3, // Velocity mode
	CAN_PACKET_SET_POS          = 4, // Position mode (likely most optimal)
	CAN_PACKET_SET_ORIGIN_HERE  = 5, // Set origin mode
	CAN_PACKET_SET_POS_SPD      = 6, // Position velocity loop mode
} can_packet_id_t;

// Feedback message mode ID
#define CAN_PACKET_FEEDBACK 0x10

// Scaling factors
#define MOTOR_POS_CMD_SCALE   10000.0f // Position: degrees * 10000
#define MOTOR_POS_FB_SCALE    10.0f    // Feedback position: raw / 10.0 = degrees
#define MOTOR_SPEED_FB_SCALE  10.0f    // speed feedback: raw / 10.0 = ERPM
#define MOTOR_CURRENT_FB_SCALE 100.0f  // current feedback: raw / 100.0 = Amps

// Thresholds
#define MOTOR_OVERVOLTAGE_THRESHOLD  24.0f // maxx voltage
#define MOTOR_OVERCURRENT_THRESHOLD  2.1f // max amps
#define MOTOR_OVERTEMP_THRESHOLD     50    // °C

// Feedback timeout
#define MOTOR_FEEDBACK_TIMEOUT_MS 500

// Servo fault codes
typedef enum {
	MOTOR_FAULT_NONE              = 0,
	MOTOR_FAULT_OVERVOLTAGE       = 1,
	MOTOR_FAULT_UNDERVOLTAGE      = 2,
	MOTOR_FAULT_DRV_FAULT         = 3,
	MOTOR_FAULT_ABS_OVERCURRENT   = 4,
	MOTOR_FAULT_OVERTEMP_FET      = 5,
	MOTOR_FAULT_OVERTEMP_MOTOR    = 6,
} motor_fault_code_t;

/**
 * @brief Decoded feedback from the servo
 */
typedef struct {
	float position_deg;
	float speed_erpm;
	float current_a;
	int8_t temperature_c;
	motor_fault_code_t fault_code;
	uint32_t timestamp_ms; // Time of last received feedback
} motor_feedback_t;

/**
 * @brief Error tracking for motor handler module
 */
typedef struct {
	bool is_init;
	uint32_t tx_errors;            // transmission failures
	uint32_t feedback_timeouts;
	uint32_t fault_count;
	motor_fault_code_t last_fault; // Most recent fault code
} motor_handler_error_data_t;

/**
 * @brief Initialize motor handler module
 *
 * Sets up internal queues and configures the FDCAN RX filter for the servo's extended ID.
 *
 * @param[in] hfdcan Pointer to FDCAN handle
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_handler_init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Motor handler task
 *
 * Checks flight phase (whether actuation is allowed)
 * Gets the commanded angle from the controller module
 * Sends position commands to the servo
 * Reads feedback and checks for fatal faults
 */
void motor_handler_task(void *argument);

/**
 * @brief Get the latest motor feedback
 *
 * @param[out] fb Pointer to store data
 * @return W_SUCCESS if feedback is available, W_FAILURE otherwise
 */
w_status_t motor_handler_get_latest_feedback(motor_feedback_t *fb);

/**
 * @brief Report motor handler health status
 *
 * Logs error statistics and latest feedback data.
 *
 * @return error bitfield
 */
uint32_t motor_handler_get_status(void);

/**
 * @brief Transmit 29-bit ID via FDCAN
 *
 * @param[in] ext_id  29-bit extended CAN ID
 * @param[in] data    Pointer to payload data
 * @param[in] len     Payload length
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_can_transmit_ext(uint32_t ext_id, const uint8_t *data, uint8_t len);

/**
 * @brief Send a position command to the servo
 *
 * @param[in] angle_deg  Target position (degres)
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_send_position_cmd(float angle_deg);

/**
 * @brief Send disable command to the servo
 *
 * @return W_SUCCESS on success, W_FAILURE on error
 */
w_status_t motor_send_disable_cmd(void);

/**
 * @brief Parse raw feedback froom servo
 *
 * @param[in]  data   8-byte feedback data payload
 * @param[out] fb     Decoded feedback struct
 */
void motor_parse_feedback(const uint8_t *data, motor_feedback_t *fb);

/**
 * @brief Check if a fault code is fatal
 *
 * @param[in] fault  Fault code
 * @return true if fatal fault
 */
bool motor_is_fatal_fault(motor_fault_code_t fault);

/**
 * @brief FDCAN RX FIFO1 callback for servo feedback messages
 *
 * Called from ISR context when a message matching the servo feedback filter
 * (extended ID 0x1001) is received.
 */
void motor_fdcan_rx_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

#endif // MOTOR_CONTROLLER_H