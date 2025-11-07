/**
 * @file hil.c
 * @brief Implementation of Hardware-in-the-Loop interface
 */

#include "application/hil/hil.h"
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include <string.h>

// CAN related includes - Ensure base types are included first
#include "canlib/can.h" // Defines can_msg_t
#include "canlib/message/msg_sensor.h" // Defines sensor message builders
#include "canlib/message/msg_state_est.h" // Defines state est message builders
#include "canlib/message_types.h" // Defines can_msg_prio_t, can_imu_id_t, etc.

// External declaration for the function in the FreeRTOS kernel
extern BaseType_t xTaskIncrementTick(void);

// --- HIL UART protocol constants ---

// Buffer for HIL UART data
uint8_t hil_uart_rx_data[HIL_UART_FRAME_SIZE] = {0};
// Count # of received packets so we can skip some (only process every 5th packet)
uint32_t package_counter = 0;
// counter for received packets with wrong header or footer
uint32_t wrong_format_packets = 0;
uint32_t wrong_footer_packets = 0;
volatile _Atomic bool hil_imu_inputs_ready = false;

/**
 * @brief Processes incoming simulator data from HIL UART payload.
 *
 * Parses the data and sends corresponding CAN messages.
 * Marked static as it's only called internally by hil_process_uart_data.
 *
 * @param data Pointer to the raw payload data.
 * @param data_len Length of the payload data (must match HIL_UART_PAYLOAD_SIZE).
 * @return w_status_t W_SUCCESS if processing is successful, error code otherwise.
 */
// static w_status_t simulator_process_data_internal(const uint8_t *data, uint16_t data_len) {
//     if (data == NULL || data_len < sizeof(HilDataPacket)) {
//         return W_FAILURE; // Not enough data for a full packet
//     }

//     const HilDataPacket *packet = (const HilDataPacket *)data;

//     // Use uint16_t for timestamp
//     uint16_t timestamp = 0;
//     can_msg_t can_msg; // Structure to hold the CAN message built by canlib
//     w_status_t status = W_SUCCESS; // Track overall status

//     // --- Convert all incoming float data to uint16_t ---
//     // Encoder
//     uint16_t encoder_angle = (uint16_t)packet->encoder_angle;
//     // Movella
//     uint16_t movella_accel_x = (uint16_t)packet->movella_accel_x;
//     uint16_t movella_accel_y = (uint16_t)packet->movella_accel_y;
//     uint16_t movella_accel_z = (uint16_t)packet->movella_accel_z;
//     uint16_t movella_gyro_x = (uint16_t)packet->movella_gyro_x;
//     uint16_t movella_gyro_y = (uint16_t)packet->movella_gyro_y;
//     uint16_t movella_gyro_z = (uint16_t)packet->movella_gyro_z;
//     uint16_t movella_mag_x = (uint16_t)packet->movella_mag_x;
//     uint16_t movella_mag_y = (uint16_t)packet->movella_mag_y;
//     uint16_t movella_mag_z = (uint16_t)packet->movella_mag_z;
//     // LSM
//     // uint16_t lsm_accel_x = (uint16_t)packet->lsm_accel_x;
//     // uint16_t lsm_accel_y = (uint16_t)packet->lsm_accel_y;
//     // uint16_t lsm_accel_z = (uint16_t)packet->lsm_accel_z;
//     // uint16_t lsm_gyro_x = (uint16_t)packet->lsm_gyro_x;
//     // uint16_t lsm_gyro_y = (uint16_t)packet->lsm_gyro_y;
//     // uint16_t lsm_gyro_z = (uint16_t)packet->lsm_gyro_z;
//     // uint16_t lsm_mag_x = (uint16_t)packet->lsm_mag_x;
//     // uint16_t lsm_mag_y = (uint16_t)packet->lsm_mag_y;
//     // uint16_t lsm_mag_z = (uint16_t)packet->lsm_mag_z;
//     // AltIMU
//     uint16_t altimu_accel_x = (uint16_t)packet->altimu_accel_x;
//     uint16_t altimu_accel_y = (uint16_t)packet->altimu_accel_y;
//     uint16_t altimu_accel_z = (uint16_t)packet->altimu_accel_z;
//     uint16_t altimu_gyro_x = (uint16_t)packet->altimu_gyro_x;
//     uint16_t altimu_gyro_y = (uint16_t)packet->altimu_gyro_y;
//     uint16_t altimu_gyro_z = (uint16_t)packet->altimu_gyro_z;
//     uint16_t altimu_mag_x = (uint16_t)packet->altimu_mag_x;
//     uint16_t altimu_mag_y = (uint16_t)packet->altimu_mag_y;
//     uint16_t altimu_mag_z = (uint16_t)packet->altimu_mag_z;
//     // Pressure
//     uint16_t movella_pressure = (uint16_t)packet->movella_pressure;
//     uint16_t altimu_pressure = (uint16_t)packet->altimu_pressure;
//     // ... (quat conversions needed later)

//     // --- Send Movella IMU Data (Accel/Gyro) ---
//     // X-axis
//     if (build_imu_data_msg(
//             PRIO_HIGH, timestamp, 'x', IMU_PROC_MTI630, movella_accel_x, movella_gyro_x, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Y-axis
//     if (build_imu_data_msg(
//             PRIO_HIGH, timestamp, 'y', IMU_PROC_MTI630, movella_accel_y, movella_gyro_y, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Z-axis
//     if (build_imu_data_msg(
//             PRIO_HIGH, timestamp, 'z', IMU_PROC_MTI630, movella_accel_z, movella_gyro_z, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     // --- Send Movella Magnetometer Data ---
//     // X-axis
//     if (build_mag_data_msg(PRIO_HIGH, timestamp, 'x', IMU_PROC_MTI630, movella_mag_x, &can_msg))
//     {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Y-axis
//     if (build_mag_data_msg(PRIO_HIGH, timestamp, 'y', IMU_PROC_MTI630, movella_mag_y, &can_msg))
//     {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Z-axis
//     if (build_mag_data_msg(PRIO_HIGH, timestamp, 'z', IMU_PROC_MTI630, movella_mag_z, &can_msg))
//     {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     // --- Send AltIMU IMU Data (Accel/Gyro) ---
//     // X-axis
//     if (build_imu_data_msg(
//             PRIO_HIGH, timestamp, 'x', IMU_PROC_ALTIMU10, altimu_accel_x, altimu_gyro_x, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Y-axis
//     if (build_imu_data_msg(
//             PRIO_HIGH, timestamp, 'y', IMU_PROC_ALTIMU10, altimu_accel_y, altimu_gyro_y, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Z-axis
//     if (build_imu_data_msg(
//             PRIO_HIGH, timestamp, 'z', IMU_PROC_ALTIMU10, altimu_accel_z, altimu_gyro_z, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     // --- Send AltIMU Magnetometer Data ---
//     // X-axis
//     if (build_mag_data_msg(PRIO_HIGH, timestamp, 'x', IMU_PROC_ALTIMU10, altimu_mag_x, &can_msg))
//     {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Y-axis
//     if (build_mag_data_msg(PRIO_HIGH, timestamp, 'y', IMU_PROC_ALTIMU10, altimu_mag_y, &can_msg))
//     {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Z-axis
//     if (build_mag_data_msg(PRIO_HIGH, timestamp, 'z', IMU_PROC_ALTIMU10, altimu_mag_z, &can_msg))
//     {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     // --- Send Pressure Data ---
//     // Movella Pressure
//     if (build_analog_data_msg(
//             PRIO_MEDIUM, timestamp, SENSOR_BARO_PRESSURE, movella_pressure, &can_msg
//         )) { // Using MEDIUM priority for pressure
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // AltIMU Pressure
//     if (build_analog_data_msg(
//             PRIO_MEDIUM, timestamp, SENSOR_BARO_PRESSURE, altimu_pressure, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     // --- Send Movella Quaternion Data ---
//     float temp_quat; // Local aligned variable
//     // Q0
//     memcpy(&temp_quat, &packet->movella_quat_0, sizeof(float)); // Copy from packed struct
//     if (build_state_est_data_msg(
//             PRIO_HIGH, timestamp, STATE_ID_ATT_Q0, &temp_quat, &can_msg
//         )) { // Pass address of local var
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Q1
//     memcpy(&temp_quat, &packet->movella_quat_1, sizeof(float));
//     if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q1, &temp_quat, &can_msg)) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Q2
//     memcpy(&temp_quat, &packet->movella_quat_2, sizeof(float));
//     if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q2, &temp_quat, &can_msg)) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }
//     // Q3
//     memcpy(&temp_quat, &packet->movella_quat_3, sizeof(float));
//     if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q3, &temp_quat, &can_msg)) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     // --- Send Encoder Data ---
//     if (build_analog_data_msg(
//             PRIO_HIGH, timestamp, SENSOR_CANARD_ENCODER_1, encoder_angle, &can_msg
//         )) {
//         if (can_handler_transmit(&can_msg) != W_SUCCESS) {
//             status = W_FAILURE;
//         }
//     } else {
//         status = W_FAILURE;
//     }

//     return status; // Return overall status
// }

/**
 * @brief Initializes the simulator module.
 *
 */
w_status_t simulator_init() {
    // Initialize simulator specific things if needed
    return W_SUCCESS;
}

// --- HIL Mechanism Logic ---

/**
 * @brief Manually advance the RTOS tick counter by one
 *
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 *
 * @return true if a context switch is requested, false otherwise
 */
void hil_increment_tick(void) {
    /* --- Standard FreeRTOS Tick Logic --- */
    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
    executes all interrupts must be unmasked.  There is therefore no need to
    save and then restore the interrupt mask value as its value is already
    known. */
    portDISABLE_INTERRUPTS();
    {
        /* Increment the RTOS tick. */
        if (xTaskIncrementTick() != pdFALSE) {
            /* A context switch is required.  Context switching is performed in
            the PendSV interrupt.  Pend the PendSV interrupt. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    portENABLE_INTERRUPTS();
}

/**
 * @brief Process HIL UART data and trigger tick updates
 *
 * This function should be called from the UART receive interrupt
 * handler. It validates the frame, processes simulator data,
 * and triggers the tick update.
 *
 * @param data Pointer to the received UART data (including header/footer)
 * @param size Size of the received data (including header/footer)
 * @return Status of the processing (W_SUCCESS, W_INVALID_PARAM)
 */
// w_status_t hil_process_uart_data(const uint8_t *data, uint16_t size) {
//     if (data == NULL) {
//         return W_INVALID_PARAM;
//     }

//     // // Validate frame size - should match the exact expected size
//     // if (size != HIL_UART_FRAME_SIZE) {
//     //     return W_INVALID_PARAM;
//     // }

//     // Check header and footer
//     // if (data[0] != HIL_UART_HEADER_CHAR || data[size - 1] != HIL_UART_FOOTER_CHAR) {
//     //     return W_INVALID_PARAM;
//     // }

//     // Process the simulator data (payload is between header and footer)
//     // Use HIL_UART_PAYLOAD_SIZE which should be based on HilDataPacket now
//     w_status_t sim_status = simulator_process_data_internal(data + 1, HIL_UART_PAYLOAD_SIZE);

//     // Increment the RTOS tick counter regardless of simulator processing result
//     hil_increment_tick();

//     // Return success only if frame validation AND simulator processing were OK
//     return (sim_status == W_SUCCESS) ? W_SUCCESS : W_FAILURE;
// }