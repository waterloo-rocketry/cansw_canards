/**
 * @file hil.h
 * @brief Hardware-in-the-Loop interface
 *
 * This module provides functionality to support Hardware-in-the-Loop (HIL)
 * testing by allowing external control over RTOS ticks.
 */

#ifndef HIL_H
#define HIL_H

#include "rocketlib/include/common.h"
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

// --- Struct Definitions ---
// Ensure this matches the structure sent by the simulation host (HIL Test Plan)
// Use __attribute__((packed)) to prevent padding
typedef struct __attribute__((packed)) {
    // Offset 0: Encoder
    float encoder_angle; // rad
    // Offset 4: Movella
    float movella_accel_x; // g
    float movella_accel_y; // g
    float movella_accel_z; // g
    float movella_gyro_x; // dps
    float movella_gyro_y; // dps
    float movella_gyro_z; // dps
    float movella_mag_x; // uT
    float movella_mag_y; // uT
    float movella_mag_z; // uT
    float movella_pressure; // bar
    float altimu_accel_x; // g
    float altimu_accel_y; // g
    float altimu_accel_z; // g
    float altimu_gyro_x; // dps
    float altimu_gyro_y; // dps
    float altimu_gyro_z; // dps
    float altimu_mag_x; // uT
    float altimu_mag_y; // uT
    float altimu_mag_z; // uT
    float altimu_pressure; // bar
} HilDataPacket;

// HIL UART protocol constants
// the header and footer are defined in the hil test doc.
// the header is 4 chars so that the payload is word-aligned.
// the footer is 1 byte cuz it doesnt matter.
#define HIL_UART_FOOTER_CHAR '\n'
#define HIL_UART_PAYLOAD_SIZE (sizeof(HilDataPacket)) // Use the updated struct size
#define HIL_UART_FRAME_SIZE (4 + HIL_UART_PAYLOAD_SIZE + 1) // Header + Payload + Footer

// time simulink spends in idle before "launch" starts. time_launch = 5; % pad delay time
// actually the real time is 5sec, but qd would happen a moment before, so make this 4sec (4000ms)
#define HIL_LAUNCH_TIMESTAMP_MS 500

extern uint8_t hil_uart_rx_data[HIL_UART_FRAME_SIZE]; // Buffer for HIL UART data
// Counter for received packages so we can skip some (only process every 5th packet)
extern uint32_t package_counter;
extern uint32_t wrong_format_packets;
extern uint32_t wrong_footer_packets;
extern _Atomic volatile bool hil_imu_inputs_ready;

/**
 * @brief Initializes the simulator logic (e.g., Canard setup if needed here).
 *
 * Implementation moved to hil.c
 *
 * @return w_status_t W_SUCCESS if initialization is successful, error code otherwise.
 */
w_status_t simulator_init(void);

/**
 * @brief Process HIL UART data and trigger tick updates
 *
 * This function should be called from the UART receive interrupt
 * handler. It validates the frame, processes simulator data,
 * and triggers the tick update.
 *
 * @param data Pointer to the received UART data (including header/footer)
 * @param size Size of the received data (including header/footer)
 * @return Status of the processing (W_SUCCESS, W_INVALID_PARAM, W_ERROR)
 */
w_status_t hil_process_uart_data(const uint8_t *data, uint16_t size);

/**
 * @brief Manually advance the RTOS tick counter by one
 *
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 *
 * @return true if a context switch is requested, false otherwise
 */
void hil_increment_tick(void);

#endif /* HIL_H */