#ifndef STATE_EST_H
#define STATE_EST_H

#include "application/controller/controller.h"
#include "application/estimator/estimator_types.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// measurement data from 1 arbitrary imu
typedef struct {
    uint32_t timestamp_imu;
    vector3d_t accelerometer; // gravities
    vector3d_t gyroscope; // rad/sec
    vector3d_t magnetometer; // mgauss (pololu) or arbitrary units (movella)
    float barometer; // Pa
    bool is_dead;
} estimator_imu_measurement_t;

// measurements from all imus together
typedef struct {
    estimator_imu_measurement_t pololu;
    estimator_imu_measurement_t movella;
    // HIL MODIFICATION: also pass encoder data here
    // todo: maybe this could be an actual change (make imu handler receive encoder instead of
    // estimator). because its easier to inject encoder data this way.
    float encoder_angle_rad; // radians
} estimator_all_imus_input_t;

/**
 * @brief Structure to track estimator errors and status
 */
typedef struct {
    bool is_init; /**< Initialization status flag */
    uint32_t imu_data_timeouts; /**< Count of IMU data receive timeouts */
    uint32_t encoder_data_fails; /**< Count of encoder data receive failures */
    uint32_t controller_data_fails; /**< Count of controller output retrieval failures */
    uint32_t pad_filter_fails; /**< Count of pad filter run failures */
    uint32_t can_log_fails; /**< Count of CAN logging failures */
    uint32_t invalid_phase_errors; /**< Count of invalid flight phase errors */
} estimator_error_data_t;

/**
 * @brief Used to update the imu inputs for estimator
 *
 * @note Should be called every 5ms or faster for optimal estimator performance
 *
 * @param data Pointer to the struct containing latest measurements from all imus
 */
w_status_t estimator_update_imu_data(estimator_all_imus_input_t *data);

/**
 * @brief initialize estimator module. call before creating estimator task
 */
w_status_t estimator_init();

/**
 * @brief Sends the complete state estimation data over CAN.
 *
 * Iterates through each state ID, builds a CAN message for it using the
 * current state data, and transmits it.
 *
 * @param current_state Pointer to the current state estimation data (x_state_t).
 * @return W_SUCCESS if all messages were sent successfully, W_FAILURE otherwise.
 */
w_status_t estimator_log_state_to_can(const x_state_t *current_state);

/**
 * @brief Report estimator module health status
 *
 * Retrieves and reports estimator error statistics and initialization status
 * through log messages.
 *
 * @return CAN board specific err bitfield
 */
uint32_t estimator_get_status(void);

void estimator_task(void *argument);

#endif

