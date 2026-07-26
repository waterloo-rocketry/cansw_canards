#include "FreeRTOS.h"
#include "math.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "canlib.h"

#include "GNC_codegen.h"
#include "application/can_handler/can_handler.h"
#include "application/can_handler/can_telemetry_scaling.h"
#include "application/health_checks/health_checks.h"
#include "application/logger/log.h"
#include "application/navigator/navigator.h"
#include "application/telemetry/telemetry.h"
#include "common/gnc/gnc_types.h"
#include "drivers/timer/timer.h"

// ---------- private variables ----------
// IDEAL task period, for calculating CAN send rate limiter
static const uint32_t ESTIMATOR_TASK_PERIOD_MS = 5;
static const float64_t TENTH_MS_TO_SEC = 0.0001;

// Rate limit CAN tx: only send data at 10Hz, every 100ms
// TODO: if kept change to static const
#define ESTIMATOR_CAN_TX_PERIOD_MS 100
#define ESTIMATOR_CAN_TX_RATE (ESTIMATOR_CAN_TX_PERIOD_MS / ESTIMATOR_TASK_PERIOD_MS)
// wait for imu data for >5ms to avoid false failure if imu takes like 5.1ms
#define DATA_WAIT_MS 10

// Error tracking
static navigator_error_data_t estimator_error_stats = {0};

// Latest nav state, stored unscaled as raw floats. Scaling + offset into the
// CAN integer fields happens at telemetry-tx time (see nav_*_telemetry below)
// so no precision is lost before the scale factor is applied.
typedef struct {
	float64_t orientation[4]; // quaternion w, x, y, z
	float64_t angular_velocity[3];
	float64_t velocity[3];
	float64_t altitude;
	float64_t variance_norm;
} nav_value_handle_t;

static QueueHandle_t nav_value_queue; // mailbox queue to hold the latest nav values

static w_status_t nav_orientation_altitude_varnorm_telemetry(void) {
	nav_value_handle_t nav_value_lastest_raw;

	if (xQueuePeek(nav_value_queue, &nav_value_lastest_raw, 0) == pdTRUE) {
		// scale quaternion x, y, z (orientation[1..3]) and offset into uint16
		int16_t orientation_x;
		int16_t orientation_y;
		int16_t orientation_z;

		if (W_SUCCESS != can_encode_scaled_float(SCALE_NAV_ORIENTATION,
											   nav_value_lastest_raw.orientation[1],
											   &orientation_x)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for orientaion x.");
		}

		if (W_SUCCESS != can_encode_scaled_float(SCALE_NAV_ORIENTATION,
											   nav_value_lastest_raw.orientation[2],
											   &orientation_y)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for orientaion y.");
		}

		if (W_SUCCESS != can_encode_scaled_float(SCALE_NAV_ORIENTATION,
											   nav_value_lastest_raw.orientation[3],
											   &orientation_z)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for orientaion z.");
		}

		// quaternion w is signed (offset into uint16); altitude and varnorm are unsigned
		int16_t orientation_w;
		uint16_t altitude;
		uint16_t varnorm;

		if (W_SUCCESS != can_encode_scaled_float(SCALE_NAV_ORIENTATION,
											   nav_value_lastest_raw.orientation[0],
											   &orientation_w)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for orientaion w.");
		}

		if (W_SUCCESS !=
			can_encode_scaled_float(SCALE_NAV_ALTITUDE, nav_value_lastest_raw.altitude, &altitude)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for altitude.");
		}

		if (W_SUCCESS != can_encode_scaled_float(SCALE_NAV_VARIANCE_NORM,
											   nav_value_lastest_raw.variance_norm,
											   &varnorm)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for variance norm.");
		}

		uint32_t timestamp;

		if (timer_get_ms(&timestamp) != W_SUCCESS) {
			log_text(0, LOG_LVL_WARN, "navigator", "Failed to get timestamp for can msg tx");
			return W_FAILURE;
		}

		w_status_t status = W_SUCCESS;

		can_msg_t msg_qxyz;
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)timestamp,
										 DEM_3D_SENSOR_CANARD_NAV_ORI_QX_QY_QZ,
										 (uint16_t)(orientation_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(orientation_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(orientation_z + TELEMETRY_INT16_OFFSET),
										 &msg_qxyz);

		if (can_handler_transmit(&msg_qxyz) != W_SUCCESS) {
			log_text(0,
					 LOG_LVL_WARN,
					 "navigator",
					 "Failed to transmit orientation x y z values through can.");
			status = W_FAILURE;
		}

		can_msg_t msg_qw_alt_var;
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)timestamp,
										 DEM_3D_SENSOR_CANARD_NAV_ORI_QW_ALT_VARNORM,
										 (uint16_t)(orientation_w + TELEMETRY_INT16_OFFSET),
										 altitude,
										 varnorm,
										 &msg_qw_alt_var);

		if (can_handler_transmit(&msg_qw_alt_var) != W_SUCCESS) {
			log_text(
				0,
				LOG_LVL_WARN,
				"navigator",
				"Failed to transmit orientation w, altitude, variance norm values through can.");
			status = W_FAILURE;
		}

		return status;
	} else {
		log_text(0,
				 LOG_LVL_WARN,
				 "navigator",
				 "Failed to peek mailbox queue while sending current nav values through can.");

		return W_FAILURE;
	}

	return W_SUCCESS;
}

// Velocity and angular velocity are carried per-axis by the 2D 24bit messages
// DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_{X,Y,Z}: each message packs that axis's
// (velocity, angular_velocity) pair, both scaled as int24 (scale table entries
// SCALE_NAV_VELOCITY / SCALE_NAV_ANGULAR_VELOCITY).
static w_status_t nav_velocity_angular_velocity_telemetry(void) {
	nav_value_handle_t nav_value_lastest_raw;

	if (xQueuePeek(nav_value_queue, &nav_value_lastest_raw, 0) != pdTRUE) {
		log_text(0,
				 LOG_LVL_WARN,
				 "navigator",
				 "Failed to peek mailbox queue while sending current nav values through can.");
		return W_FAILURE;
	}

	const can_dem_2d_sensor_id_t axis_ids[3] = {
		DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_X,
		DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_Y,
		DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_Z,
	};

	uint32_t timestamp;
	if (timer_get_ms(&timestamp) != W_SUCCESS) {
		log_text(0, LOG_LVL_WARN, "navigator", "Failed to get timestamp for can msg tx");
		return W_FAILURE;
	}

	w_status_t status = W_SUCCESS;
	for (int axis = 0; axis < 3; axis++) {
		int32_t velocity = 0;
		if (W_SUCCESS != can_encode_scaled_float(SCALE_NAV_VELOCITY,
												 (float32_t)nav_value_lastest_raw.velocity[axis],
												 &velocity)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for velocity.");
			status = W_FAILURE;
		}

		int32_t angular_velocity = 0;
		if (W_SUCCESS !=
			can_encode_scaled_float(SCALE_NAV_ANGULAR_VELOCITY,
									(float32_t)nav_value_lastest_raw.angular_velocity[axis],
									&angular_velocity)) {
			log_text(0, LOG_LVL_WARN, "navigator", "Can encode failed for angular velocity.");
			status = W_FAILURE;
		}

		can_msg_t msg;
		build_2d_analog_sensor_24bit_msg(PRIO_LOW,
										 (uint16_t)timestamp,
										 axis_ids[axis],
										 (uint32_t)(velocity + TELEMETRY_INT24_OFFSET),
										 (uint32_t)(angular_velocity + TELEMETRY_INT24_OFFSET),
										 &msg);

		if (can_handler_transmit(&msg) != W_SUCCESS) {
			log_text(0,
					 LOG_LVL_WARN,
					 "navigator",
					 "Failed to transmit velocity/angular velocity values through can.");
			status = W_FAILURE;
		}
	}

	return status;
}

// ---------- public functions ----------

w_status_t navigator_init(void) {
	// Initialize error tracking
	estimator_error_stats = (navigator_error_data_t){.is_init = true};

	// create mailbox queue for nav values
	nav_value_queue = xQueueCreate(1, sizeof(nav_value_handle_t));
	configASSERT(nav_value_queue != NULL);

	static const telemetry_source_config_t telemetry_sources[] = {
		{"Navigator angular velocity and velocity",
		 nav_velocity_angular_velocity_telemetry,
		 STATE_PAD_FILTER,
		 1000 / 10},
		{"Navigator angular velocity and velocity",
		 nav_velocity_angular_velocity_telemetry,
		 STATE_PAD_NAV,
		 1000 / 10},
		{"Navigator angular velocity and velocity",
		 nav_velocity_angular_velocity_telemetry,
		 STATE_BOOST,
		 1000 / 5},
		{"Navigator angular velocity and velocity",
		 nav_velocity_angular_velocity_telemetry,
		 STATE_ACT_ALLOWED,
		 1000 / 5},

		{"Navigator orientation, altitude, and variance norm",
		 nav_orientation_altitude_varnorm_telemetry,
		 STATE_PAD_FILTER,
		 1000 / 10},
		{"Navigator orientation, altitude, and variance norm",
		 nav_orientation_altitude_varnorm_telemetry,
		 STATE_PAD_NAV,
		 1000 / 10},
		{"Navigator orientation, altitude, and variance norm",
		 nav_orientation_altitude_varnorm_telemetry,
		 STATE_BOOST,
		 1000 / 10},
		{"Navigator orientation, altitude, and variance norm",
		 nav_orientation_altitude_varnorm_telemetry,
		 STATE_ACT_ALLOWED,
		 1000 / 10},
	};

	static const size_t telemetry_source_count =
		sizeof(telemetry_sources) / sizeof(telemetry_source_config_t);
	w_status_t telemetry_register_status = W_SUCCESS;

	// Register callbacks and check
	for (size_t i = 0; i < telemetry_source_count; i++) {
		telemetry_register_status |= telemetry_register(&telemetry_sources[i]);
	}

	if (W_SUCCESS != telemetry_register_status) {
		log_text(1, LOG_LVL_WARN, "Navigator", "Failed to register telemetry sources.");
		return W_FAILURE;
	}

	return W_SUCCESS;
}

w_status_t navigator_step(const navigator_input_t *p_input, const uint32_t timestamp_tenth_ms,
						  navigator_ctx_t *p_ctx, navigator_output_t *p_output) {
	if ((NULL == p_input) || (NULL == p_ctx) || (NULL == p_output)) {
		log_text(0, LOG_LVL_WARN, "navigator", "Invalid context ptr.");
		return W_INVALID_PARAM;
	}
	// calculate remainder navigator data
	float64_t dt_sec = (timestamp_tenth_ms - (p_ctx->last_run_tenth_ms)) * TENTH_MS_TO_SEC;
	bool in_flight_phase =
		(STATE_PAD_FILTER != p_input->fsm_state); // since earliest entry point is pad filter

	// construct sensor inputs
	gnc_navigator_sensor_input_t codegen_sensor_input = {0};

	// status repersents if this sensor should be used in this cycle of nav
	// lsm6
	memcpy(codegen_sensor_input.board_accel.meas,
		   p_input->sensor_data->board_meas.board_imu.accel.array,
		   sizeof(codegen_sensor_input.board_accel.meas));
	codegen_sensor_input.board_accel.status = p_input->sensor_data->board_meas.board_imu.is_new;

	memcpy(codegen_sensor_input.board_gyro.meas,
		   p_input->sensor_data->board_meas.board_imu.gyro.array,
		   sizeof(codegen_sensor_input.board_gyro.meas));
	codegen_sensor_input.board_gyro.status = p_input->sensor_data->board_meas.board_imu.is_new;

	// Baro
	codegen_sensor_input.board_baro.meas = p_input->sensor_data->board_meas.board_baro.meas;
	codegen_sensor_input.board_baro.status = p_input->sensor_data->board_meas.board_baro.is_new;

	// Mag
	memcpy(codegen_sensor_input.board_mag.meas,
		   p_input->sensor_data->board_meas.board_mag.meas.array,
		   sizeof(codegen_sensor_input.board_mag.meas));
	codegen_sensor_input.board_mag.status = p_input->sensor_data->board_meas.board_mag.is_new;

	// MTI
	memcpy(codegen_sensor_input.mti_accel.meas,
		   p_input->sensor_data->mti_meas.mti_accel.meas.array,
		   sizeof(codegen_sensor_input.mti_accel.meas));
	codegen_sensor_input.mti_accel.status = p_input->sensor_data->mti_meas.mti_accel.is_new;

	memcpy(codegen_sensor_input.mti_gyro.meas,
		   p_input->sensor_data->mti_meas.mti_gyro.meas.array,
		   sizeof(codegen_sensor_input.mti_gyro.meas));
	codegen_sensor_input.mti_gyro.status = p_input->sensor_data->mti_meas.mti_gyro.is_new;

	memcpy(codegen_sensor_input.mti_mag.meas,
		   p_input->sensor_data->mti_meas.mti_mag.meas.array,
		   sizeof(codegen_sensor_input.mti_mag.meas));
	codegen_sensor_input.mti_mag.status = p_input->sensor_data->mti_meas.mti_mag.is_new;

	codegen_sensor_input.mti_baro.meas = p_input->sensor_data->mti_meas.mti_baro.meas;
	codegen_sensor_input.mti_baro.status = p_input->sensor_data->mti_meas.mti_baro.is_new;

	// AD Accel
	memcpy(codegen_sensor_input.ad_accel.meas,
		   p_input->sensor_data->ad_meas.ad_accel.meas.array,
		   sizeof(codegen_sensor_input.ad_accel.meas));
	codegen_sensor_input.ad_accel.status = p_input->sensor_data->ad_meas.ad_accel.is_new;

	codegen_sensor_input.ad_gyro.meas[0] = p_input->sensor_data->ad_meas.ad_gyro.meas; // x axis
	codegen_sensor_input.ad_gyro.meas[1] = 0.0;
	codegen_sensor_input.ad_gyro.meas[2] = 0.0;
	codegen_sensor_input.ad_gyro.status = p_input->sensor_data->ad_meas.ad_gyro.is_new;

	bool is_run = false;

	navigation_codegen_entry(p_ctx->p_gnc_stack_data,
							 dt_sec,
							 in_flight_phase,
							 p_ctx->gnc_navigator_ctx.x.arr,
							 p_ctx->gnc_navigator_ctx.P,
							 &(p_ctx->gnc_navigator_ctx.bias),
							 &(p_ctx->gnc_navigator_ctx.sensor_filter),
							 &codegen_sensor_input,
							 &(p_output->cov_norm),
							 p_output->roll_state,
							 &(p_output->dynamic_pressure),
							 &is_run);
#ifdef HIL
	log_data_container_t container = {0};
	container.navigator_pt1.orient_w = p_ctx->gnc_navigator_ctx.x.q.w;
	container.navigator_pt1.orient_x = p_ctx->gnc_navigator_ctx.x.q.x;
	container.navigator_pt1.orient_y = p_ctx->gnc_navigator_ctx.x.q.y;
	container.navigator_pt1.orient_z = p_ctx->gnc_navigator_ctx.x.q.z;
	container.navigator_pt1.altitude = p_ctx->gnc_navigator_ctx.x.altitude;
	container.navigator_pt1.variance_norm = p_output->cov_norm;

	log_data(1, LOG_TYPE_NAVIGATOR_PT1, (log_data_container_t *)&container);

	container.navigator_pt2.velocity.x = p_ctx->gnc_navigator_ctx.x.vel.x;
	container.navigator_pt2.velocity.y = p_ctx->gnc_navigator_ctx.x.vel.y;
	container.navigator_pt2.velocity.z = p_ctx->gnc_navigator_ctx.x.vel.z;
	container.navigator_pt2.angular_velocity.x = p_ctx->gnc_navigator_ctx.x.ang_rate.x;
	container.navigator_pt2.angular_velocity.y = p_ctx->gnc_navigator_ctx.x.ang_rate.y;
	container.navigator_pt2.angular_velocity.z = p_ctx->gnc_navigator_ctx.x.ang_rate.z;

	log_data(1, LOG_TYPE_NAVIGATOR_PT2, (log_data_container_t *)&container);

	log_text(1,
			 LOG_LVL_INFO,
			 "bias",
			 "board_baro bias %f, mti_baro %f",
			 p_ctx->gnc_navigator_ctx.bias.board_baro,
			 p_ctx->gnc_navigator_ctx.bias.mti_baro);
	log_text(1,
			 LOG_LVL_INFO,
			 "sensorfilter",
			 "board_baro bias %f, mti_baro %f",
			 p_ctx->gnc_navigator_ctx.sensor_filter.board_baro,
			 p_ctx->gnc_navigator_ctx.sensor_filter.mti_baro);
#endif

#ifdef HIL
	p_ctx->last_run_tenth_ms = timestamp_tenth_ms;
#else
	if (is_run) { // if nav ran
		p_ctx->last_run_tenth_ms = timestamp_tenth_ms;
	} else {
		log_text(0, LOG_LVL_WARN, "Navigator", "Nav failed to run");
	}
#endif

	nav_value_handle_t nav_latest_values;

	// Copied unscaled; scaling happens at tx.
	memcpy(nav_latest_values.orientation,
		   p_ctx->gnc_navigator_ctx.x.q.array,
		   sizeof(nav_latest_values.orientation));
	memcpy(nav_latest_values.angular_velocity,
		   p_ctx->gnc_navigator_ctx.x.ang_rate.array,
		   sizeof(nav_latest_values.angular_velocity));
	memcpy(nav_latest_values.velocity,
		   p_ctx->gnc_navigator_ctx.x.vel.array,
		   sizeof(nav_latest_values.velocity));
	nav_latest_values.altitude = p_ctx->gnc_navigator_ctx.x.altitude;

	// variance_norm is not part of the x-state; it comes from the nav output
	nav_latest_values.variance_norm = p_output->cov_norm;

	xQueueOverwrite(nav_value_queue, &nav_latest_values);

	return W_SUCCESS;
}

w_status_t pad_filter_init(navigator_ctx_t *p_ctx, all_sensors_data_t *p_sensor_data) {
	if ((NULL == p_ctx) || (NULL == p_sensor_data)) {
		log_text(0, LOG_LVL_WARN, "navigator", "Invalid context ptr.");
		return W_INVALID_PARAM;
	}

	// check if sensor is alive
	if (p_sensor_data->board_meas.board_imu.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.board_accel,
			   p_sensor_data->board_meas.board_imu.accel.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.board_accel));
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.board_gyro,
			   p_sensor_data->board_meas.board_imu.gyro.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.board_gyro));
	}
	if (p_sensor_data->board_meas.board_mag.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.board_mag,
			   p_sensor_data->board_meas.board_mag.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.board_mag));
	}
	if (p_sensor_data->board_meas.board_baro.is_new) {
		p_ctx->gnc_navigator_ctx.sensor_filter.board_baro =
			p_sensor_data->board_meas.board_baro.meas;
	}

	if (p_sensor_data->ad_meas.ad_accel.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.ad_accel,
			   p_sensor_data->ad_meas.ad_accel.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.ad_accel));
	}
	if (p_sensor_data->ad_meas.ad_gyro.is_new) {
		p_ctx->gnc_navigator_ctx.sensor_filter.ad_gyro[0] = p_sensor_data->ad_meas.ad_gyro.meas;
		p_ctx->gnc_navigator_ctx.sensor_filter.ad_gyro[1] = 0;
		p_ctx->gnc_navigator_ctx.sensor_filter.ad_gyro[2] = 0;
	}

	if (p_sensor_data->mti_meas.mti_accel.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.mti_accel,
			   p_sensor_data->mti_meas.mti_accel.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.mti_accel));
	}
	if (p_sensor_data->mti_meas.mti_gyro.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.mti_gyro,
			   p_sensor_data->mti_meas.mti_gyro.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.mti_gyro));
	}
	if (p_sensor_data->mti_meas.mti_mag.is_new) {
		memcpy(p_ctx->gnc_navigator_ctx.sensor_filter.mti_mag,
			   p_sensor_data->mti_meas.mti_mag.meas.array,
			   sizeof(p_ctx->gnc_navigator_ctx.sensor_filter.mti_mag));
	}
	if (p_sensor_data->mti_meas.mti_baro.is_new) {
		p_ctx->gnc_navigator_ctx.sensor_filter.mti_baro = p_sensor_data->mti_meas.mti_baro.meas;
	}
	return W_SUCCESS;
}

// TODO: to be revived
// w_status_t navigator_log_state_to_can(const x_state_t *current_state) {
// 	can_msg_t msg;
// 	uint32_t current_time_ms;
// 	w_status_t status = W_SUCCESS;

// 	if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
// 		current_time_ms = 0; // Default to 0 if timer fails
// 	}
// 	uint16_t timestamp_16bit = (uint16_t)current_time_ms;

// 	// TODO: Redo how messages are built and sent
// 	// Iterate through all defined state IDs
// 	for (can_state_est_id_t state_id = 0; state_id < STATE_ID_ENUM_MAX; ++state_id) {
// 		// The x_state_t union maps directly to the enum order if accessed as an array
// 		// Convert the doubles in x_state_t to floats for CAN message
// 		float state_value = (float)current_state->array[state_id];

// 		if (!build_state_est_data_msg(PRIO_LOW, timestamp_16bit, state_id, &state_value, &msg)) {
// 			log_text(0, "Estimator", "Failed to build CAN message for state ID %d", state_id);
// 			estimator_error_stats.can_log_fails++;
// 			status = W_FAILURE; // Mark as failure but continue trying other states
// 			continue;
// 		}

// 		if (W_SUCCESS != can_handler_transmit(&msg)) {
// 			log_text(0, "Estimator", "Failed to transmit CAN message for state ID %d", state_id);
// 			estimator_error_stats.can_log_fails++;
// 			status = W_FAILURE; // Mark as failure but continue trying other states
// 		}
// 	}

// 	return status;
// }

health_status_t navigator_get_status(void) {
	// Log all error statistics
	log_text(0,
			 LOG_LVL_INFO,
			 "Navigator",
			 "imu_timeouts=%lu, encoder_miss=%lu, controller_miss=%lu,",
			 estimator_error_stats.imu_data_timeouts,
			 estimator_error_stats.encoder_data_fails,
			 estimator_error_stats.controller_data_fails);
	log_text(0,
			 LOG_LVL_INFO,
			 "Navigator",
			 "pad_filter_fails=%lu, can_log_fails=%lu, invalid_phase=%lu",
			 estimator_error_stats.pad_filter_fails,
			 estimator_error_stats.can_log_fails,
			 estimator_error_stats.invalid_phase_errors);

	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_NAVIGATOR,
							  .error_bitfield = 0};

	return status;
}

