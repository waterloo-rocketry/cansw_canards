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
static const uint32_t NAV_LOG_DATA_TIMEOUT = 0;

static const uint8_t NUM_VEL_AXIS = 3;

// maximum number of times that nav can fail to run before error is reported
static const uint8_t MAXIMUM_NAV_NOT_RUN_COUNT = 3;

// Rate limit CAN tx: only send data at 10Hz, every 100ms
// TODO: if kept change to static const
#define ESTIMATOR_CAN_TX_PERIOD_MS 100
#define ESTIMATOR_CAN_TX_RATE (ESTIMATOR_CAN_TX_PERIOD_MS / ESTIMATOR_TASK_PERIOD_MS)
// wait for imu data for >5ms to avoid false failure if imu takes like 5.1ms
#define DATA_WAIT_MS 10

// Error tracking
static navigator_error_data_t navigator_error_stats = {0};

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

static w_status_t nav_can_telemetry(void) {
	nav_value_handle_t nav_value_lastest_raw;

	w_status_t status = W_SUCCESS;

	if (xQueuePeek(nav_value_queue, &nav_value_lastest_raw, 0) != pdTRUE) {
		navigator_error_stats.queue_is_empty = true;
		return W_FAILURE;
	}

	uint32_t timestamp = 0;

	if (timer_get_ms(&timestamp) != W_SUCCESS) {
		navigator_error_stats.timestamp_fail_count++;
		navigator_error_stats.can_telem_tx_fail = true;
		return W_FAILURE;
	}

	w_status_t nav_pt1_enc_status = W_SUCCESS;

	// quaternion w is signed (offset into uint16); altitude and varnorm are unsigned
	int16_t orientation_w = 0;
	uint16_t altitude = 0;
	uint16_t varnorm = 0;

	nav_pt1_enc_status |= can_encode_scaled_float(
		SCALE_NAV_ORIENTATION, nav_value_lastest_raw.orientation[0], &orientation_w);

	nav_pt1_enc_status |=
		can_encode_scaled_float(SCALE_NAV_ALTITUDE, nav_value_lastest_raw.altitude, &altitude);

	nav_pt1_enc_status |= can_encode_scaled_float(
		SCALE_NAV_VARIANCE_NORM, nav_value_lastest_raw.variance_norm, &varnorm);

	if (W_SUCCESS == nav_pt1_enc_status) {
		can_msg_t msg_qw_alt_var = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)timestamp,
										 DEM_3D_SENSOR_CANARD_NAV_ORI_QW_ALT_VARNORM,
										 (uint16_t)(orientation_w + TELEMETRY_INT16_OFFSET),
										 altitude,
										 varnorm,
										 &msg_qw_alt_var);

		if (can_handler_transmit(&msg_qw_alt_var) != W_SUCCESS) {
			navigator_error_stats.can_telem_tx_fail_count++;
			navigator_error_stats.can_telem_tx_fail = true;
			status |= W_FAILURE;
		}

	} else {
		navigator_error_stats.can_encode_fail_count++;
		navigator_error_stats.can_telem_tx_fail = true;
		status = W_FAILURE;
	}

	// scale quaternion x, y, z (orientation[1..3]) and offset into uint16
	int16_t orientation_x = 0;
	int16_t orientation_y = 0;
	int16_t orientation_z = 0;

	w_status_t nav_pt2_enc_status = W_SUCCESS;

	nav_pt2_enc_status |= can_encode_scaled_float(
		SCALE_NAV_ORIENTATION, nav_value_lastest_raw.orientation[1], &orientation_x);

	nav_pt2_enc_status |= can_encode_scaled_float(
		SCALE_NAV_ORIENTATION, nav_value_lastest_raw.orientation[2], &orientation_y);

	nav_pt2_enc_status |= can_encode_scaled_float(
		SCALE_NAV_ORIENTATION, nav_value_lastest_raw.orientation[3], &orientation_z);
	if (W_SUCCESS == nav_pt2_enc_status) {
		can_msg_t msg_qxyz = {0};
		build_3d_analog_sensor_16bit_msg(PRIO_LOW,
										 (uint16_t)timestamp,
										 DEM_3D_SENSOR_CANARD_NAV_ORI_QX_QY_QZ,
										 (uint16_t)(orientation_x + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(orientation_y + TELEMETRY_INT16_OFFSET),
										 (uint16_t)(orientation_z + TELEMETRY_INT16_OFFSET),
										 &msg_qxyz);

		if (can_handler_transmit(&msg_qxyz) != W_SUCCESS) {
			navigator_error_stats.can_telem_tx_fail_count++;
			navigator_error_stats.can_telem_tx_fail = true;
			status |= W_FAILURE;
		}
	} else {
		navigator_error_stats.can_encode_fail_count++;
		navigator_error_stats.can_telem_tx_fail = true;
		status = W_FAILURE;
	}

	const can_dem_2d_sensor_id_t axis_ids[3] = {
		DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_X,
		DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_Y,
		DEM_2D_SENSOR_CANARD_NAV_VEL_ANGLE_VEL_Z,
	};

	for (uint8_t axis = 0; axis < NUM_VEL_AXIS; axis++) {
		w_status_t nav_vel_ang_vel_enc_status = W_SUCCESS;

		int32_t velocity = 0;
		nav_vel_ang_vel_enc_status |= can_encode_scaled_float(
			SCALE_NAV_VELOCITY, (float32_t)nav_value_lastest_raw.velocity[axis], &velocity);

		int32_t angular_velocity = 0;
		nav_vel_ang_vel_enc_status |=
			can_encode_scaled_float(SCALE_NAV_ANGULAR_VELOCITY,
									(float32_t)nav_value_lastest_raw.angular_velocity[axis],
									&angular_velocity);

		if (W_SUCCESS == nav_vel_ang_vel_enc_status) {
			can_msg_t msg = {0};
			build_2d_analog_sensor_24bit_msg(PRIO_LOW,
											 (uint16_t)timestamp,
											 axis_ids[axis],
											 (uint32_t)(velocity + TELEMETRY_INT24_OFFSET),
											 (uint32_t)(angular_velocity + TELEMETRY_INT24_OFFSET),
											 &msg);

			if (can_handler_transmit(&msg) != W_SUCCESS) {
				navigator_error_stats.can_telem_tx_fail_count++;
				navigator_error_stats.can_telem_tx_fail = true;
				status |= W_FAILURE;
			}
		} else {
			navigator_error_stats.can_encode_fail_count++;
			navigator_error_stats.can_telem_tx_fail = true;
			status = W_FAILURE;
		}
	}

	return status;
}

static w_status_t nav_sd_telemetry(void) {
	nav_value_handle_t nav_value_lastest_raw;

	w_status_t status = W_SUCCESS;

	if (xQueuePeek(nav_value_queue, &nav_value_lastest_raw, 0) != pdTRUE) {
		navigator_error_stats.queue_is_empty = true;
		return W_FAILURE;
	}

	// set up the log data
	log_data_container_t log_container = {0};

	// nav pt1
	log_container.navigator_pt1.orient_w = (float32_t)nav_value_lastest_raw.orientation[0];
	log_container.navigator_pt1.orient_x = (float32_t)nav_value_lastest_raw.orientation[1];
	log_container.navigator_pt1.orient_y = (float32_t)nav_value_lastest_raw.orientation[2];
	log_container.navigator_pt1.orient_z = (float32_t)nav_value_lastest_raw.orientation[3];
	log_container.navigator_pt1.altitude = (float32_t)nav_value_lastest_raw.altitude;
	log_container.navigator_pt1.variance_norm = (float32_t)nav_value_lastest_raw.variance_norm;

	if (log_data(NAV_LOG_DATA_TIMEOUT, LOG_TYPE_NAVIGATOR_PT1, &log_container) != W_SUCCESS) {
		navigator_error_stats.log_data_fail_count++;
		navigator_error_stats.can_telem_tx_fail = true;
		status |= W_FAILURE;
	}

	// nav pt2
	log_container.navigator_pt2.velocity.x = (float32_t)nav_value_lastest_raw.velocity[0];
	log_container.navigator_pt2.velocity.y = (float32_t)nav_value_lastest_raw.velocity[1];
	log_container.navigator_pt2.velocity.z = (float32_t)nav_value_lastest_raw.velocity[2];

	log_container.navigator_pt2.angular_velocity.x =
		(float32_t)nav_value_lastest_raw.angular_velocity[0];
	log_container.navigator_pt2.angular_velocity.y =
		(float32_t)nav_value_lastest_raw.angular_velocity[1];
	log_container.navigator_pt2.angular_velocity.z =
		(float32_t)nav_value_lastest_raw.angular_velocity[2];

	if (log_data(NAV_LOG_DATA_TIMEOUT, LOG_TYPE_NAVIGATOR_PT2, &log_container) != W_SUCCESS) {
		navigator_error_stats.log_data_fail_count++;
		navigator_error_stats.can_telem_tx_fail = true;
		status |= W_FAILURE;
	}

	return status;
}

// ---------- public functions ----------

w_status_t navigator_init(void) {
	// Initialize error tracking
	navigator_error_stats = (navigator_error_data_t){0};

	// create mailbox queue for nav values
	nav_value_queue = xQueueCreate(1, sizeof(nav_value_handle_t));
	if (NULL == nav_value_queue) {
		log_text(0, LOG_LVL_FATAL, "navigator", "unable to allocate memory for queue.");
		return W_FAILURE;
	}

	static const telemetry_source_config_t telemetry_sources[] = {

		{"Nav CAN Telem", nav_can_telemetry, STATE_PAD_FILTER, 1000 / 10},
		{"Nav CAN Telem", nav_can_telemetry, STATE_PAD_NAV, 1000 / 10},
		{"Nav CAN Telem", nav_can_telemetry, STATE_BOOST, 1000 / 10},
		{"Nav CAN Telem", nav_can_telemetry, STATE_ACT_ALLOWED, 1000 / 10},

		{"Nav SD Telem", nav_sd_telemetry, STATE_PAD_FILTER, 1000 / 20},
		{"Nav SD Telem", nav_sd_telemetry, STATE_PAD_NAV, 1000 / MAX_LOGGING_RATE_HZ},
		{"Nav SD Telem", nav_sd_telemetry, STATE_BOOST, 1000 / MAX_LOGGING_RATE_HZ},
		{"Nav SD Telem", nav_sd_telemetry, STATE_ACT_ALLOWED, 1000 / MAX_LOGGING_RATE_HZ},
		{"Nav SD Telem", nav_sd_telemetry, STATE_RECOVERY, 1000 / 20},
		{"Nav SD Telem", nav_sd_telemetry, STATE_SLEEPY, 1000 / 1},
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

	navigator_error_stats.is_init = true;

	return W_SUCCESS;
}

w_status_t navigator_step(const navigator_input_t *p_input, const uint32_t timestamp_tenth_ms,
						  navigator_ctx_t *p_ctx, navigator_output_t *p_output) {
	if ((NULL == p_input) || (NULL == p_ctx) || (NULL == p_output)) {
		navigator_error_stats.null_ctx_count++;
		navigator_error_stats.ctx_is_null = true;
		return W_INVALID_PARAM;
	}
	// calculate remainder navigator data
	float64_t dt_sec = (timestamp_tenth_ms - (p_ctx->last_run_tenth_ms)) * TENTH_MS_TO_SEC;
	bool in_flight_phase =
		(STATE_PAD_FILTER != p_input->fsm_state); // since earliest entry point is pad filter

	// construct sensor inputs
	gnc_navigator_sensor_input_t codegen_sensor_input = {0};

	// status represents if this sensor should be used in this cycle of nav
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
	if (is_run) { // if nav ran
		p_ctx->last_run_tenth_ms = timestamp_tenth_ms;
	} else {
		navigator_error_stats.nav_not_run_count++;
		if (navigator_error_stats.nav_not_run_count > MAXIMUM_NAV_NOT_RUN_COUNT) {
			navigator_error_stats.nav_not_run = true;
			navigator_error_stats.nav_not_run_count = 0;
		}
	}

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
		navigator_error_stats.null_ctx_count++;
		navigator_error_stats.ctx_is_null = true;
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

health_status_t navigator_get_status(void) {
	health_status_t status = {.severity = CANARDS_HEALTH_SEVERITY_HEALTH_OK,
							  .module_id = CANARDS_MODULE_ID_NAVIGATOR,
							  .error_bitfield = 0};

	if (navigator_error_stats.ctx_is_null) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_INVALID_PARAM_OFFSET;
		navigator_error_stats.ctx_is_null = false;
	}

	if (navigator_error_stats.can_telem_tx_fail) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_TX_FAILURE_OFFSET;
		navigator_error_stats.can_telem_tx_fail = false;
	}

	if (navigator_error_stats.nav_not_run) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_LOOP_TIMING_OFFSET;
		navigator_error_stats.nav_not_run = false;
	}

	if (navigator_error_stats.queue_is_empty) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_ERROR;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_OS_OFFSET;
		navigator_error_stats.queue_is_empty = false;
	}

	if (!navigator_error_stats.is_init) {
		status.severity = CANARDS_HEALTH_SEVERITY_HEALTH_FATAL;
		status.error_bitfield |= 1 << CANARDS_MODULE_E_NOT_INIT_OFFSET;
	}

	// Log all error statistics

	log_text(10,
			 LOG_LVL_INFO,
			 "navigator",
			 "init=%lu, null_ctx=%lu, nav_not_run=%lu, can_encode_fail=%lu",
			 navigator_error_stats.is_init,
			 navigator_error_stats.null_ctx_count,
			 navigator_error_stats.nav_not_run_count,
			 navigator_error_stats.can_encode_fail_count);

	log_text(10,
			 LOG_LVL_INFO,
			 "navigator",
			 "timestamp_fail=%lu, log_data_fail=%lu, can_telem_tx_fail=%lu",
			 navigator_error_stats.timestamp_fail_count,
			 navigator_error_stats.log_data_fail_count,
			 navigator_error_stats.can_telem_tx_fail_count);

	return status;
}

