/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csyn/csyn_codec.h>
#include <csyn/csyn_zros.h>

#include "FixedWingOuterLoop.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_topic.h>

LOG_MODULE_REGISTER(cubs2, LOG_LEVEL_INF);

#define CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS 20000000
#define CUBS2_CONTROL_PERIOD_US (CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS / 1000U)

struct control_context {
	struct csyn_mocap_rigid_body mocap;
	struct csyn_manual_control manual;
	csyn_rc_channels16_t control_rc;
	synapse_topic_PwmSignalOutputsData_t pwm_outputs;
	synapse_topic_VehicleHealthData_t vehicle_health;
	synapse_topic_AttitudeEstimateData_t attitude_estimate;
	synapse_topic_AttitudeCommandData_t attitude_command;
	synapse_topic_ControlLoopMetricsData_t control_loop_metrics;
	uint32_t mocap_generation;
	uint32_t manual_generation;
	uint32_t main_loop_us;
	bool previous_auto_mode;
};

static FixedWingOuterLoopState g_model;
static struct control_context g_control_ctx;
static struct zros_node g_node;
static struct zros_pub g_pwm_outputs_pub;
static struct zros_pub g_vehicle_health_pub;
static struct zros_pub g_attitude_estimate_pub;
static struct zros_pub g_attitude_command_pub;
static struct zros_pub g_control_loop_metrics_pub;

static bool read_mocap_if_updated(struct control_context *ctx)
{
	uint32_t generation = csyn_zros_generation(&topic_mocap);

	if (generation == 0U || generation == ctx->mocap_generation) {
		return false;
	}

	if (zros_topic_read(&topic_mocap, &ctx->mocap) != 0) {
		return false;
	}

	ctx->mocap_generation = generation;
	return true;
}

static bool read_manual_if_updated(struct control_context *ctx)
{
	uint32_t generation = csyn_zros_generation(&topic_manual_control);

	if (generation == 0U || generation == ctx->manual_generation) {
		return false;
	}

	if (zros_topic_read(&topic_manual_control, &ctx->manual) != 0) {
		return false;
	}

	ctx->manual_generation = generation;
	return true;
}

static void fixed_wing_map_input(FixedWingOuterLoopState *model,
				 const struct csyn_mocap_rigid_body *mocap)
{
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;

	if (mocap->valid) {
		synapse_types_Quaternionf_t quat = {
			.w = mocap->qw,
			.x = mocap->qx,
			.y = mocap->qy,
			.z = mocap->qz,
		};

		/* FixedWingOuterLoop consumes Euler [roll, pitch, yaw] in radians. */
		csyn_euler_from_quatf(&quat, &roll, &pitch, &yaw);
		pitch = -pitch;
	}

	model->position_m[0] = mocap->x;
	model->position_m[1] = mocap->y;
	model->position_m[2] = mocap->z;
	model->euler_rad[0] = roll;
	model->euler_rad[1] = pitch;
	model->euler_rad[2] = yaw;
}

static void fixed_wing_map_output(const FixedWingOuterLoopState *model,
				  csyn_rc_channels16_t *rc)
{
	*rc = (csyn_rc_channels16_t){
		.ch0 = csyn_pwm_from_centered_axis(-(float)model->aileron),
		.ch1 = csyn_pwm_from_centered_axis(-(float)model->elevator),
		.ch2 = csyn_pwm_from_throttle_axis((float)model->throttle),
		.ch3 = csyn_pwm_from_centered_axis((float)model->rudder),
		.ch4 = (int32_t)csyn_clampf((float)model->stabilizer, 1000.0f, 2000.0f),
		.ch5 = (int32_t)model->currentWaypoint,
		.ch6 = (int32_t)(1000.0f * (float)model->desiredSpeed),
		.ch7 = (int32_t)(1000.0f * (float)model->rollCommand),
		.ch8 = (int32_t)(1000.0f * (float)model->courseError),
	};
}

static void idle_output(csyn_rc_channels16_t *rc)
{
	*rc = (csyn_rc_channels16_t){
		.ch0 = 1500,
		.ch1 = 1500,
		.ch2 = 1000,
		.ch3 = 1500,
		.ch4 = 1000,
	};
}

static bool manual_control_valid(const struct csyn_manual_control *manual, int32_t *switch_us)
{
	const int32_t *channels = csyn_rc_channels_data(&manual->rc);
	bool valid = manual->valid;

	*switch_us = channels[CONFIG_CUBS2_AUTO_SWITCH_CHANNEL];

	for (size_t i = 0U; i < 5U; i++) {
		valid = valid && (channels[i] >= 900) && (channels[i] <= 2100);
	}

	return valid && (*switch_us >= 900) && (*switch_us <= 2100);
}

static bool auto_mode_selected(const struct control_context *ctx)
{
#if defined(CONFIG_CUBS2_MANUAL_OVERRIDE)
	int32_t switch_us;

	return !manual_control_valid(&ctx->manual, &switch_us) ||
	       (switch_us > CONFIG_CUBS2_AUTO_SWITCH_THRESHOLD_US);
#else
	ARG_UNUSED(ctx);
	return true;
#endif
}

static int control_pubs_init(void)
{
	struct {
		struct zros_pub *pub;
		struct zros_topic *topic;
		void *msg;
	} pubs[] = {
		{&g_pwm_outputs_pub, &topic_pwm_signal_outputs, &g_control_ctx.pwm_outputs},
		{&g_vehicle_health_pub, &topic_vehicle_health, &g_control_ctx.vehicle_health},
		{&g_attitude_estimate_pub, &topic_attitude_estimate,
		 &g_control_ctx.attitude_estimate},
		{&g_attitude_command_pub, &topic_attitude_command,
		 &g_control_ctx.attitude_command},
		{&g_control_loop_metrics_pub, &topic_control_loop_metrics,
		 &g_control_ctx.control_loop_metrics},
	};

	zros_node_init(&g_node, "cubs2_control");

	for (size_t i = 0U; i < ARRAY_SIZE(pubs); i++) {
		int rc = zros_pub_init(pubs[i].pub, &g_node, pubs[i].topic, pubs[i].msg);

		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

static void update_telemetry(struct control_context *ctx, bool auto_mode)
{
	uint64_t now_us = (uint64_t)k_uptime_get() * 1000ULL;
	bool armed = !ctx->manual.valid || ctx->manual.rc.ch6 >= 1500;

	ctx->vehicle_health = (synapse_topic_VehicleHealthData_t){
		.timestamp_us = now_us,
		.flight_mode = auto_mode ? 1U : 0U,
		.link_quality_pct = ctx->manual.valid ? 100U : 0U,
		.flags = armed ? synapse_topic_VehicleHealthFlags_Armed : 0U,
	};

	ctx->attitude_estimate = (synapse_topic_AttitudeEstimateData_t){
		.timestamp_us = now_us,
		.angular_velocity_flu_rad_s = {
			.roll = (float)g_model.eulerRateEstimate_rad_s[0],
			.pitch = (float)g_model.eulerRateEstimate_rad_s[1],
			.yaw = (float)g_model.eulerRateEstimate_rad_s[2],
		},
	};
	csyn_quatf_from_euler((float)g_model.euler_rad[0], (float)g_model.euler_rad[1],
			      (float)g_model.euler_rad[2], &ctx->attitude_estimate.attitude);

	ctx->attitude_command = (synapse_topic_AttitudeCommandData_t){
		.timestamp_us = now_us,
		.thrust = csyn_clampf((float)g_model.throttle, 0.0f, 1.0f),
	};
	csyn_quatf_from_euler((float)g_model.rollCommand, 0.0f, (float)g_model.desiredHeading,
			      &ctx->attitude_command.attitude);

	ctx->control_loop_metrics = (synapse_topic_ControlLoopMetricsData_t){
		.timestamp_us = now_us,
		.period_us = CUBS2_CONTROL_PERIOD_US,
		.latency_us = ctx->main_loop_us,
	};

	(void)zros_pub_update(&g_vehicle_health_pub);
	(void)zros_pub_update(&g_attitude_estimate_pub);
	(void)zros_pub_update(&g_attitude_command_pub);
	(void)zros_pub_update(&g_control_loop_metrics_pub);
}

static void publish_outputs(struct control_context *ctx, bool auto_mode)
{
	csyn_pwm_outputs_from_rc(&ctx->control_rc, &ctx->pwm_outputs, k_uptime_get() * 1000LL);
	(void)zros_pub_update(&g_pwm_outputs_pub);
	update_telemetry(ctx, auto_mode);
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	int rc;

	*ctx = (struct control_context){0};
	EFMI_INIT(FixedWingOuterLoop, &g_model);
	EFMI_RECALIBRATE(FixedWingOuterLoop, &g_model);

	rc = control_pubs_init();
	if (rc != 0) {
		return rc;
	}

	LOG_INF("CUBS2 fixed-wing control starting");

	while (true) {
		uint32_t start_cycles = k_cycle_get_32();
		csyn_rc_channels16_t auto_rc = {0};
		bool auto_mode;

		(void)read_mocap_if_updated(ctx);
		(void)read_manual_if_updated(ctx);

		auto_mode = auto_mode_selected(ctx);
		if (auto_mode && !ctx->previous_auto_mode) {
			EFMI_INIT(FixedWingOuterLoop, &g_model);
			EFMI_RECALIBRATE(FixedWingOuterLoop, &g_model);
		}
		ctx->previous_auto_mode = auto_mode;

		if (auto_mode) {
			if (ctx->mocap.valid) {
				fixed_wing_map_input(&g_model, &ctx->mocap);
				EFMI_STEP(FixedWingOuterLoop, &g_model);
				fixed_wing_map_output(&g_model, &auto_rc);
			} else {
				idle_output(&auto_rc);
			}
			ctx->control_rc = auto_rc;
		} else {
			ctx->control_rc = ctx->manual.rc;
		}

		ctx->main_loop_us = k_cyc_to_us_floor32(k_cycle_get_32() - start_cycles);
		publish_outputs(ctx, auto_mode);
		k_sleep(K_NSEC(CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS));
	}

	return 0;
}
