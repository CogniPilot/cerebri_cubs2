/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csyn/csyn.h>
#include <csyn/csyn_codec.h>
#include <csyn/csyn_zros.h>

#include "FixedWingOuterLoop.h"
#if defined(CONFIG_CUBS2_FASTDYN)
#include "lockstep.h"
#endif
#include "runtime_control.h"

#include <math.h>
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

/* Producer-defined VehicleCommand id broadcasting one local-frame mission item:
 * arg0 = item seq (0-based), arg1 = total items, arg2..4 = ENU east/north/up in
 * meters, arg5 = mission_id. Items cycle so the ground station can rebuild the
 * full plan from any join point.
 */
#define CUBS2_CMD_MISSION_ITEM_LOCAL 32001U
#define CUBS2_MISSION_BROADCAST_PERIOD                                         \
  10U /* control loops per item, 50 Hz / 10 = 5 Hz */
#define CUBS2_MISSION_ID 1U

struct control_context {
  struct csyn_mocap_rigid_body mocap;
#if defined(CONFIG_CUBS2_FASTDYN)
  synapse_topic_OdometryData_t odometry;
  uint64_t next_control_time_us;
#endif
  struct csyn_manual_control manual;
  csyn_rc_channels16_t control_rc;
  synapse_topic_PwmSignalOutputsData_t pwm_outputs;
  synapse_topic_VehicleHealthData_t vehicle_health;
  synapse_topic_AttitudeEstimateData_t attitude_estimate;
  synapse_topic_AttitudeCommandData_t attitude_command;
  synapse_topic_ControlLoopMetricsData_t control_loop_metrics;
  synapse_topic_MissionProgressData_t mission_progress;
  synapse_topic_LocalPositionCommandData_t local_position_command;
  struct csyn_vehicle_command vehicle_command;
  synapse_topic_NavigationTargetData_t navigation_target;
  uint32_t mission_broadcast_countdown;
  uint16_t mission_broadcast_seq;
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
static struct zros_pub g_mission_progress_pub;
static struct zros_pub g_local_position_command_pub;
static struct zros_pub g_vehicle_command_pub;
static struct zros_pub g_navigation_target_pub;

/* Mocap: RawPose subscribed directly on the deployment key; csyn enforces
 * the RawPoseData value contract before the sample reaches the store.
 */
#if defined(CONFIG_CUBS2_FASTDYN)
static bool odometry_valid(const synapse_topic_OdometryData_t *odometry) {
  const uint32_t required = synapse_topic_OdometryFlags_PositionValid |
                            synapse_topic_OdometryFlags_AttitudeValid;
  const uint32_t reject = synapse_topic_OdometryFlags_OutlierRejected |
                          synapse_topic_OdometryFlags_Lost;

  return (odometry->flags & required) == required &&
         (odometry->flags & reject) == 0U &&
         odometry->status != synapse_topic_OdometryStatus_Lost &&
         odometry->status != synapse_topic_OdometryStatus_OutlierRejected;
}

static void read_fastdyn_odometry(struct control_context *ctx) {
  const synapse_topic_OdometryData_t *odometry = &ctx->odometry;

  ctx->mocap = (struct csyn_mocap_rigid_body){
      .x = odometry->pose.position_enu_m.x,
      .y = odometry->pose.position_enu_m.y,
      .z = odometry->pose.position_enu_m.z,
      .qw = odometry->pose.attitude.w,
      .qx = odometry->pose.attitude.x,
      .qy = odometry->pose.attitude.y,
      .qz = odometry->pose.attitude.z,
      .valid = odometry_valid(odometry),
  };
}

static bool fastdyn_control_step_due(struct control_context *ctx) {
  if (ctx->odometry.timestamp_us < ctx->next_control_time_us) {
    return false;
  }

  ctx->next_control_time_us =
      ((ctx->odometry.timestamp_us / CUBS2_CONTROL_PERIOD_US) + 1U) *
      CUBS2_CONTROL_PERIOD_US;
  return true;
}
#else
static struct csyn_topic *g_pose_raw_topic;

static bool read_mocap_if_updated(struct control_context *ctx) {
  synapse_topic_RawPoseData_t pose;
  size_t len = 0U;
  uint32_t generation = csyn_topic_generation(g_pose_raw_topic);
  bool finite = true;

  if (generation == 0U || generation == ctx->mocap_generation) {
    return false;
  }

  if (!csyn_topic_copy(g_pose_raw_topic, &pose, sizeof(pose), &len, NULL) ||
      len != sizeof(pose)) {
    return false;
  }

  const float values[7] = {
      pose.pose.position_enu_m.x, pose.pose.position_enu_m.y,
      pose.pose.position_enu_m.z, pose.pose.attitude.w,
      pose.pose.attitude.x,       pose.pose.attitude.y,
      pose.pose.attitude.z,
  };

  for (size_t i = 0U; i < ARRAY_SIZE(values); i++) {
    finite = finite && isfinite(values[i]);
  }

  ctx->mocap_generation = generation;
  ctx->mocap = (struct csyn_mocap_rigid_body){
      .x = values[0],
      .y = values[1],
      .z = values[2],
      .qw = values[3],
      .qx = values[4],
      .qy = values[5],
      .qz = values[6],
      .valid = finite,
  };
  return true;
}
#endif

static bool read_manual_if_updated(struct control_context *ctx) {
#if defined(CONFIG_CSYN_ZROS_BRIDGE)
  uint32_t generation = csyn_zros_generation(&topic_manual_control);

  if (generation == 0U || generation == ctx->manual_generation) {
    return false;
  }

  if (zros_topic_read(&topic_manual_control, &ctx->manual) != 0) {
    return false;
  }

  ctx->manual_generation = generation;
  return true;
#else
  ARG_UNUSED(ctx);
  return false;
#endif
}

static void fixed_wing_map_input(FixedWingOuterLoopState *model,
                                 const struct csyn_mocap_rigid_body *mocap) {
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
                                  csyn_rc_channels16_t *rc) {
  /* Wire is nose-up = high us on ch1 (2026-07-09 flight): no negations. */
  *rc = (csyn_rc_channels16_t){
#if defined(CONFIG_CUBS2_FASTDYN)
      /* The CMM SportCub plant uses its aerodynamic surface convention. */
      .ch0 = csyn_pwm_from_centered_axis(-(float)model->aileron),
      .ch1 = csyn_pwm_from_centered_axis(-(float)model->elevator),
#else
      .ch0 = csyn_pwm_from_centered_axis((float)model->aileron),
      .ch1 = csyn_pwm_from_centered_axis((float)model->elevator),
#endif
      .ch2 = csyn_pwm_from_throttle_axis((float)model->throttle),
      .ch3 = csyn_pwm_from_centered_axis((float)model->rudder),
      .ch4 = (int32_t)csyn_clampf((float)model->stabilizer, 1000.0f, 2000.0f),
      .ch5 = (int32_t)model->currentWaypoint,
      .ch6 = (int32_t)(1000.0f * (float)model->desiredSpeed),
      .ch7 = (int32_t)(1000.0f * (float)model->rollCommand),
      .ch8 = (int32_t)(1000.0f * (float)model->courseError),
  };
}

static void idle_output(csyn_rc_channels16_t *rc) {
  *rc = (csyn_rc_channels16_t){
      .ch0 = 1500,
      .ch1 = 1500,
      .ch2 = 1000,
      .ch3 = 1500,
      .ch4 = 1000,
  };
}

static bool manual_control_valid(const struct csyn_manual_control *manual,
                                 int32_t *switch_us) {
  const int32_t *channels = csyn_rc_channels_data(&manual->rc);
  bool valid = manual->valid;

  *switch_us = channels[CONFIG_CUBS2_AUTO_SWITCH_CHANNEL];

  for (size_t i = 0U; i < 5U; i++) {
    valid = valid && (channels[i] >= 900) && (channels[i] <= 2100);
  }

  return valid && (*switch_us >= 900) && (*switch_us <= 2100);
}

static bool auto_mode_selected(const struct control_context *ctx) {
#if defined(CONFIG_CUBS2_MANUAL_OVERRIDE)
  int32_t switch_us;

  return !manual_control_valid(&ctx->manual, &switch_us) ||
         (switch_us > CONFIG_CUBS2_AUTO_SWITCH_THRESHOLD_US);
#else
  ARG_UNUSED(ctx);
  return true;
#endif
}

static int control_pubs_init(void) {
  struct {
    struct zros_pub *pub;
    struct zros_topic *topic;
    void *msg;
  } pubs[] = {
      {&g_pwm_outputs_pub, &topic_pwm_signal_outputs,
       &g_control_ctx.pwm_outputs},
      {&g_vehicle_health_pub, &topic_vehicle_health,
       &g_control_ctx.vehicle_health},
      {&g_attitude_estimate_pub, &topic_attitude_estimate,
       &g_control_ctx.attitude_estimate},
      {&g_attitude_command_pub, &topic_attitude_command,
       &g_control_ctx.attitude_command},
      {&g_control_loop_metrics_pub, &topic_control_loop_metrics,
       &g_control_ctx.control_loop_metrics},
      {&g_mission_progress_pub, &topic_mission_progress,
       &g_control_ctx.mission_progress},
      {&g_local_position_command_pub, &topic_local_position_command,
       &g_control_ctx.local_position_command},
      {&g_vehicle_command_pub, &topic_vehicle_command,
       &g_control_ctx.vehicle_command},
      {&g_navigation_target_pub, &topic_navigation_target,
       &g_control_ctx.navigation_target},
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

static void update_mission_telemetry(struct control_context *ctx,
                                     uint64_t now_us, bool auto_mode) {
  /* currentWaypoint is the 1-based active segment; the segment target is
   * route_waypoints row currentWaypoint in C indexing (row 0 is the route
   * start point), so seq n targets route_waypoints[n + 1].
   */
  uint16_t total = (uint16_t)g_model.route_nSegments;
  uint16_t current_seq = 0U;

  if (g_model.currentWaypoint >= 1) {
    current_seq = (uint16_t)g_model.currentWaypoint - 1U;
  }
  if (total > 0U && current_seq >= total) {
    current_seq = total - 1U;
  }

  ctx->mission_progress = (synapse_topic_MissionProgressData_t){
      .timestamp_us = now_us,
      .mission_id = CUBS2_MISSION_ID,
      .current_seq = current_seq,
      .total = total,
      .mission_state = (auto_mode && ctx->mocap.valid)
                           ? synapse_types_MissionState_Active
                           : synapse_types_MissionState_Idle,
  };

  ctx->local_position_command = (synapse_topic_LocalPositionCommandData_t){
      .timestamp_us = now_us,
      .position_enu_m =
          {
              .x = (float)g_model.route_waypoints[current_seq + 1U][0],
              .y = (float)g_model.route_waypoints[current_seq + 1U][1],
              .z = (float)g_model.route_waypoints[current_seq + 1U][2],
          },
      .yaw_rad = (float)g_model.desiredHeading,
      .type_mask = synapse_topic_LocalPositionCommandMask_IgnoreVelocityX |
                   synapse_topic_LocalPositionCommandMask_IgnoreVelocityY |
                   synapse_topic_LocalPositionCommandMask_IgnoreVelocityZ |
                   synapse_topic_LocalPositionCommandMask_IgnoreAccelerationX |
                   synapse_topic_LocalPositionCommandMask_IgnoreAccelerationY |
                   synapse_topic_LocalPositionCommandMask_IgnoreAccelerationZ,
      .coordinate_frame = synapse_types_LocalFrame_LocalEnu,
  };

  (void)zros_pub_update(&g_mission_progress_pub);
  (void)zros_pub_update(&g_local_position_command_pub);

  if (ctx->mission_broadcast_countdown > 0U) {
    ctx->mission_broadcast_countdown--;
    return;
  }
  ctx->mission_broadcast_countdown = CUBS2_MISSION_BROADCAST_PERIOD - 1U;

  if (total == 0U) {
    return;
  }
  if (ctx->mission_broadcast_seq >= total) {
    ctx->mission_broadcast_seq = 0U;
  }

  ctx->vehicle_command = (struct csyn_vehicle_command){
      .timestamp_us = now_us,
      .arg0 = (float)ctx->mission_broadcast_seq,
      .arg1 = (float)total,
      .arg2 =
          (float)g_model.route_waypoints[ctx->mission_broadcast_seq + 1U][0],
      .arg3 =
          (float)g_model.route_waypoints[ctx->mission_broadcast_seq + 1U][1],
      .arg4 =
          (float)g_model.route_waypoints[ctx->mission_broadcast_seq + 1U][2],
      .arg5 = (float)CUBS2_MISSION_ID,
      .command_id = CUBS2_CMD_MISSION_ITEM_LOCAL,
  };
  (void)zros_pub_update(&g_vehicle_command_pub);
  ctx->mission_broadcast_seq++;
}

static int16_t cdeg_from_rad(double angle_rad) {
  /* REP-0103 angle in centidegrees (18000/pi), clamped to the schema range. */
  return (int16_t)csyn_clampf((float)(angle_rad * 5729.5779513082325),
                              -18000.0f, 18000.0f);
}

static void update_navigation_target(struct control_context *ctx,
                                     uint64_t now_us, bool auto_mode) {
  double target_east = 0.0;
  double target_north = 0.0;
  float distance = 0.0f;
  int16_t target_yaw = 0;
  int32_t wp = g_model.currentWaypoint;

  if (wp >= 1 && wp <= g_model.route_nSegments) {
    /* segment target: route_waypoints row wp (row 0 = route start) */
    target_east = g_model.route_waypoints[wp][0];
    target_north = g_model.route_waypoints[wp][1];
  }
  if (ctx->mocap.valid) {
    float d_east = (float)target_east - ctx->mocap.x;
    float d_north = (float)target_north - ctx->mocap.y;

    distance = sqrtf(d_east * d_east + d_north * d_north);
    target_yaw = cdeg_from_rad(atan2f(d_north, d_east));
  }

  ctx->navigation_target = (synapse_topic_NavigationTargetData_t){
      .timestamp_us = now_us,
      .altitude_error_m = (float)g_model.guidance_altitudeError,
      .airspeed_error_m_s = (float)(g_model.guidance_setpoints_speed -
                                    g_model.estimator_estimate_speed),
      .xtrack_error_m = (float)g_model.guidance_crossTrackError,
      /* Manual flight publishes the integral-free preview: what auto would fly.
       */
      .desired_roll_cdeg = cdeg_from_rad(
          auto_mode ? g_model.rollCommand : g_model.rollCommandPreview),
      .desired_pitch_cdeg = cdeg_from_rad(
          auto_mode ? g_model.tecs_pitchCommand : g_model.pitchCommandPreview),
      .desired_yaw_cdeg = cdeg_from_rad(g_model.desiredHeading),
      .target_yaw_cdeg = target_yaw,
      .distance_to_waypoint_m = (uint16_t)csyn_clampf(distance, 0.0f, 65535.0f),
  };
}

static void update_telemetry(struct control_context *ctx, bool auto_mode,
                             uint64_t now_us) {
  bool armed = !ctx->manual.valid || ctx->manual.rc.ch6 >= 1500;

  ctx->vehicle_health = (synapse_topic_VehicleHealthData_t){
      .timestamp_us = now_us,
      .flight_mode = auto_mode ? 1U : 0U,
      .link_quality_pct = ctx->manual.valid ? 100U : 0U,
      .flags = armed ? synapse_topic_VehicleHealthFlags_Armed : 0U,
  };

  ctx->attitude_estimate = (synapse_topic_AttitudeEstimateData_t){
      .timestamp_us = now_us,
      .angular_velocity_flu_rad_s =
          {
              .roll = (float)g_model.eulerRateEstimate_rad_s[0],
              .pitch = (float)g_model.eulerRateEstimate_rad_s[1],
              .yaw = (float)g_model.eulerRateEstimate_rad_s[2],
          },
  };
  csyn_quatf_from_euler(
      (float)g_model.euler_rad[0], (float)g_model.euler_rad[1],
      (float)g_model.euler_rad[2], &ctx->attitude_estimate.attitude);

  ctx->attitude_command = (synapse_topic_AttitudeCommandData_t){
      .timestamp_us = now_us,
      .thrust = csyn_clampf((float)g_model.throttle, 0.0f, 1.0f),
  };
  csyn_quatf_from_euler((float)g_model.rollCommand, 0.0f,
                        (float)g_model.desiredHeading,
                        &ctx->attitude_command.attitude);

  ctx->control_loop_metrics = (synapse_topic_ControlLoopMetricsData_t){
      .timestamp_us = now_us,
      .period_us = CUBS2_CONTROL_PERIOD_US,
      .latency_us = ctx->main_loop_us,
  };

  update_navigation_target(ctx, now_us, auto_mode);

  (void)zros_pub_update(&g_vehicle_health_pub);
  (void)zros_pub_update(&g_attitude_estimate_pub);
  (void)zros_pub_update(&g_attitude_command_pub);
  (void)zros_pub_update(&g_control_loop_metrics_pub);
  (void)zros_pub_update(&g_navigation_target_pub);

  update_mission_telemetry(ctx, now_us, auto_mode);
}

static void publish_outputs(struct control_context *ctx, bool auto_mode) {
#if defined(CONFIG_CUBS2_FASTDYN)
  uint64_t now_us = ctx->odometry.timestamp_us;
#else
  uint64_t now_us = (uint64_t)k_uptime_get() * 1000ULL;
#endif

  csyn_pwm_outputs_from_rc(&ctx->control_rc, &ctx->pwm_outputs,
                           (int64_t)now_us);
  (void)zros_pub_update(&g_pwm_outputs_pub);
  update_telemetry(ctx, auto_mode, now_us);
}

int main(void) {
  struct control_context *const ctx = &g_control_ctx;
  int rc;

  *ctx = (struct control_context){0};
  EFMI_INIT(FixedWingOuterLoop, &g_model);
  EFMI_RECALIBRATE(FixedWingOuterLoop, &g_model);
  /* Mocap delivers pose only: the estimator derives velocity and rates. */
  g_model.estimator_useMeasuredRates = 0.0;
  if (!cubs2_runtime_control_init(&g_model)) {
    LOG_ERR("failed to register runtime configuration services");
    return -1;
  }
#if defined(CONFIG_CUBS2_FASTDYN)
  rc = cubs2_lockstep_start();
  if (rc != 0) {
    return rc;
  }
#else
  g_pose_raw_topic = csyn_topic_find("pose_raw");
  if (g_pose_raw_topic == NULL) {
    LOG_ERR("pose_raw topic missing");
    return -1;
  }
#endif

  rc = control_pubs_init();
  if (rc != 0) {
    return rc;
  }

  LOG_INF("CUBS2 fixed-wing control starting");

#if !defined(CONFIG_CUBS2_FASTDYN)
  /* Absolute-period scheduling: relative sleep adds the work time to every
   * cycle, stretching the model's fixed dt and scaling all derivatives. */
  int64_t next_cycle_ticks = k_uptime_ticks();
  const uint32_t period_ticks =
      k_ns_to_ticks_ceil32(CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS);
#endif

  while (true) {
    uint32_t start_cycles = k_cycle_get_32();
    csyn_rc_channels16_t auto_rc = {0};
    bool auto_mode;
    bool step_control = true;

#if defined(CONFIG_CUBS2_FASTDYN)
    if (cubs2_lockstep_receive(&ctx->odometry) != 0) {
      return -EIO;
    }
    read_fastdyn_odometry(ctx);
    step_control = fastdyn_control_step_due(ctx);
#else
    (void)read_mocap_if_updated(ctx);
#endif
    (void)read_manual_if_updated(ctx);

    auto_mode = auto_mode_selected(ctx);
    ctx->previous_auto_mode = auto_mode;
    cubs2_runtime_control_apply(&g_model, !ctx->manual.valid ||
                                              ctx->manual.rc.ch6 >= 1500);

    /* The estimator stays warm while manual. FastDyn receives plant samples
     * at 100 Hz but preserves the model's 50 Hz fixed step. */
    if (step_control && ctx->mocap.valid) {
      fixed_wing_map_input(&g_model, &ctx->mocap);
      g_model.engaged = auto_mode ? 1.0 : 0.0;
      EFMI_STEP(FixedWingOuterLoop, &g_model);
    }

    if (auto_mode) {
      if (ctx->mocap.valid) {
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
#if defined(CONFIG_CUBS2_FASTDYN)
    if (cubs2_lockstep_send(&ctx->pwm_outputs, &ctx->attitude_command) != 0) {
      return -EIO;
    }
#else
    next_cycle_ticks += period_ticks;
    if (k_uptime_ticks() > next_cycle_ticks + 2 * (int64_t)period_ticks) {
      next_cycle_ticks = k_uptime_ticks(); /* resync after a stall */
    }
    k_sleep(K_TIMEOUT_ABS_TICKS(next_cycle_ticks));
#endif
  }

  return 0;
}
