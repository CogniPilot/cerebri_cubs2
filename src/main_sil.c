/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csyn/csyn.h>
#include <csyn/csyn_codec.h>
#include <csyn/csyn_zros.h>

#include "FixedWingOuterLoop.h"
#include "lockstep.h"
#include "runtime_control.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

LOG_MODULE_REGISTER(cubs2, LOG_LEVEL_INF);

#define CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS 20000000
#define CUBS2_CONTROL_PERIOD_US (CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS / 1000U)

struct control_context {
  synapse_topic_OdometryData_t odometry;
  struct csyn_manual_control manual;
  csyn_rc_channels16_t control_rc;
  synapse_topic_PwmSignalOutputsData_t pwm_outputs;
  synapse_topic_VehicleHealthData_t vehicle_health;
  synapse_topic_AttitudeEstimateData_t attitude_estimate;
  synapse_topic_AttitudeCommandData_t attitude_command;
  synapse_topic_ControlLoopMetricsData_t control_loop_metrics;
  uint32_t main_loop_us;
  uint64_t lockstep_time_us;
  uint64_t next_control_time_us;
  bool lockstep_time_valid;
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
static struct zros_sub g_odometry_sub;
static struct zros_sub g_manual_sub;

static bool read_odometry_if_updated(struct control_context *ctx) {
  if (zros_sub_update(&g_odometry_sub) != 0) {
    return false;
  }

  ctx->lockstep_time_us = ctx->odometry.timestamp_us;
  ctx->lockstep_time_valid = true;
  return true;
}

static bool read_manual_if_updated(struct control_context *ctx) {
  return zros_sub_update(&g_manual_sub) == 0;
}

static bool odometry_valid(const synapse_topic_OdometryData_t *odom) {
  const uint32_t required = synapse_topic_OdometryFlags_PositionValid |
                            synapse_topic_OdometryFlags_AttitudeValid |
                            synapse_topic_OdometryFlags_LinearVelocityValid |
                            synapse_topic_OdometryFlags_AngularVelocityValid;
  const uint32_t reject = synapse_topic_OdometryFlags_OutlierRejected |
                          synapse_topic_OdometryFlags_Lost;
  uint32_t flags = odom->flags;

  return (flags & required) == required && (flags & reject) == 0U &&
         odom->status != synapse_topic_OdometryStatus_Lost &&
         odom->status != synapse_topic_OdometryStatus_OutlierRejected;
}

static void fixed_wing_map_input(FixedWingOuterLoopState *model,
                                 const synapse_topic_OdometryData_t *odom) {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  /* FixedWingOuterLoop consumes Euler [roll, pitch, yaw] in radians. */
  csyn_euler_from_quatf(&odom->pose.attitude, &roll, &pitch, &yaw);

  model->position_m[0] = odom->pose.position_enu_m.x;
  model->position_m[1] = odom->pose.position_enu_m.y;
  model->position_m[2] = odom->pose.position_enu_m.z;
  model->euler_rad[0] = roll;
  model->euler_rad[1] = pitch;
  model->euler_rad[2] = yaw;
  model->velocity_m_s[0] = odom->twist.linear_velocity_enu_m_s.x;
  model->velocity_m_s[1] = odom->twist.linear_velocity_enu_m_s.y;
  model->velocity_m_s[2] = odom->twist.linear_velocity_enu_m_s.z;
  model->eulerRate_rad_s[0] = odom->twist.angular_velocity_flu_rad_s.roll;
  model->eulerRate_rad_s[1] = odom->twist.angular_velocity_flu_rad_s.pitch;
  model->eulerRate_rad_s[2] = odom->twist.angular_velocity_flu_rad_s.yaw;
}

static void fixed_wing_map_output(const FixedWingOuterLoopState *model,
                                  csyn_rc_channels16_t *rc) {
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
  int rc;
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
  };

  zros_node_init(&g_node, "cubs2_control");

  for (size_t i = 0U; i < ARRAY_SIZE(pubs); i++) {
    rc = zros_pub_init(pubs[i].pub, &g_node, pubs[i].topic, pubs[i].msg);

    if (rc != 0) {
      return rc;
    }
  }

  rc = zros_sub_init(&g_odometry_sub, &g_node, &topic_odometry,
                     &g_control_ctx.odometry, 0.0);
  if (rc != 0) {
    return rc;
  }
  rc = zros_sub_init(&g_manual_sub, &g_node, &topic_manual_control,
                     &g_control_ctx.manual, 0.0);
  if (rc != 0) {
    return rc;
  }

  return 0;
}
static uint64_t control_timestamp_us(const struct control_context *ctx) {
#if defined(CONFIG_BOARD_NATIVE_SIM) || defined(CONFIG_CUBS2_FASTDYN)
  if (ctx->lockstep_time_valid) {
    return ctx->lockstep_time_us;
  }

  return 0ULL;
#endif

  return (uint64_t)k_uptime_get() * 1000ULL;
}

static bool control_step_due(struct control_context *ctx) {
#if defined(CONFIG_BOARD_NATIVE_SIM) || defined(CONFIG_CUBS2_FASTDYN)
  if (ctx->lockstep_time_us < ctx->next_control_time_us) {
    return false;
  }

  ctx->next_control_time_us =
      ((ctx->lockstep_time_us / CUBS2_CONTROL_PERIOD_US) + 1U) *
      CUBS2_CONTROL_PERIOD_US;
#else
  ARG_UNUSED(ctx);
#endif

  return true;
}

static void update_telemetry(struct control_context *ctx, bool auto_mode,
                             uint64_t timestamp_us) {
  bool armed = !ctx->manual.valid || ctx->manual.rc.ch6 >= 1500;

  ctx->vehicle_health = (synapse_topic_VehicleHealthData_t){
      .timestamp_us = timestamp_us,
      .flight_mode = auto_mode ? 1U : 0U,
      .link_quality_pct = ctx->manual.valid ? 100U : 0U,
      .flags = armed ? synapse_topic_VehicleHealthFlags_Armed : 0U,
  };

  ctx->attitude_estimate = (synapse_topic_AttitudeEstimateData_t){
      .timestamp_us = timestamp_us,
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
      .timestamp_us = timestamp_us,
      .thrust = csyn_clampf((float)g_model.throttle, 0.0f, 1.0f),
  };
  csyn_quatf_from_euler((float)g_model.rollCommand, 0.0f,
                        (float)g_model.desiredHeading,
                        &ctx->attitude_command.attitude);

  ctx->control_loop_metrics = (synapse_topic_ControlLoopMetricsData_t){
      .timestamp_us = timestamp_us,
      .period_us = CUBS2_CONTROL_PERIOD_US,
      .latency_us = ctx->main_loop_us,
  };

  (void)zros_pub_update(&g_vehicle_health_pub);
  (void)zros_pub_update(&g_attitude_estimate_pub);
  (void)zros_pub_update(&g_attitude_command_pub);
  (void)zros_pub_update(&g_control_loop_metrics_pub);
}

static void publish_outputs(struct control_context *ctx, bool auto_mode) {
  uint64_t timestamp_us = control_timestamp_us(ctx);

  csyn_pwm_outputs_from_rc(&ctx->control_rc, &ctx->pwm_outputs,
                           (int64_t)timestamp_us);
  (void)zros_pub_update(&g_pwm_outputs_pub);
  update_telemetry(ctx, auto_mode, timestamp_us);
}

int main(void) {
  struct control_context *const ctx = &g_control_ctx;
  int rc;

  *ctx = (struct control_context){0};
  EFMI_INIT(FixedWingOuterLoop, &g_model);
  EFMI_RECALIBRATE(FixedWingOuterLoop, &g_model);
  if (!cubs2_runtime_control_init(&g_model)) {
    LOG_ERR("failed to register runtime configuration services");
    return -1;
  }

  rc = control_pubs_init();
  if (rc != 0) {
    return rc;
  }
  rc = cubs2_lockstep_start();
  if (rc != 0) {
    return rc;
  }

  LOG_INF("CUBS2 fixed-wing control starting");

  while (true) {
    uint32_t start_cycles = k_cycle_get_32();
    csyn_rc_channels16_t auto_rc = {0};
    bool odometry_updated;
    bool auto_mode;
    bool step_control;

#if defined(CONFIG_CUBS2_LOCKSTEP)
    if (cubs2_lockstep_enabled()) {
      if (cubs2_lockstep_receive(&ctx->odometry) != 0) {
        return -EIO;
      }
      ctx->lockstep_time_us = ctx->odometry.timestamp_us;
      ctx->lockstep_time_valid = true;
      odometry_updated = true;
    } else {
      if (zros_sub_wait(&g_odometry_sub, K_FOREVER) != 0) {
        continue;
      }
      odometry_updated = read_odometry_if_updated(ctx);
    }
    if (!odometry_updated) {
      continue;
    }
#else
    odometry_updated = read_odometry_if_updated(ctx);
#endif
    (void)read_manual_if_updated(ctx);

    auto_mode = auto_mode_selected(ctx);
    if (auto_mode && !ctx->previous_auto_mode) {
      EFMI_INIT(FixedWingOuterLoop, &g_model);
      EFMI_RECALIBRATE(FixedWingOuterLoop, &g_model);
      cubs2_runtime_control_restore(&g_model);
    }
    ctx->previous_auto_mode = auto_mode;
    cubs2_runtime_control_apply(&g_model, !ctx->manual.valid ||
                                              ctx->manual.rc.ch6 >= 1500);
    step_control = control_step_due(ctx);

    if (auto_mode) {
      if (step_control && odometry_valid(&ctx->odometry)) {
        fixed_wing_map_input(&g_model, &ctx->odometry);
        g_model.engaged = 1.0;
        EFMI_STEP(FixedWingOuterLoop, &g_model);
        fixed_wing_map_output(&g_model, &auto_rc);
        ctx->control_rc = auto_rc;
      } else if (!odometry_valid(&ctx->odometry)) {
        idle_output(&auto_rc);
        ctx->control_rc = auto_rc;
      }
    } else {
      ctx->control_rc = ctx->manual.rc;
    }

    ctx->main_loop_us = k_cyc_to_us_floor32(k_cycle_get_32() - start_cycles);
    publish_outputs(ctx, auto_mode);
#if defined(CONFIG_CUBS2_LOCKSTEP)
    if (cubs2_lockstep_enabled()) {
      if (cubs2_lockstep_send(&ctx->pwm_outputs, &ctx->attitude_command) != 0) {
        return -EIO;
      }
    } else {
      k_yield();
    }
#else
    k_sleep(K_NSEC(CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS));
#endif
  }

  return 0;
}
