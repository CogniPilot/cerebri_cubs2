/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csyn/csyn.h>

#include <synapse/all_reader.h>

/* csyn carries only what the application declares; each key's final segment
 * resolves through the synapse_fbs 0.7.0 catalog. */
CSYN_TOPIC_DEFINE(manual, "manual", CSYN_DIR_RX,
                  sizeof(synapse_topic_ManualControlData_t));
#if defined(CONFIG_CUBS2_REALTIME)
CSYN_TOPIC_DEFINE(pose_raw, "qualisys/cub1/pose_raw", CSYN_DIR_RX,
                  sizeof(synapse_topic_RawPoseData_t));
#else
CSYN_TOPIC_DEFINE(odom, "odom", CSYN_DIR_RX,
                  sizeof(synapse_topic_OdometryData_t));
#endif
CSYN_TOPIC_DEFINE(pwm, "pwm", CSYN_DIR_TX,
                  sizeof(synapse_topic_PwmSignalOutputsData_t));
CSYN_TOPIC_DEFINE(health, "health", CSYN_DIR_TX,
                  sizeof(synapse_topic_VehicleHealthData_t));
CSYN_TOPIC_DEFINE(att, "att", CSYN_DIR_TX,
                  sizeof(synapse_topic_AttitudeEstimateData_t));
CSYN_TOPIC_DEFINE(att_sp, "att_sp", CSYN_DIR_TX,
                  sizeof(synapse_topic_AttitudeCommandData_t));
CSYN_TOPIC_DEFINE(loop, "loop", CSYN_DIR_TX,
                  sizeof(synapse_topic_ControlLoopMetricsData_t));
#if defined(CONFIG_CUBS2_REALTIME)
CSYN_TOPIC_DEFINE(mission, "mission", CSYN_DIR_TX,
                  sizeof(synapse_topic_MissionProgressData_t));
CSYN_TOPIC_DEFINE(pos_sp, "pos_sp", CSYN_DIR_TX,
                  sizeof(synapse_topic_LocalPositionCommandData_t));
CSYN_TOPIC_DEFINE(nav, "nav", CSYN_DIR_TX,
                  sizeof(synapse_topic_NavigationTargetData_t));
#endif
