/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csyn/csyn_zros.h>

#include <zros/private/zros_topic_struct.h>
#include <zros/zros_topic.h>

/* csyn 0.5.0 moved zros topic storage to the vehicle. */
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(manual_control, struct csyn_manual_control);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(odometry, synapse_topic_OdometryData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(pwm_signal_outputs,
                                   synapse_topic_PwmSignalOutputsData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(vehicle_health,
                                   synapse_topic_VehicleHealthData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(attitude_estimate,
                                   synapse_topic_AttitudeEstimateData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(attitude_command,
                                   synapse_topic_AttitudeCommandData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(control_loop_metrics,
                                   synapse_topic_ControlLoopMetricsData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(mission_progress,
                                   synapse_topic_MissionProgressData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(local_position_command,
                                   synapse_topic_LocalPositionCommandData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(trajectory_segment,
                                   synapse_topic_TrajectorySegmentData_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(navigation_target,
                                   synapse_topic_NavigationTargetData_t);
