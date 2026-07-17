#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <csyn/csyn.h>
#include <flatcc/flatcc_builder.h>
#include <synapse/trajectory_builder.h>
#include <synapse/transfer_builder.h>
#include <synapse/transfer_reader.h>
#include <synapse/transfer_verifier.h>
#include <zephyr/sys/util.h>

#include "Vehicles_Cubs2_OuterLoop.h"
#include "runtime_control.h"

#define REQUEST_CAPACITY 2048U
#define SERVICE_COUNT 3U

struct registered_service {
  const char *key;
  csyn_query_handler_t handler;
  void *user;
};

static struct registered_service services[SERVICE_COUNT];
static size_t service_count;

bool csyn_zenoh_register_queryable(const char *key,
                                   csyn_query_handler_t handler, void *user) {
  assert(service_count < SERVICE_COUNT);
  services[service_count++] = (struct registered_service){
      .key = key,
      .handler = handler,
      .user = user,
  };
  return true;
}

static struct registered_service *find_service(const char *key) {
  for (size_t i = 0U; i < service_count; i++) {
    if (strcmp(services[i].key, key) == 0) {
      return &services[i];
    }
  }
  assert(false);
  return NULL;
}

static size_t finish_request(flatcc_builder_t *builder, uint8_t *request) {
  size_t request_len;
  void *data = flatcc_builder_get_direct_buffer(builder, &request_len);

  assert(data != NULL);
  assert(request_len <= REQUEST_CAPACITY);
  memcpy(request, data, request_len);
  flatcc_builder_clear(builder);
  return request_len;
}

static size_t build_param_set(uint8_t *request, const char *name,
                              double value) {
  flatcc_builder_t builder;

  assert(flatcc_builder_init(&builder) == 0);
  assert(synapse_cmd_ParamSetRequest_start_as_root(&builder) == 0);
  assert(synapse_cmd_ParamSetRequest_value_start(&builder) == 0);
  assert(synapse_cmd_ParamValue_name_create_str(&builder, name) == 0);
  assert(synapse_cmd_ParamValue_kind_add(&builder,
                                         synapse_cmd_ParamKind_Float) == 0);
  assert(synapse_cmd_ParamValue_float_value_add(&builder, value) == 0);
  assert(synapse_cmd_ParamSetRequest_value_end(&builder) == 0);
  assert(synapse_cmd_ParamSetRequest_end_as_root(&builder) != 0);
  return finish_request(&builder, request);
}

static size_t build_param_get(uint8_t *request, const char *name) {
  flatcc_builder_t builder;

  assert(flatcc_builder_init(&builder) == 0);
  assert(synapse_cmd_ParamGetRequest_start_as_root(&builder) == 0);
  assert(synapse_cmd_ParamGetRequest_name_create_str(&builder, name) == 0);
  assert(synapse_cmd_ParamGetRequest_limit_add(&builder, 1U) == 0);
  assert(synapse_cmd_ParamGetRequest_end_as_root(&builder) != 0);
  return finish_request(&builder, request);
}

static synapse_topic_TrajectorySegmentData_t make_segment(uint32_t sequence,
                                                          float base) {
  return (synapse_topic_TrajectorySegmentData_t){
      .p0_enu_m = {.x = base, .y = base + 1.0f, .z = base + 2.0f},
      .p1_enu_m = {.x = base + 3.0f, .y = base + 4.0f, .z = base + 5.0f},
      .trajectory_id = 1U,
      .segment_seq = sequence,
      .frame = synapse_types_LocalFrame_LocalEnu,
  };
}

static size_t
build_trajectory(uint8_t *request, uint32_t expected_version,
                 const synapse_topic_TrajectorySegmentData_t *segments,
                 size_t count) {
  flatcc_builder_t builder;

  assert(flatcc_builder_init(&builder) == 0);
  assert(synapse_cmd_TrajectorySetRequest_start_as_root(&builder) == 0);
  assert(synapse_cmd_TrajectorySetRequest_trajectory_id_add(&builder, 1U) == 0);
  assert(synapse_cmd_TrajectorySetRequest_expected_plan_version_add(
             &builder, expected_version) == 0);
  assert(synapse_cmd_TrajectorySetRequest_total_add(&builder, count) == 0);
  assert(synapse_cmd_TrajectorySetRequest_segments_create(&builder, segments,
                                                          count) == 0);
  assert(synapse_cmd_TrajectorySetRequest_end_as_root(&builder) != 0);
  return finish_request(&builder, request);
}

static size_t query(const char *key, const uint8_t *request, size_t request_len,
                    uint8_t *reply) {
  struct registered_service *service = find_service(key);
  size_t reply_len = 0U;

  assert(service->handler(request, request_len, reply, REQUEST_CAPACITY,
                          &reply_len, service->user));
  assert(reply_len > 0U);
  return reply_len;
}

static synapse_cmd_TrajectorySetReply_table_t
trajectory_reply(const uint8_t *reply, size_t reply_len) {
  assert(synapse_cmd_TrajectorySetReply_verify_as_root(reply, reply_len) == 0);
  return synapse_cmd_TrajectorySetReply_as_root(reply);
}

static synapse_types_CommandResultCode_enum_t
param_set_result(const uint8_t *reply, size_t reply_len) {
  synapse_cmd_ParamSetReply_table_t result;

  assert(synapse_cmd_ParamSetReply_verify_as_root(reply, reply_len) == 0);
  result = synapse_cmd_ParamSetReply_as_root(reply);
  return synapse_cmd_ParamSetReply_result(result);
}

static double param_get_value(const uint8_t *reply, size_t reply_len) {
  synapse_cmd_ParamGetReply_table_t result;
  synapse_cmd_ParamValue_vec_t values;

  assert(synapse_cmd_ParamGetReply_verify_as_root(reply, reply_len) == 0);
  result = synapse_cmd_ParamGetReply_as_root(reply);
  assert(synapse_cmd_ParamGetReply_result(result) ==
         synapse_types_CommandResultCode_Accepted);
  values = synapse_cmd_ParamGetReply_values(result);
  assert(values != NULL);
  assert(synapse_cmd_ParamValue_vec_len(values) == 1U);
  return synapse_cmd_ParamValue_float_value(
      synapse_cmd_ParamValue_vec_at(values, 0U));
}

static void test_malformed_requests(void) {
  const uint8_t malformed[8] = {0xfc, 0xff, 0xff, 0x7f, 0, 0, 0, 0};
  const char *keys[] = {"cmd/param_set", "cmd/param_get", "cmd/trajectory_set"};
  uint8_t reply[REQUEST_CAPACITY];

  for (size_t i = 0U; i < ARRAY_SIZE(keys); i++) {
    (void)query(keys[i], malformed, sizeof(malformed), reply);
  }
}

static void test_parameter_restore(Vehicles_Cubs2_OuterLoopState *model) {
  uint8_t request[REQUEST_CAPACITY];
  uint8_t reply[REQUEST_CAPACITY];
  size_t request_len = build_param_set(request, "route.cruiseSpeed", 7.25);

  (void)query("cmd/param_set", request, request_len, reply);
  cubs2_runtime_control_apply(model, false);
  assert(model->route_cruiseSpeed == 7.25);
  assert(model->guidance_route_cruiseSpeed == 7.25);

  Vehicles_Cubs2_OuterLoop_startup(model);
  Vehicles_Cubs2_OuterLoop_recalibrate(model);
  assert(model->route_cruiseSpeed != 7.25);
  cubs2_runtime_control_restore(model);
  assert(model->route_cruiseSpeed == 7.25);
  assert(model->guidance_route_cruiseSpeed == 7.25);
}

static void test_trusted_parameter_policy(Vehicles_Cubs2_OuterLoopState *model) {
  uint8_t request[REQUEST_CAPACITY];
  uint8_t reply[REQUEST_CAPACITY];
  size_t request_len = build_param_set(request, "route.cruiseSpeed", 12.5);
  size_t reply_len = query("cmd/param_set", request, request_len, reply);

  assert(param_set_result(reply, reply_len) ==
         synapse_types_CommandResultCode_Accepted);
  cubs2_runtime_control_apply(model, true);
  assert(model->route_cruiseSpeed == 12.5);

  request_len = build_param_set(request, "route.cruiseSpeed", NAN);
  reply_len = query("cmd/param_set", request, request_len, reply);
  assert(param_set_result(reply, reply_len) ==
         synapse_types_CommandResultCode_Denied);
  cubs2_runtime_control_apply(model, true);
  assert(model->route_cruiseSpeed == 12.5);
}

struct refresh_stress_context {
  size_t iterations;
};

static void *refresh_stress_thread(void *arg) {
  const struct refresh_stress_context *ctx = arg;

  for (size_t i = 0U; i < ctx->iterations; i++) {
    uint8_t request[REQUEST_CAPACITY];
    uint8_t reply[REQUEST_CAPACITY];
    size_t request_len = build_param_get(request, "velocity.setpoint");
    size_t reply_len = query("cmd/param_get", request, request_len, reply);

    assert(isfinite(param_get_value(reply, reply_len)));
  }
  return NULL;
}

static void test_parameter_refresh_concurrency(Vehicles_Cubs2_OuterLoopState *model) {
  const struct refresh_stress_context ctx = {.iterations = 2000U};
  pthread_t thread;

  assert(pthread_create(&thread, NULL, refresh_stress_thread, (void *)&ctx) ==
         0);
  for (size_t i = 0U; i < ctx.iterations; i++) {
    cubs2_runtime_control_apply(model, true);
  }
  assert(pthread_join(thread, NULL) == 0);
}

static uint32_t
submit_trajectory(Vehicles_Cubs2_OuterLoopState *model, uint32_t expected_version,
                  synapse_topic_TrajectorySegmentData_t *segments, size_t count,
                  synapse_types_CommandResultCode_enum_t expected_result,
                  bool apply) {
  uint8_t request[REQUEST_CAPACITY];
  uint8_t reply[REQUEST_CAPACITY];
  size_t request_len =
      build_trajectory(request, expected_version, segments, count);
  size_t reply_len = query("cmd/trajectory_set", request, request_len, reply);
  synapse_cmd_TrajectorySetReply_table_t result =
      trajectory_reply(reply, reply_len);

  assert(synapse_cmd_TrajectorySetReply_result(result) == expected_result);
  if (apply) {
    cubs2_runtime_control_apply(model, false);
  }
  return synapse_cmd_TrajectorySetReply_plan_version(result);
}

static void test_trajectory_transactions(Vehicles_Cubs2_OuterLoopState *model) {
  synapse_topic_TrajectorySegmentData_t long_route[5];
  synapse_topic_TrajectorySegmentData_t short_route[2];
  synapse_topic_TrajectorySegmentData_t staged_route[2];
  synapse_topic_TrajectorySegmentData_t invalid_route[2];
  uint32_t version;

  for (size_t i = 0U; i < ARRAY_SIZE(long_route); i++) {
    long_route[i] = make_segment(i, (float)(10U * i));
  }
  version = submit_trajectory(model, 1U, long_route, ARRAY_SIZE(long_route),
                              synapse_types_CommandResultCode_Accepted, true);
  assert(version == 2U);
  assert(model->route_nSegments == 5);

  model->currentWaypoint = 5;
  model->guidance_currentWaypoint = 5;
  model->guidance_activeWaypoint = 5;
  model->previous_guidance_currentWaypoint = 5;
  for (size_t i = 0U; i < ARRAY_SIZE(short_route); i++) {
    short_route[i] = make_segment(i, 100.0f + (float)(10U * i));
  }
  version =
      submit_trajectory(model, version, short_route, ARRAY_SIZE(short_route),
                        synapse_types_CommandResultCode_Accepted, true);
  assert(version == 3U);
  assert(model->route_nSegments == 2);
  assert(model->currentWaypoint == 2);
  assert(model->guidance_currentWaypoint == 2);
  assert(model->guidance_activeWaypoint == 2);
  assert(model->previous_guidance_currentWaypoint == 2);
  assert(model->route_waypoints[6][0] == short_route[1].p1_enu_m.x);

  assert(submit_trajectory(model, 2U, long_route, ARRAY_SIZE(long_route),
                           synapse_types_CommandResultCode_TemporarilyRejected,
                           false) == version);

  for (size_t i = 0U; i < ARRAY_SIZE(staged_route); i++) {
    staged_route[i] = make_segment(i, 200.0f + (float)(10U * i));
    invalid_route[i] = make_segment(i, 300.0f + (float)(10U * i));
  }
  version =
      submit_trajectory(model, version, staged_route, ARRAY_SIZE(staged_route),
                        synapse_types_CommandResultCode_Accepted, false);
  assert(version == 4U);
  invalid_route[1].p1_enu_m.x = NAN;
  assert(submit_trajectory(
             model, version, invalid_route, ARRAY_SIZE(invalid_route),
             synapse_types_CommandResultCode_Denied, false) == version);

  cubs2_runtime_control_apply(model, false);
  assert(model->route_waypoints[0][0] == staged_route[0].p0_enu_m.x);
  assert(model->route_waypoints[2][0] == staged_route[1].p1_enu_m.x);

  Vehicles_Cubs2_OuterLoop_startup(model);
  Vehicles_Cubs2_OuterLoop_recalibrate(model);
  cubs2_runtime_control_restore(model);
  assert(model->route_nSegments == 2);
  assert(model->route_waypoints[0][0] == staged_route[0].p0_enu_m.x);
  assert(model->route_waypoints[2][0] == staged_route[1].p1_enu_m.x);
}

int main(void) {
  Vehicles_Cubs2_OuterLoopState model;

  Vehicles_Cubs2_OuterLoop_startup(&model);
  Vehicles_Cubs2_OuterLoop_recalibrate(&model);
  assert(cubs2_runtime_control_init(&model));
  assert(service_count == SERVICE_COUNT);

  test_malformed_requests();
  test_parameter_restore(&model);
  test_trusted_parameter_policy(&model);
  test_parameter_refresh_concurrency(&model);
  test_trajectory_transactions(&model);
  puts("runtime_control tests passed");
  return EXIT_SUCCESS;
}
