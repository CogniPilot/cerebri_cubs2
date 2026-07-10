/* SPDX-License-Identifier: Apache-2.0 */

#ifndef CUBS2_NATIVE_SIL_TRANSPORT_H_
#define CUBS2_NATIVE_SIL_TRANSPORT_H_

#include <stdbool.h>
#include <stdint.h>

#include <synapse/control_reader.h>
#include <synapse/state_reader.h>

#define CUBS2_NATIVE_SIL_MAGIC UINT32_C(0x43554253)

struct cubs2_native_sil_shared {
	uint32_t magic;
	uint32_t odometry_sequence;
	uint32_t response_sequence;
	uint32_t terminate;
	synapse_topic_ExternalOdometryData_t odometry;
	synapse_topic_PwmSignalOutputsData_t pwm;
	synapse_topic_AttitudeCommandData_t attitude;
};

_Static_assert(sizeof(struct cubs2_native_sil_shared) == 176,
		       "native SIL shared layout mismatch");

int cubs2_native_sil_transport_start(void);
bool cubs2_native_sil_transport_enabled(void);
int cubs2_native_sil_transport_receive(synapse_topic_ExternalOdometryData_t *odometry);
int cubs2_native_sil_transport_send(const synapse_topic_PwmSignalOutputsData_t *pwm,
				    const synapse_topic_AttitudeCommandData_t *attitude);

#endif
