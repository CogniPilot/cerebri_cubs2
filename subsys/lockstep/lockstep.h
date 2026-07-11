/* SPDX-License-Identifier: Apache-2.0 */

#ifndef CUBS2_LOCKSTEP_H_
#define CUBS2_LOCKSTEP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <synapse/control_reader.h>
#include <synapse/state_reader.h>

#define CUBS2_LOCKSTEP_MAGIC UINT32_C(0x43554253)

/* The payload storage and exported FastDyn symbol are vehicle-owned. The
 * shared cerebri_lockstep module owns only sequencing behavior. */
struct cubs2_lockstep_shared {
	uint32_t magic;
	uint32_t odometry_sequence;
	uint32_t response_sequence;
	uint32_t terminate;
	synapse_topic_OdometryData_t odometry;
	synapse_topic_PwmSignalOutputsData_t pwm;
	synapse_topic_AttitudeCommandData_t attitude;
};

_Static_assert(sizeof(struct cubs2_lockstep_shared) == 184,
	       "CUBS2 lockstep shared layout mismatch");
_Static_assert(offsetof(struct cubs2_lockstep_shared, odometry) == 16,
	       "CUBS2 odometry ABI offset mismatch");
_Static_assert(offsetof(struct cubs2_lockstep_shared, pwm) == 88, "CUBS2 PWM ABI offset mismatch");
_Static_assert(offsetof(struct cubs2_lockstep_shared, attitude) == 136,
	       "CUBS2 attitude ABI offset mismatch");

int cubs2_lockstep_start(void);
bool cubs2_lockstep_enabled(void);
int cubs2_lockstep_receive(synapse_topic_OdometryData_t *odometry);
int cubs2_lockstep_send(const synapse_topic_PwmSignalOutputsData_t *pwm,
			const synapse_topic_AttitudeCommandData_t *attitude);

#endif
