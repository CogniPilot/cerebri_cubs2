/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CUBS2_RUNTIME_CONTROL_H_
#define CUBS2_RUNTIME_CONTROL_H_

#include <stdbool.h>

#include "FixedWingOuterLoop.h"

/* Register Synapse ParamSet/Get and TrajectorySet/Get services.  Incoming
 * changes are staged by the Zenoh thread and applied atomically by
 * cubs2_runtime_control_apply() on the 50 Hz controller thread. */
bool cubs2_runtime_control_init(FixedWingOuterLoopState *model);
void cubs2_runtime_control_apply(FixedWingOuterLoopState *model, bool armed);

#endif
