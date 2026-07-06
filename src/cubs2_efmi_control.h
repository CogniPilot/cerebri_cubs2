#ifndef CUBS2_EFMI_CONTROL_H_
#define CUBS2_EFMI_CONTROL_H_

#include "FixedWingOuterLoop.h"
#include "efmi_wrapper.h"

#define CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_S 0.02
#define CUBS2_FIXED_WING_OUTER_LOOP_PERIOD_NS 20000000

static inline void cubs2_efmi_fixed_wing_outer_loop_init(FixedWingOuterLoopState *state)
{
	CUBS2_EFMI_INIT(FixedWingOuterLoop, state);
}

static inline void cubs2_efmi_fixed_wing_outer_loop_recalibrate(FixedWingOuterLoopState *state)
{
	CUBS2_EFMI_RECALIBRATE(FixedWingOuterLoop, state);
}

static inline void cubs2_efmi_fixed_wing_outer_loop_step(FixedWingOuterLoopState *state)
{
	CUBS2_EFMI_STEP(FixedWingOuterLoop, state);
}

#endif
