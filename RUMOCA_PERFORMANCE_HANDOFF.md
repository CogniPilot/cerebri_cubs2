# Rumoca Performance Handoff: Eventful Flight SIL Simulation

## Summary

Rumoca Python binding simulation is extremely slow for the CUBS2 full flight SIL models that combine continuous fixed-wing plant dynamics with sampled discrete outer-loop control. The same Rumoca wheel simulates a simpler open-loop takeoff model quickly, so the current evidence points at the event/discrete-partition simulation path, not Python overhead, plotting, or CSV generation.

Observed with Rumoca `0.9.13` from the Nix-pinned Python wheel.

## Interactive Scenario Runner Binding Gap

The CUBS2 native-sim SIL scenario cannot currently run through the Python
binding because Rumoca `0.9.13` does not expose an interactive TOML scenario
runner. The installed module exposes:

```text
rumoca.version() -> 0.9.13
rumoca.Session.from_scenario(path)
model.simulate(...)
model.codegen(...)
```

It does not expose a Python function/class such as:

```text
rumoca.run_scenario(path)
rumoca.simulate_scenario(path)
rumoca.Scenario.from_file(path).run()
rumoca.Session.run_scenario(path)
```

Native SIL scenario requiring that API:

```text
/home/jgoppert/git/cerebri_cubs2/tests/zephyr/rumoca-scenario.native-sim.toml
```

Native SIL Modelica model:

```text
/home/jgoppert/git/cerebri_cubs2/tests/zephyr/Cubs2NativeSimSIL.mo
```

Native SIL schema:

```text
/home/jgoppert/git/cerebri_cubs2/tests/zephyr/native_sil_io.fbs
```

Source roots used by the native SIL scenario:

```text
/home/jgoppert/git/cerebri_cubs2/models/vendor/CMM-v0.0.2
/home/jgoppert/git/cerebri_cubs2/models/plant
```

The scenario TOML owns the lockstep timing, Zenoh transport, FlatBuffer schema,
publish/subscribe routes, signal mapping, debug log capture, model selection,
physics, and solver setup. The CUBS2 side intentionally does not implement
lockstep, transport, or simulation in handwritten Python; it only launches the
Zephyr app and bridges real Synapse topics while Rumoca should run the scenario.

Requested binding API behavior:

1. Accept a scenario TOML path from Python and run the same interactive
   lockstep/transport behavior exposed by Rumoca's scenario system.
2. Honor `[transport.zenoh]`, `[lockstep]`, `[schema]`, `[publish]`,
   `[subscribe]`, `[send]`, `[receive]`, `[signals.send]`, and `[debug_log]`
   from the TOML.
3. Return a process-style status or raise structured Python exceptions so CI can
   distinguish configuration errors, transport errors, simulation failures, and
   failed model checks.
4. Keep batch `Session.from_scenario(...); model.simulate(...)` available for
   non-interactive scenarios.

## Repository

Repository root:

```text
/home/jgoppert/git/cerebri_cubs2
```

Run all commands below from that directory.

## Scenario Files

Primary slow repro:

```text
/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.altitude.toml
```

Comparison fast scenario:

```text
/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.takeoff.toml
```

Other flight scenarios:

```text
/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.heading.toml
/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.pattern.toml
```

## Model Files And Included Libraries

Scenario model file:

```text
/home/jgoppert/git/cerebri_cubs2/tests/flight/Cubs2FlightScenarios.mo
```

Source roots used by the flight scenario TOMLs, resolved to absolute paths:

```text
/home/jgoppert/git/cerebri_cubs2/models/vendor/CMM-v0.0.2
/home/jgoppert/git/cerebri_cubs2/models/plant/SportCubSIL.mo
/home/jgoppert/git/cerebri_cubs2/models/plant/FixedWingSIL.mo
/home/jgoppert/git/cerebri_cubs2/src/FixedWingOuterLoop.mo
```

The relevant model names are:

```text
Cubs2TakeoffOpenLoop
Cubs2AltitudeHold
Cubs2HeadingHold
Cubs2PatternMission
```

The slow path starts when `Cubs2FlightScenarios.mo` instantiates:

```text
FixedWingOuterLoop
```

from:

```text
/home/jgoppert/git/cerebri_cubs2/src/FixedWingOuterLoop.mo
```

That controller contains sampled blocks using `when sample(0.0, dt)`:

```text
StateEstimator
RouteGuidance
TECSController
AttitudeController
PidController
```

The plant is continuous 6-DOF SportCub dynamics from:

```text
/home/jgoppert/git/cerebri_cubs2/models/plant/SportCubSIL.mo
```

The included vendor library root is:

```text
/home/jgoppert/git/cerebri_cubs2/models/vendor/CMM-v0.0.2
```

Important subpackages used by the plant:

```text
/home/jgoppert/git/cerebri_cubs2/models/vendor/CMM-v0.0.2/RigidBody
/home/jgoppert/git/cerebri_cubs2/models/vendor/CMM-v0.0.2/LieGroup
/home/jgoppert/git/cerebri_cubs2/models/vendor/CMM-v0.0.2/LieGroups
```

## Minimal Python Repro

```bash
cd /home/jgoppert/git/cerebri_cubs2

nix develop --quiet --option warn-dirty false --command python - <<'PY'
import time
import rumoca as rm

scenario = "/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.altitude.toml"
_session, model, config = rm.Session.from_scenario(scenario)

t0 = time.perf_counter()
result = model.simulate(t=(0.0, 2.0), config=config)
elapsed = time.perf_counter() - t0

print("rumoca", rm.version())
print("elapsed", elapsed)
print("samples", len(result.time))
print("final x z airspeed",
      float(result["x_m"][-1]),
      float(result["z_m"][-1]),
      float(result["airspeed_m_s"][-1]))
PY
```

Observed result:

```text
rumoca 0.9.13
elapsed approximately 15 seconds for 2 simulated seconds
samples 101 with dt=0.02
final state remains stable, around x=8.8 m, z=2.5 m, airspeed=4.3 m/s
```

Fast comparison:

```bash
cd /home/jgoppert/git/cerebri_cubs2

nix develop --quiet --option warn-dirty false --command python - <<'PY'
import time
import rumoca as rm

scenario = "/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.takeoff.toml"
_session, model, config = rm.Session.from_scenario(scenario)

t0 = time.perf_counter()
result = model.simulate(t=(0.0, 4.0), config=config)
elapsed = time.perf_counter() - t0

print("elapsed", elapsed)
print("samples", len(result.time))
print("final x z airspeed",
      float(result["x_m"][-1]),
      float(result["z_m"][-1]),
      float(result["airspeed_m_s"][-1]))
PY
```

Observed result:

```text
elapsed approximately 2 seconds for 4 simulated seconds
```

## Model Structure

Rumoca structure summaries:

```text
Cubs2TakeoffOpenLoop: 9 states, 111 algebraic, 0 inputs, 33 outputs, 92 parameters
Cubs2AltitudeHold:    9 states, 113 algebraic, 0 inputs, 30 outputs, 346 parameters
Cubs2HeadingHold:     9 states, 113 algebraic, 0 inputs, 31 outputs, 348 parameters
Cubs2PatternMission:  9 states, 113 algebraic, 0 inputs, 34 outputs, 348 parameters
```

The continuous state count is nearly identical between the fast open-loop model and slow closed-loop models. The main difference is the sampled discrete outer loop and large `__pre__`/event state surface.

## Stability Observations

The slow altitude scenario appears stable. This is probably not ground collision or spinning.

Completed 16 s altitude trace from:

```text
/home/jgoppert/git/cerebri_cubs2/artifacts/flight/altitude.csv
```

Observed values:

```text
time final:       16.0 s
x final:          63.36 m
z min/max/final:  2.39 / 3.13 / 3.06 m
airspeed min/max/final: 3.49 / 4.75 / 3.84 m/s
roll:             0 rad throughout
yaw:              0 rad throughout
```

Frame convention: position is East/North/Up, so positive `z` is altitude.

## Solver Timing Notes

For `Cubs2AltitudeHold`, 2 simulated seconds:

```text
solver="rk-like": approximately 15 s
solver="auto":    approximately 7.6 s
solver="bdf":     approximately 7.7 s
solver="diffsol": approximately 7.8 s
```

For the full 16 simulated second altitude scenario:

```text
solver="rk-like": roughly 4-5 minutes
solver="auto":    approximately 113 seconds
```

Changing output `dt` from `0.05` to `0.1` barely helps. Increasing `outerLoop.dt` from `0.02` to `0.1` or `0.2` only partially helps. This suggests the runtime is dominated by the event-capable row/slot evaluation path rather than output density alone.

## Perf Repro

```bash
cd /home/jgoppert/git/cerebri_cubs2
mkdir -p /home/jgoppert/git/cerebri_cubs2/artifacts/perf

timeout 3m nix develop --quiet --option warn-dirty false --command \
  perf record -F 99 -g --call-graph fp \
  -o /home/jgoppert/git/cerebri_cubs2/artifacts/perf/altitude-2s.data \
  -- python - <<'PY'
from pathlib import Path
import time
import rumoca as rm

scenario = Path("/home/jgoppert/git/cerebri_cubs2/tests/flight/rumoca-scenario.altitude.toml")
_session, model, config = rm.Session.from_scenario(str(scenario))
config.dt = 0.1
config.max_wall_seconds = 90.0

start = time.perf_counter()
result = model.simulate(t=(0.0, 2.0), config=config)
print("elapsed", time.perf_counter() - start)
print("samples", len(result.time), "names", len(result.names))
print("final", float(result["x_m"][-1]), float(result["z_m"][-1]), float(result["airspeed_m_s"][-1]))
PY

perf report --stdio --no-children \
  -i /home/jgoppert/git/cerebri_cubs2/artifacts/perf/altitude-2s.data
```

Existing perf data file:

```text
/home/jgoppert/git/cerebri_cubs2/artifacts/perf/altitude-2s.data
```

Perf summary:

```text
~52% self in rumoca_eval_solve::eval_row_prepared_fast
~9%  self in rumoca_eval_solve::runtime::SolveRuntime::refresh_slots_with_plan
~8%  self in rumoca_eval_solve::prepared::PreparedScalarProgramBlock::eval_target_assignment_row_inner
small amounts in atan2, malloc/free, memmove, rk45::combine_stage
```

This strongly suggests `model.simulate(...)` is not executing generated compact C for an ODE RHS plus output/update functions. It is spending most time in Rumoca's generic Rust solve row evaluator and slot-refresh machinery.

## Codegen Findings

Generated sampled controller C from the firmware build:

```text
/home/jgoppert/git/cerebri_cubs2/build-mr_vmu_tropic/generated/rumoca/FixedWingOuterLoop/ProductionCode/FixedWingOuterLoop.c
/home/jgoppert/git/cerebri_cubs2/build-mr_vmu_tropic/generated/rumoca/FixedWingOuterLoop/ProductionCode/FixedWingOuterLoop.h
```

That generated controller C is around 619 lines, but it shows heavy repeated expression expansion in `FixedWingOuterLoop_dostep`, especially around route waypoint selection, TECS command saturation, and heading/course error expressions.

Full scenario codegen attempts:

```text
model.codegen("embedded-c") -> fails: unsupported-feature:events
model.codegen("c-solve")    -> fails: unsupported-feature:events
model.codegen("rust-solve") -> fails: unsupported-feature:events
model.codegen("mlir")       -> fails: unsupported-feature:events
model.codegen("cuda-c")     -> fails: unsupported-feature:events
model.codegen("galec")      -> fails: unsupported-feature:continuous_states
```

`cranelift-solve-jit` exists as a target but is manifest-only:

```text
target 'cranelift-solve-jit' is manifest-only and does not define generated files yet
```

`jax-solve` renders:

```text
/home/jgoppert/git/cerebri_cubs2/artifacts/perf/altitude-jax-solve/Cubs2AltitudeHold_jax_solve.py
```

But the generated file appears inconsistent:

```text
Header says N_Y = 18
rhs references x[220]
```

So it should not be used as the fix without investigation.

## Expected Rumoca Behavior

For a model with 9 continuous states and a periodic sampled controller, simulation should be much faster than real time. The desired backend shape is:

```text
continuous ODE RHS:       xdot = f(t, x, discrete_state, params)
discrete sample update:   discrete_state_next = g(t, x, discrete_state, params)
output equation function: y = h(t, x, discrete_state, params)
known sample scheduler:   sample times from sample(0.0, dt), no generic root search needed
```

The object-oriented Modelica structure and record fields should be flattened away for simulation, as they are for generated code. The generic row/slot evaluator should not dominate runtime for this case.

## Requested Rumoca Fixes

1. Add or enable a fast simulation backend for models with continuous states plus periodic sampled discrete events.
2. Treat `when sample(start, interval)` as scheduled events instead of requiring expensive generic event/root handling.
3. Lower eventful solve models to compact compiled/JIT functions for:
   - continuous RHS
   - output equations
   - discrete/pre-state update
   - event guards
4. Make Python `model.simulate(...)` use that fast path automatically when possible.
5. Allow a solve target such as `c-solve`, `rust-solve`, or `cranelift-solve-jit` to handle this continuous+sampled-events case.
6. Fix or clarify `jax-solve` for this model. The generated RHS currently references state indices beyond `N_Y`.
7. Add profiling counters or optional diagnostics for:
   - number of RHS evaluations
   - number of event evaluations
   - number of row refreshes
   - time spent in slot refresh versus scalar row evaluation

## Important Local Constraint

The CUBS2 repo should continue to call Rumoca through Python bindings, not the Rumoca CLI. The goal is for Rumoca to own physics, control code generation, and simulation from the scenario TOMLs.
