# Cerebri CubS2

[![CI](https://github.com/CogniPilot/cerebri_cubs2/actions/workflows/ci.yml/badge.svg)](https://github.com/CogniPilot/cerebri_cubs2/actions/workflows/ci.yml)

`cerebri_cubs2` is a Zephyr fixed-wing control app for the CUBS2 remotely
piloted aircraft.

The board is an Ethernet/Zenoh control node:

- subscribe to `synapse/v1/topic/manual_control_command`
- subscribe to `synapse/v1/topic/external_odometry`
- mirror inbound payloads from csyn into zros
- run the Rumoca-generated fixed-wing eFMI controller from zros state
- publish `synapse/v1/topic/pwm_signal_outputs`
- publish `vehicle_health`, `attitude_estimate`, `attitude_command`, and
  `control_loop_metrics` synapse topics for controller diagnostics

The app itself (`src/`) is the control loop and the single Modelica controller
source. The fixed-wing controller equations live in `src/FixedWingOuterLoop.mo`;
CMake generates the eFMU plus C production code under
`${CMAKE_BINARY_DIR}/generated/rumoca`. Generated Rumoca artifacts are not
committed source.

Everything topic-related lives in the
[csyn](https://github.com/CogniPilot/csyn) west module
(`modules/lib/csyn`), which provides the topic store, the synapse_fbs
schema pin, the zros bridge and topic definitions, the
`csyn topic list/info/echo/hz/watch` shell diagnostics, and the Zenoh
transport used by both hardware and `native_sim`.

There are no local RC, sensor, actuator, storage, driver, schema, or include
directories in this app.

## Workspace

Bootstrap a fresh west workspace with this repo as the manifest:

```sh
mkdir -p /tmp/cerebri-ws
git clone https://github.com/CogniPilot/cerebri_cubs2 /tmp/cerebri-ws/cerebri_cubs2
cd /tmp/cerebri-ws
west init -l cerebri_cubs2
west update
west build -b mr_vmu_tropic cerebri_cubs2
```

The csyn module downloads the pinned `synapse_fbs-c.tar.gz` release asset
and uses the generated C headers from that release, so the schema version is
locked by csyn rather than per app. No local `flatc` install is required.
Inbound `ManualControlData` and `ExternalOdometryData` are fixed-layout struct
payloads, and all outbound topics are fixed-layout struct payloads.

Modelica code generation and SIL simulation run through the Rumoca Python
binding. The Nix environment pins the Rumoca `v0.9.13` wheel and exports
`CUBS2_RUMOCA_PYTHON` for CMake. For non-Nix builds, use a Python interpreter
with `rumoca` installed or set `CUBS2_RUMOCA_PYTHON=/path/to/python`.

The flight SIL test passes each `rumoca-scenario.*.toml` file to the Rumoca
Python API, so Rumoca owns the physics, controller compilation, solver, and
scenario settings. Python only normalizes traces for checks and plots. CI runs
the flight SIL test through `nix run .#flight-sil-test` and uploads the
generated CSV, PNG, Markdown, and HTML report artifacts.

Use separate build directories when switching boards:

```sh
west build -b mr_vmu_tropic -d build-mr_vmu_tropic cerebri_cubs2
west build -b native_sim -d build-native_sim cerebri_cubs2
west build -b native_sim/native/64 -d build-native_sim_native_64 cerebri_cubs2
```

## Nix Environment

Nix support is optional and lives in the root `flake.nix` so it is easy to
find from a fresh checkout.

To install or configure Nix for this checkout and verify the flake:

```sh
./scripts/setup-nix.sh
```

Then use the pinned host tools from the repository root:

```sh
nix develop
nix run .#west-update
nix run .#build
```

The flake also exposes `.#build-native-sim`, `.#build-native-sim-64`, `.#flash`,
`.#menuconfig`, and an inlined `nixosModules.default` for NixOS host setup.

To run the Zephyr `native_sim` app through Zenoh, use:

```sh
nix run .#west-update
nix run .#native-sim-sil-test
```

The native-sim SIL runner starts `zenohd` on `udp/127.0.0.1:7447`, launches
`build-native_sim/zephyr/zephyr.exe`, and passes
`tests/zephyr/rumoca-scenario.native-sim.toml` to the Rumoca Python
`Session.run_scenario(...)` path. The TOML owns the lockstep, Zenoh, schema,
publish/subscribe, debug log, physics, and solver setup. Rumoca uses the
`synapse_fbs` FlatBuffers wrapper schemas for the SIL transport, while the
bridge forwards fixed-layout Synapse topic payloads to and from the Zephyr app.
It then writes CSV, PNG, Markdown, and HTML artifacts that grade route laps,
altitude, velocity, bank, pitch, crosstrack tracking, and the 1 s native-sim
lockstep boot-time acknowledgement.

For a focused lockstep timing regression without the full flight-quality run,
reuse an existing native-sim executable:

```sh
nix run .#native-sim-sil-run -- \
  --sim build-native_sim/zephyr/zephyr.exe \
  --artifacts artifacts/native-sim-lockstep \
  --lockstep-regression-only
```

The same traffic is inspectable from another terminal while the test is
running:

```sh
nix develop -c csyn --connect udp/127.0.0.1:7447 topic echo external_odometry
nix develop -c csyn --connect udp/127.0.0.1:7447 topic hz pwm_signal_outputs
nix develop -c csyn --connect udp/127.0.0.1:7447 topic echo attitude_command
```

A Zenoh ground station can connect to the same router while the native_sim
executable is running.

For the 64-bit native simulator target, use:

```sh
nix run .#build-native-sim-64
nix run .#native-sim-64-sil-test
```
