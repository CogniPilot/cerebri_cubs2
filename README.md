# Cerebri CubS2

[![CI](https://github.com/CogniPilot/cerebri_cubs2/actions/workflows/ci.yml/badge.svg)](https://github.com/CogniPilot/cerebri_cubs2/actions/workflows/ci.yml)

`cerebri_cubs2` is a Zephyr fixed-wing control app for the CUBS2 remotely
piloted aircraft.

The board is an Ethernet/Zenoh control node:

- subscribe to `manual`
- subscribe to realtime `qualisys/cub1/pose_raw` or SIL `odom`
- mirror inbound payloads from csyn into zros
- run the Rumoca-generated fixed-wing eFMI controller from zros state
- publish `pwm`
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
Inbound `ManualControlData` and `OdometryData` are fixed-layout struct
payloads, and all outbound topics are fixed-layout struct payloads.

Modelica code generation and SIL simulation run through the Rumoca Python
binding. The Nix environment pins the Rumoca `v0.9.19` package and exports
`CUBS2_RUMOCA_PYTHON` for CMake. For non-Nix builds, use a Python interpreter
with `rumoca` installed or set `CUBS2_RUMOCA_PYTHON=/path/to/python`.

The default Nix input uses the portable upstream `v0.9.19` source. To test a
Rumoca workspace next to this repository without changing `flake.nix` or
`flake.lock`, override that input with a relative path:

```sh
nix run --override-input rumoca-src \
  "git+file://$(realpath ../rumoca)?ref=main" \
  .#native-sim-sil-test
```

The override can point at any Rumoca Git checkout. No user-specific absolute
path is stored in the Nix configuration.

The flight SIL test passes each `rumoca-scenario.*.toml` file to the Rumoca
Python API, so Rumoca owns the physics, controller compilation, solver, and
scenario settings. Python only normalizes traces for checks and plots. CI runs
the flight SIL test through `nix run .#flight-sil-test` and uploads the
generated CSV, PNG, Markdown, and HTML report artifacts.

The pattern scenario uses the checked-in mission. When
`CONFIG_CUBS2_RUNTIME_CONTROL=y`, deployed realtime, SIL, and FastDyn builds can
stage a new mission at a control-cycle boundary via the `trajectory_set` Zenoh
queryable service. This feature defaults on with CSyn Zenoh and is disabled
automatically for network-free FastDyn profiles.

Use separate build directories when switching boards:

```sh
west build -b mr_vmu_tropic -d build-mr_vmu_tropic cerebri_cubs2
west build -b native_sim -d build-native_sim cerebri_cubs2
west build -b native_sim/native/64 -d build-native_sim_native_64 cerebri_cubs2
```

## Realtime and SIL native_sim builds

`native_sim` has two mutually exclusive Kconfig execution modes. Realtime is
the default; `boards/native_sim_sil.conf` selects `CONFIG_CUBS2_LOCKSTEP=y`:

- **Realtime** (`build-native_sim`, the default config above): the flying
  ground-side autopilot. The control loop paces itself on the wall clock,
  subscribes to the Zenoh mocap pose key directly, and publishes mission
  telemetry over the csyn bridge.
- **Lockstep SIL** (`build-native_sim_sil`): the regression/simulation build.
  It consumes external odometry (Zenoh bridge or the shared-memory lockstep
  transport) and steps on simulation time:

```sh
west build -b native_sim -d build-native_sim_sil cerebri_cubs2 -- \
  -DEXTRA_CONF_FILE=$PWD/cerebri_cubs2/boards/native_sim_sil.conf
```

`nix run .#build-native-sim` and `.#build-native-sim-64` produce the SIL
flavor (into `build-native_sim_sil` and `build-native_sim_native_64_sil`),
which is what CI tests and releases ship. Keep the two build directories
separate so the flying binary can never inherit a SIL configuration.

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

The Nix commands use an isolated CUBS2 West workspace under
`.devenv/state/west/` by default. Set `CUBS2_WEST_WORKSPACE=/path/to/workspace`
to choose its location explicitly; the selected workspace is governed only by
this repository's `west.yml`.

The flake also exposes `.#build-native-sim`, `.#build-native-sim-64`, `.#flash`,
`.#menuconfig`, and an inlined `nixosModules.default` for NixOS host setup.

To run the Zephyr `native_sim` SIL test, use:

```sh
nix run .#west-update
nix run .#native-sim-sil-test
```

The default path generates and builds an FMI 3 Co-Simulation FMU with Rumoca,
then runs the FMU and the Zephyr controller in compiled C. A native-only shared
memory transport carries the fixed-layout Synapse odometry, PWM, and attitude
structs without putting the per-step loop in Python. Python only orchestrates
the processes and produces CSV, PNG, Markdown, and HTML checks. Hardware builds
do not include the native transport behavior.

The generated source FMU includes the official FMI 3.0.2 headers and validates
and simulates with FMPy. The native runner uses those generated function types;
its custom code is limited to the Zephyr lockstep transport, pacing, and test
telemetry rather than a separate FMI ABI.

The interpreted Rumoca scenario remains available as a routed reference with
`--plant-backend rumoca`; its TOML owns the solver, Zenoh, schema, and trace
settings.

For a focused lockstep timing regression without the full flight-quality run,
reuse an existing native-sim executable:

```sh
nix run .#native-sim-sil-run -- \
  --sim build-native_sim_sil/zephyr/zephyr.exe \
  --artifacts artifacts/native-sim-lockstep \
  --lockstep-regression-only
```

With `--plant-backend rumoca`, the same traffic is inspectable from another
terminal while the test is running:

```sh
nix develop -c csyn --connect udp/127.0.0.1:7447 topic echo odom
nix develop -c csyn --connect udp/127.0.0.1:7447 topic hz pwm
nix develop -c csyn --connect udp/127.0.0.1:7447 topic echo att_sp
```

The compiled FMI path can also be paced for interactive controller diagnostics:

```sh
nix run .#native-sim-sil-run -- --sim-speed 1
nix develop -c csyn --connect udp/127.0.0.1:7447 topic hz pwm
```

At realtime speed the native lockstep wait yields to the CSyn/Zenoh threads, so
the controller's outbound topics remain available to the CLI. The FMI plant's
inbound odometry stays on the private shared-memory link; use
`--plant-backend rumoca` when that inbound topic must also be inspected.

A Zenoh ground station can connect to the same router while the native_sim
executable is running.

For the 64-bit native simulator target, use:

```sh
nix run .#build-native-sim-64
nix run .#native-sim-64-sil-test
```

## FastDyn execution modes

The FastDyn base profile selects `CONFIG_CUBS2_LOCKSTEP=y`. It uses the direct
shared-memory simulator transport and runs without Ethernet or Zenoh. To build
the mutually exclusive realtime mode, append the checked-in configuration
fragment after FastDyn's base profile:

```sh
base="$(realpath ../FastDyn/tests/integration/cerebri_cubs2_fastdyn.conf)"
realtime="$(realpath boards/mr_vmu_tropic_fastdyn_realtime.conf)"
overlay="$(realpath ../FastDyn/tests/integration/cerebri_cubs2_fastdyn.overlay)"

CUBS2_BUILD_DIR="$PWD/build-mr_vmu_tropic-fastdyn-realtime" \
  nix run .#build -- -p always -- \
  "-DEXTRA_CONF_FILE=$base;$realtime" \
  "-DDTC_OVERLAY_FILE=$overlay"
```

FastDyn runs QEMU with a virtual instruction-count clock, so its host FMI3
bridge owns realtime pacing. Run a realtime image with
`CUBS2_FASTDYN_SIM_SPEED=1`; accelerated lockstep keeps the default value.
Both modes retain the same fixed-layout shared-memory ABI. Add FastDyn's
communications profile when runtime `param_get`, `param_set`, and
`trajectory_set` services are needed; `CONFIG_CUBS2_RUNTIME_CONTROL` then
defaults on with `CONFIG_CSYN_ZENOH`.
