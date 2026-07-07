# CUBS2

`cerebri_cubs2` is a Zephyr fixed-wing control app for the CUBS2 remotely
piloted aircraft.

The board is an Ethernet/Zenoh control node:

- subscribe to `synapse/v1/topic/manual_control_command`
- subscribe to `synapse/v1/topic/mocap_frame`
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
`csyn topic list/info/echo/hz/watch` shell diagnostics, and the zenoh and
native_sim UDP transports.

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
Inbound `ManualControlData` is a fixed-layout struct payload, `MocapFrame`
remains a FlatBuffer table, and all outbound topics are fixed-layout struct
payloads.

The current Modelica source requires Rumoca GALEC fixes after `v0.9.11`.
The Nix build apps use a Rumoca compiler pinned to commit
`36503311c7622b65fdf94971e7547341b7f00b2e`. For non-Nix builds, point CMake at
a Rumoca compiler from that commit or newer:

```sh
export CUBS2_RUMOCA_EXECUTABLE=~/git/rumoca/target/debug/rumoca
```

After Rumoca is released with those fixes, update the pinned
`CUBS2_RUMOCA_VERSION` in `CMakeLists.txt` and the build can return to
downloading the verified release binary automatically.

Use separate build directories when switching boards:

```sh
west build -b mr_vmu_tropic -d build-mr_vmu_tropic cerebri_cubs2
west build -b native_sim -d build-native_sim cerebri_cubs2
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

The flake also exposes `.#build-native-sim`, `.#flash`, `.#menuconfig`, and an
inlined `nixosModules.default` for NixOS host setup.
