# CUBS2 binary-in-the-loop development

CUBS2 owns its FastDyn integration. FastDyn supplies the generic QEMU
rehosting runtime; it does not own CUBS2 board policy, lockstep protocol, or
mission acceptance criteria.

The repository-owned pieces are:

- `fastdyn/mr_vmu_tropic.toml`: rehosting and process configuration;
- `fastdyn/prj.conf` and `fastdyn/mr_vmu_tropic.overlay`: Zephyr build inputs;
- `fastdyn/comms.conf`: optional Ethernet/Zenoh side channel;
- `tools/fastdyn_bridge`: compiled shared-memory bridge;
- `fastdyn/run_mission.sh`: bounded mission and artifact checks.

The closed loop is:

```text
Rumoca FMI 3 plant <-> CUBS2 bridge <-> FastDyn/QEMU RAM <-> CUBS2 ARM firmware
```

The firmware and bridge exchange generated `synapse_fbs` payloads. Vehicle
physics stays in `modelica_models`; FastDyn contains no CUBS2 plant equations.

## Standalone repository setup

Start in this repository. Its West manifest fetches Zephyr, the reusable
Modelica library, and the firmware modules into an isolated workspace:

```sh
nix run .#west-update
```

Build the hardware binary with the CUBS2-owned rehosting inputs:

```sh
conf_file="$(realpath fastdyn/prj.conf)"
overlay="$(realpath fastdyn/mr_vmu_tropic.overlay)"
CUBS2_BUILD_DIR="$PWD/build-mr_vmu_tropic-fastdyn" \
  nix run .#build -- -p always -- \
    -DCONF_FILE="$conf_file" \
    -DDTC_OVERLAY_FILE="$overlay"

cargo test --release --locked --manifest-path tools/fastdyn_bridge/Cargo.toml
cargo build --release --locked --manifest-path tools/fastdyn_bridge/Cargo.toml
```

FastDyn is a separate tool dependency and may be checked out anywhere. Point
the mission at that installation and at this repository's West workspace:

```sh
export FASTDYN_ROOT=/path/to/FastDyn
export CUBS2_WORKSPACE_ROOT=/path/to/cubs2-west-workspace
export CUBS2_FASTDYN_BUILD_DIR="$PWD/build-mr_vmu_tropic-fastdyn"
fastdyn/run_mission.sh
```

There is no sibling-directory requirement. `FASTDYN_ROOT` and
`CUBS2_WORKSPACE_ROOT` are explicit because they identify independently owned
tool and dependency workspaces. The CogniPilot Devenv profile supplies these
values automatically when all repositories are being edited together.

## Optional communications profile

The base profile uses direct lockstep shared memory and does not require a
network. To retain Ethernet, CSyn, and Zenoh as an asynchronous diagnostics
channel, merge the vehicle-owned communications fragment:

```sh
conf_file="$(realpath fastdyn/prj.conf)"
extra_conf="$(realpath fastdyn/comms.conf)"
overlay="$(realpath fastdyn/mr_vmu_tropic.overlay)"
CUBS2_BUILD_DIR="$PWD/build-mr_vmu_tropic-fastdyn-comms" \
  nix run .#build -- -p always -- \
    -DCONF_FILE="$conf_file" \
    -DEXTRA_CONF_FILE="$extra_conf" \
    -DDTC_OVERLAY_FILE="$overlay"
```

Set `FASTDYN_CUBS2_NETWORK_SETUP=true` only for this communications profile.
The network remains a side channel and never paces the simulation.

## Outputs and overrides

The default mission writes its logs, traces, CSV files, and reports below
`artifacts/bil/`, including the canonical
`work/cerebri_cubs2_fmi3/mission-trajectory.csv` used by
`nix run .#trajectory-compare`. Useful mission overrides are:

```sh
export CUBS2_FASTDYN_T_END=10
export CUBS2_FASTDYN_SIM_SPEED=1000
export CUBS2_FASTDYN_STARTUP_TIMEOUT_S=60
export CUBS2_FASTDYN_RESPONSE_TIMEOUT_S=30
```

For the equivalent host-native firmware path, run
`nix run .#native-sim-64-sil-test`. Both paths use the same named CUBS2 plant
and acceptance scenarios from `modelica_models`; only firmware execution and
transport differ.
