# CUBS2

`cerebri_cubs2` is a Zephyr fixed-wing control app for the CUBS2 remotely
piloted aircraft.

The board is an Ethernet/Zenoh control node:

- subscribe to `synapse/v1/topic/manual_control_command`
- subscribe to `synapse/v1/topic/mocap_frame`
- mirror inbound payloads from csyn into zros
- run the generated fixed-wing controller from zros state
- publish `synapse/v1/topic/pwm_signal_outputs`
- publish `vehicle_health`, `attitude_estimate`, `attitude_command`, and
  `control_loop_metrics` synapse topics for controller diagnostics

The app itself (`src/`) is only the control loop plus the generated
controller. Everything topic-related lives in the
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

Use separate build directories when switching boards:

```sh
west build -b mr_vmu_tropic -d build-mr_vmu_tropic cerebri_cubs2
west build -b native_sim -d build-native_sim cerebri_cubs2
```

## Nix Environment

Nix support is optional and lives under `nix/` so the repository root stays
focused on the normal west application layout.

Because the flake is not in the repository root, use `./nix` in Nix commands;
bare `nix develop` will not find it.

To install or configure Nix for this checkout and verify the flake:

```sh
./scripts/setup-nix.sh
```

Then use the pinned host tools from the repository root:

```sh
nix develop ./nix
nix run ./nix#west-update
nix run ./nix#build
```

Full Nix, NixOS module, and flake app details are in
[nix/README.md](nix/README.md).
