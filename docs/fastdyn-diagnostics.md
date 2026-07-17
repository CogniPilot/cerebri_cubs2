# CUBS2 FastDyn diagnostics

Use the CUBS2-owned `fastdyn/mr_vmu_tropic.toml` for static analysis, probing,
and trace analysis. Set paths explicitly so the vehicle repository and FastDyn
can live anywhere:

```sh
export CEREBRI_CUBS2_ROOT=/path/to/cerebri_cubs2
export CUBS2_WORKSPACE_ROOT=/path/to/cubs2-west-workspace
export CUBS2_FASTDYN_BUILD_DIR="$CEREBRI_CUBS2_ROOT/build-mr_vmu_tropic-fastdyn"
export FASTDYN_ROOT=/path/to/FastDyn
export FASTDYN_INSTALL_ROOT="$FASTDYN_ROOT"
export FASTDYN_QEMU_PATH="$FASTDYN_ROOT/build/qemu/build/qemu-system-arm"
export FASTDYN_MONITOR_ELF="$FASTDYN_ROOT/build/qemu/ws/monitor.elf"

cd "$FASTDYN_ROOT"
FASTDYN="$FASTDYN_ROOT/build/venv/bin/fastdyn"
CONFIG="$CEREBRI_CUBS2_ROOT/fastdyn/mr_vmu_tropic.toml"
SVD=third_party/common/cmsis-svd-data
```

Refresh static analysis whenever the firmware ELF, Modelica-generated control
code, board configuration, or rehosting configuration changes:

```sh
$FASTDYN static-analyze -c "$CONFIG" -s "$SVD" --force
```

Use probe and trace analysis for peripheral-model development:

```sh
$FASTDYN probe-run -c "$CONFIG" -s "$SVD" -o /tmp/cubs2-probe
$FASTDYN trace-analyze \
  -c "$CONFIG" \
  -s "$SVD" \
  -o /tmp/cubs2-analysis \
  --latest-run-dir /tmp/cubs2-probe
```

Execution at `PC=0x00000000` can be valid because
`Vehicles_Cubs2_OuterLoop_dostep` is relocated into ITCM at address zero.
Treat a probe heuristic at that address as diagnostic evidence, not by itself
as a firmware panic. The bounded mission in `fastdyn/run_mission.sh` is the
closed-loop acceptance test.
