#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
build_dir="${1:-$root/build-native_sim_realtime}"
if [[ "$build_dir" != /* ]]; then
  build_dir="$root/$build_dir"
fi
model_dir="$build_dir/generated/rumoca/FixedWingOuterLoop/ProductionCode"
schema_dir="$build_dir/_deps/synapse_fbs_c-src"
output="${TMPDIR:-/tmp}/cubs2-runtime-control-test"

cc -std=c11 -Wall -Wextra -Wno-misleading-indentation \
  -DCONFIG_CUBS2_RUNTIME_CONTROL=1 \
  -I"$root/tests/runtime_control/include" \
  -I"$root/src" \
  -I"$model_dir" \
  -I"$schema_dir/include" \
  "$root/tests/runtime_control/runtime_control_test.c" \
  "$root/src/runtime_control.c" \
  "$model_dir/FixedWingOuterLoop.c" \
  "$schema_dir/src/flatcc-runtime/builder.c" \
  "$schema_dir/src/flatcc-runtime/emitter.c" \
  "$schema_dir/src/flatcc-runtime/refmap.c" \
  "$schema_dir/src/flatcc-runtime/verifier.c" \
  -lm -o "$output"

"$output"
