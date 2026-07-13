#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
build_dir="${1:-$root/build-native_sim_realtime}"
if [[ "$build_dir" != /* ]]; then
  build_dir="$root/$build_dir"
fi
model_dir="$build_dir/generated/rumoca/FixedWingOuterLoop/ProductionCode"
schema_dir="$build_dir/_deps/synapse_fbs_c-src"
if [[ ! -d "$schema_dir" && -f "$build_dir/CMakeCache.txt" ]]; then
  schema_dir="$(sed -n 's/^synapse_fbs_c_SOURCE_DIR:STATIC=//p' "$build_dir/CMakeCache.txt" | head -1)"
fi
if [[ ! -d "$schema_dir" ]]; then
  printf 'Synapse C package not found for build directory: %s\n' "$build_dir" >&2
  exit 1
fi
output="${TMPDIR:-/tmp}/cubs2-runtime-control-test"

cc -std=c11 -Wall -Wextra -Wno-misleading-indentation -pthread \
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
