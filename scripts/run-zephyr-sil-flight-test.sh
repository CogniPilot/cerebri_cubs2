#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
default_nix_flags=(
  --quiet
  --option warn-dirty false
  --option http-connections 1
  --option max-substitution-jobs 1
)

skip_west_update=0
skip_build=0
lockstep_only=0
run_args=()

log() {
  printf '%s\n' "$*" >&2
}

die() {
  log "error: $*"
  exit 1
}

usage() {
  cat >&2 <<'EOF'
Usage: scripts/run-zephyr-sil-flight-test.sh [options] [-- runner-args...]

Run the same Zephyr native_sim SIL sequence used by the GitHub CI
zephyr_sil_flight_test job.

Options:
  --skip-west-update   Reuse the current west workspace.
  --skip-build         Reuse build-native_sim/zephyr/zephyr.exe.
  --lockstep-only      Pass --lockstep-regression-only to the SIL runner.
  -h, --help           Show this help.

Environment:
  NIX_FLAGS            Extra flags passed to nix. Defaults to the CI flags.

Examples:
  scripts/run-zephyr-sil-flight-test.sh
  scripts/run-zephyr-sil-flight-test.sh --skip-west-update --lockstep-only
  scripts/run-zephyr-sil-flight-test.sh --skip-west-update --skip-build -- --t-end 3
EOF
}

run() {
  log "+ $*"
  "$@"
}

parse_args() {
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --skip-west-update)
        skip_west_update=1
        ;;
      --skip-build)
        skip_build=1
        ;;
      --lockstep-only)
        lockstep_only=1
        ;;
      -h | --help)
        usage
        exit 0
        ;;
      --)
        shift
        run_args+=("$@")
        break
        ;;
      *)
        run_args+=("$1")
        ;;
    esac
    shift
  done
}

nix_cmd() {
  local flags=()

  if [ -n "${NIX_FLAGS:-}" ]; then
    # Intentionally split NIX_FLAGS like GitHub Actions does in `nix $NIX_FLAGS run`.
    # shellcheck disable=SC2206
    flags=(${NIX_FLAGS})
  else
    flags=("${default_nix_flags[@]}")
  fi

  run nix "${flags[@]}" "$@"
}

main() {
  parse_args "$@"
  cd "$repo_root"

  command -v nix >/dev/null 2>&1 || die "nix is not on PATH; try ./scripts/setup-nix.sh"

  if [ "$lockstep_only" -eq 1 ]; then
    run_args+=(--lockstep-regression-only)
  fi

  if [ "$skip_west_update" -eq 0 ]; then
    nix_cmd run .#west-update
  fi

  if [ "$skip_build" -eq 0 ]; then
    nix_cmd run .#build-native-sim
  fi

  if [ ! -f build-native_sim/zephyr/zephyr.exe ]; then
    die "missing build-native_sim/zephyr/zephyr.exe; rerun without --skip-build"
  fi

  run chmod +x build-native_sim/zephyr/zephyr.exe
  nix_cmd run .#native-sim-sil-run -- \
    --sim build-native_sim/zephyr/zephyr.exe \
    "${run_args[@]}"

  if [ "$lockstep_only" -eq 0 ]; then
    run test -s artifacts/native-sim-sil/native-sim-overview.png
  fi
}

main "$@"
