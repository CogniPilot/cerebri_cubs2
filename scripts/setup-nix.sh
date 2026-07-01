#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
flake_dir="$repo_root/nix"
nix_conf_dir="${XDG_CONFIG_HOME:-$HOME/.config}/nix"
nix_conf="$nix_conf_dir/nix.conf"

log() {
  printf '%s\n' "$*" >&2
}

die() {
  log "error: $*"
  exit 1
}

is_nixos() {
  [ -e /etc/NIXOS ]
}

source_nix_profile() {
  local profile

  for profile in \
    /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh \
    "$HOME/.nix-profile/etc/profile.d/nix.sh"
  do
    if [ -r "$profile" ]; then
      # shellcheck source=/dev/null
      . "$profile"
    fi
  done
}

install_nix_if_missing() {
  local install_args=()

  if command -v nix >/dev/null 2>&1; then
    return 0
  fi

  if is_nixos; then
    die "this is NixOS but nix is not on PATH; fix the system NixOS configuration first"
  fi

  command -v curl >/dev/null 2>&1 ||
    die "curl is required to install Nix from https://nixos.org/nix/install"

  case "$(uname -s)" in
    Linux)
      if command -v systemctl >/dev/null 2>&1 &&
        [ "$(ps -p 1 -o comm= 2>/dev/null || true)" = "systemd" ]; then
        install_args=(--daemon)
      else
        install_args=(--no-daemon)
      fi
      ;;
    Darwin)
      install_args=()
      ;;
    *)
      die "unsupported OS for automatic Nix install: $(uname -s)"
      ;;
  esac

  log "Installing Nix with the official installer..."
  curl --proto '=https' --tlsv1.2 -fsSL https://nixos.org/nix/install |
    sh -s -- "${install_args[@]}"

  source_nix_profile
  command -v nix >/dev/null 2>&1 ||
    die "Nix was installed, but nix is not on PATH yet; open a new shell and rerun this script"
}

ensure_flake_config() {
  local tmp

  mkdir -p "$nix_conf_dir"
  touch "$nix_conf"
  tmp="$(mktemp)"

  awk '
    function trim(value) {
      sub(/^[ \t]+/, "", value)
      sub(/[ \t]+$/, "", value)
      return value
    }

    function has_feature(value, feature) {
      return index(" " value " ", " " feature " ") > 0
    }

    /^[ \t]*experimental-features[ \t]*=/ {
      found = 1
      value = $0
      sub(/^[^=]*=/, "", value)
      value = trim(value)

      if (!has_feature(value, "nix-command")) {
        value = trim(value " nix-command")
      }
      if (!has_feature(value, "flakes")) {
        value = trim(value " flakes")
      }

      print "experimental-features = " value
      next
    }

    { print }

    END {
      if (!found) {
        print "experimental-features = nix-command flakes"
      }
    }
  ' "$nix_conf" >"$tmp"

  mv "$tmp" "$nix_conf"
}

print_nixos_hint() {
  if ! is_nixos; then
    return 0
  fi

  cat >&2 <<'EOF'

For declarative NixOS setup, add this to your system configuration:

  {
    nix.package = pkgs.nixVersions.latest;
    nix.settings.experimental-features = [ "nix-command" "flakes" ];
  }

Then run:

  sudo nixos-rebuild switch

The user-level nix.conf was also updated, so the current user can use this
repo immediately without waiting for a system rebuild.
EOF
}

verify_flakes() {
  nix flake metadata "$flake_dir" >/dev/null 2>&1 ||
    die "Nix is installed, but flakes are still not enabled; check $nix_conf"
}

main() {
  source_nix_profile
  install_nix_if_missing
  ensure_flake_config
  verify_flakes
  print_nixos_hint

  log "Nix is ready: $(nix --version)"
  log "Next:"
  log "  nix develop ./nix"
  log "  nix run ./nix#west-update"
  log "  nix run ./nix#build"
}

main "$@"
