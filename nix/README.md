# Nix Support

This directory contains the optional Nix flake and NixOS module for
`cerebri_cubs2`. The normal project root remains a west application checkout.

Run commands below from the repository root unless a command says otherwise.
Because the flake lives in this directory, use `./nix` in Nix commands. Bare
`nix develop` from the repository root only works when `flake.nix` is also in
the repository root.

## First-Time Setup

The setup helper installs Nix when needed, enables `nix-command` and `flakes`
for the invoking user, and verifies this flake:

```sh
./scripts/setup-nix.sh
```

On NixOS, prefer declarative system configuration:

```nix
{
  nix.package = pkgs.nixVersions.latest;
  nix.settings.experimental-features = [ "nix-command" "flakes" ];
}
```

The provided NixOS module enables those settings by default when
`programs.cerebri-cubs2.enable = true`.

## Development Shell

Enter the pinned host-tool environment:

```sh
nix develop ./nix
```

Inside the shell, the main commands are:

```sh
cubs2-west-update
cubs2-build
cubs2-build-native-sim
cubs2-flash
```

`cubs2-build` defaults to `mr_vmu_tropic` and `build-mr_vmu_tropic`.
`cubs2-build-native-sim` defaults to `native_sim` and `build-native_sim`.

## Flake Apps

The same commands are exposed as flake apps:

```sh
nix run ./nix#west-update
nix run ./nix#build
nix run ./nix#build-native-sim
nix run ./nix#flash
nix run ./nix#menuconfig
```

## West Workspace Handling

If this checkout is already inside the west workspace initialized from this
repo's `west.yml`, the Nix commands use that workspace directly.

If this checkout is nested inside a different west workspace, `cubs2-west-update`
creates a private module workspace under
`${XDG_CACHE_HOME:-$HOME/.cache}/cerebri-cubs2` and exports it through
`CUBS2_WORKSPACE_ROOT` for the build commands.

Set `CUBS2_WEST_WORKSPACE` to choose a different managed workspace location.
Set `CUBS2_ALLOW_FOREIGN_WEST=1` to use the currently active west workspace
even when its manifest is not this app's `west.yml`.

## NixOS Module

Import the module and enable the host tools:

```nix
{
  inputs.cerebri-cubs2.url = "github:CogniPilot/cerebri_cubs2?dir=nix";

  outputs = { self, nixpkgs, cerebri-cubs2, ... }: {
    nixosConfigurations.devbox = nixpkgs.lib.nixosSystem {
      system = "x86_64-linux";
      modules = [
        cerebri-cubs2.nixosModules.default
        {
          programs.cerebri-cubs2 = {
            enable = true;
            users = [ "alice" ];
          };
        }
      ];
    };
  };
}
```

The module installs the host tools, adds optional debug-probe udev rules, and
can add selected users to the serial/debug access groups.
