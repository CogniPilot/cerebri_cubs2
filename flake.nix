{
  description = "Nix development and host support for the cerebri_cubs2 Zephyr app";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rumoca-src = {
      url = "github:CogniPilot/rumoca/v0.9.19";
      flake = false;
    };
  };

  outputs =
    { self, nixpkgs, rumoca-src }:
    let
      lib = nixpkgs.lib;
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = lib.genAttrs supportedSystems;
      pkgsFor = system: import nixpkgs { inherit system; };
      appDirectory = "cerebri_cubs2";
      appDisplayName = "CUBS2";
      appCacheName = "cerebri-cubs2";
      defaultBoard = "mr_vmu_tropic";
      defaultNativeSimBoard = "native_sim";
      defaultNativeSim64Board = "native_sim/native/64";
      rumocaVersion = "0.9.19";
      synapseFbsVersion = "0.6.0";
      mkRumocaPythonPackage =
        pkgs:
        pkgs.python3Packages.buildPythonPackage {
          pname = "rumoca";
          version = rumocaVersion;
          pyproject = true;

          src = rumoca-src;
          sourceRoot = "source/crates/rumoca-bind-python";
          cargoRoot = "../..";

          cargoDeps = pkgs.rustPlatform.fetchCargoVendor {
            src = rumoca-src;
            cargoRoot = ".";
            name = "rumoca-${rumocaVersion}-cargo-vendor";
            hash = "sha256-8KcX5LawzhwqQv7+yd65l4fLCQy+1x31tpCU6ec/ZGg=";
          };

          nativeBuildInputs = [
            pkgs.pkg-config
            pkgs.rustfmt
            pkgs.rustPlatform.cargoSetupHook
            pkgs.rustPlatform.maturinBuildHook
          ];

          postPatch = ''
            chmod -R u+w ../..
          '';

          buildInputs = [
            pkgs.udev
          ];

          env.CARGO_TARGET_DIR = "./target";
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";

          doCheck = false;
          pythonImportsCheck = [ "rumoca" ];

          meta = {
            description = "Rumoca Modelica compiler Python binding";
            homepage = "https://github.com/CogniPilot/rumoca";
            license = lib.licenses.asl20;
            platforms = supportedSystems;
          };
        };
      mkSynapseFbsPythonPackage =
        pkgs:
        pkgs.python3Packages.buildPythonPackage {
          pname = "synapse-fbs";
          version = synapseFbsVersion;
          pyproject = true;

          src = pkgs.fetchurl {
            url = "https://files.pythonhosted.org/packages/source/s/synapse-fbs/synapse_fbs-${synapseFbsVersion}.tar.gz";
            hash = "sha256-armcV2B+tkQMSZwFbfXb1i0msxt3e3dP2ptERV7QyZM=";
          };

          nativeBuildInputs = [
            pkgs.python3Packages.setuptools
            pkgs.python3Packages.wheel
          ];

          propagatedBuildInputs = [
            pkgs.python3Packages.flatbuffers
          ];

          doCheck = false;
          pythonImportsCheck = [
            "synapse.topic.ExternalOdometry"
            "synapse.topic.PwmSignalOutputs"
          ];

          meta = {
            description = "Generated Python FlatBuffers bindings for Synapse schemas";
            homepage = "https://github.com/CogniPilot/synapse_fbs";
            license = lib.licenses.asl20;
            platforms = supportedSystems;
          };
        };
      mkPythonEnv =
        pkgs: rumocaPythonPackage: synapseFbsPythonPackage:
        pkgs.python3.withPackages (
          ps: with ps; [
            anytree
            intelhex
            jinja2
            jsonschema
            matplotlib
            numpy
            packaging
            pyelftools
            pykwalify
            pyserial
            pyyaml
            requests
            semver
            tqdm
            west
            rumocaPythonPackage
            synapseFbsPythonPackage
          ]
        );
      mkFlightPythonEnv =
        pkgs: rumocaPythonPackage:
        pkgs.python3.withPackages (
          ps: with ps; [
            matplotlib
            numpy
            rumocaPythonPackage
          ]
        );
      mkNativeSimSilPythonEnv =
        pkgs: rumocaPythonPackage: synapseFbsPythonPackage:
        pkgs.python3.withPackages (
          ps: with ps; [
            flatbuffers
            matplotlib
            numpy
            rumocaPythonPackage
            synapseFbsPythonPackage
            zenoh
          ]
        );
    in
    {
      packages = forAllSystems (
        system:
        let
          pkgs = pkgsFor system;
          rumocaPythonPackage = mkRumocaPythonPackage pkgs;
          synapseFbsPythonPackage = mkSynapseFbsPythonPackage pkgs;
          pythonEnv = mkPythonEnv pkgs rumocaPythonPackage synapseFbsPythonPackage;
          flightPythonEnv = mkFlightPythonEnv pkgs rumocaPythonPackage;
          nativeSimSilPythonEnv = mkNativeSimSilPythonEnv pkgs rumocaPythonPackage synapseFbsPythonPackage;
          hostCc = if system == "x86_64-linux" then pkgs.gcc_multi else pkgs.stdenv.cc;
          hostMultilibTools = lib.optionals (system == "x86_64-linux") [
            pkgs.glibc_multi.dev
          ];

          baseTools = [
            pkgs.ccache
            pkgs.cmake
            pkgs.coreutils
            pkgs.curl
            pkgs.dtc
            pkgs.file
            pkgs.findutils
            pkgs.gcc-arm-embedded
            pkgs.git
            pkgs.gitRepo
            pkgs.gnumake
            pkgs.gnugrep
            pkgs.gnused
            pkgs.gperf
            pkgs.ncurses
            pkgs.ninja
            pkgs.openocd
            pkgs.openssh
            pkgs.picocom
            pkgs.pkg-config
            pkgs.python3Packages.pyocd
            pkgs.zenoh
            hostCc
            pkgs.unzip
            pkgs.which
            pkgs.xz
            pkgs.zip
            pythonEnv
          ] ++ hostMultilibTools;

          commonScript = ''
            zephyr_app_find_app() {
              local dir
              dir="$(pwd -P)"

              while [ "$dir" != "/" ]; do
                if [ -f "$dir/west.yml" ] && [ -f "$dir/prj.conf" ] && [ -f "$dir/CMakeLists.txt" ]; then
                  printf '%s\n' "$dir"
                  return 0
                fi

                if [ -f "$dir/${appDirectory}/west.yml" ] && [ -f "$dir/${appDirectory}/prj.conf" ]; then
                  printf '%s\n' "$dir/${appDirectory}"
                  return 0
                fi

                dir="$(dirname "$dir")"
              done

              printf 'error: could not find the ${appDirectory} app from %s\n' "$(pwd -P)" >&2
              printf 'run this command from the app directory or its west workspace root\n' >&2
              return 1
            }

            zephyr_app_source_workspace() {
              dirname "$1"
            }

            zephyr_app_active_manifest() {
              local dir="$1"
              (cd "$dir" && env -u ZEPHYR_BASE -u WEST_TOPDIR west manifest --path 2>/dev/null || true)
            }

            zephyr_app_managed_workspace() {
              local app="$1"
              local cache_root
              local key

              if [ -n "''${CUBS2_WEST_WORKSPACE:-}" ]; then
                realpath -m "$CUBS2_WEST_WORKSPACE"
                return 0
              fi

              cache_root="''${XDG_CACHE_HOME:-$HOME/.cache}/${appCacheName}"
              key="$(printf '%s' "$app" | sha256sum | cut -c1-16)"
              printf '%s\n' "$cache_root/west-$key"
            }

            zephyr_app_workspace() {
              local app="$1"
              local source_workspace
              local expected_manifest
              local actual_manifest

              if [ -n "''${CUBS2_WORKSPACE_ROOT:-}" ]; then
                realpath -m "$CUBS2_WORKSPACE_ROOT"
                return 0
              fi

              source_workspace="$(zephyr_app_source_workspace "$app")"
              expected_manifest="$(realpath "$app/west.yml")"
              actual_manifest="$(zephyr_app_active_manifest "$app")"

              if [ -n "$actual_manifest" ]; then
                actual_manifest="$(realpath "$actual_manifest")"
              fi

              if [ -z "$actual_manifest" ] ||
                 [ "$actual_manifest" = "$expected_manifest" ] ||
                 [ "''${CUBS2_ALLOW_FOREIGN_WEST:-0}" = "1" ]; then
                printf '%s\n' "$source_workspace"
              else
                zephyr_app_managed_workspace "$app"
              fi
            }

            zephyr_app_prepare_managed_workspace() {
              local app="$1"
              local workspace="$2"
              local manifest_dir="$workspace/manifest"
              local manifest_path="$manifest_dir/west.yml"
              local actual_manifest

              mkdir -p "$manifest_dir"
              if [ ! -d "$manifest_dir/.git" ]; then
                git init -q "$manifest_dir"
              fi

              cp "$app/west.yml" "$manifest_path"
              (
                cd "$manifest_dir"
                git add west.yml
                if ! git rev-parse --verify HEAD >/dev/null 2>&1 ||
                   ! git diff --cached --quiet; then
                  git -c user.name='cerebri-cubs2 nix' \
                      -c user.email='cerebri-cubs2-nix@example.invalid' \
                      commit -q -m 'Update cerebri_cubs2 manifest'
                fi
              )

              if [ ! -d "$workspace/.west" ]; then
                (cd "$workspace" && env -u ZEPHYR_BASE -u WEST_TOPDIR west init -l manifest)
              fi

              actual_manifest="$(zephyr_app_active_manifest "$workspace")"
              if [ -z "$actual_manifest" ]; then
                printf 'error: could not read the managed west manifest in %s\n' "$workspace" >&2
                return 1
              fi
              actual_manifest="$(realpath "$actual_manifest")"
              if [ "$actual_manifest" != "$(realpath "$manifest_path")" ]; then
                printf 'error: managed workspace %s uses unexpected manifest %s\n' "$workspace" "$actual_manifest" >&2
                printf '       expected %s\n' "$manifest_path" >&2
                return 1
              fi
            }

            zephyr_app_require_module_paths() {
              local workspace="$1"
              local missing=0
              local path

              for path in \
                zephyr \
                modules/hal/cmsis \
                modules/hal/cmsis_6 \
                modules/hal/nxp \
                modules/lib/zenoh-pico \
                modules/lib/cerebri_lockstep \
                modules/lib/zros \
                modules/lib/csyn \
                modules/lib/zephyr_boards \
                models/vendor/CMM-v0.0.2
              do
                if [ ! -d "$workspace/$path" ]; then
                  printf 'error: missing required west checkout: %s/%s\n' "$workspace" "$path" >&2
                  missing=1
                fi
              done

              if [ "$missing" -ne 0 ]; then
                printf 'run from the app checkout: nix run .#west-update\n' >&2
                return 1
              fi
            }

            zephyr_app_export_common() {
              local app="$1"
              local workspace
              workspace="$(zephyr_app_workspace "$app")"

              export WEST_PYTHON="''${WEST_PYTHON:-${pythonEnv}/bin/python}"
              export CUBS2_RUMOCA_PYTHON="''${CUBS2_RUMOCA_PYTHON:-${pythonEnv}/bin/python}"
              export GNUARMEMB_TOOLCHAIN_PATH="''${GNUARMEMB_TOOLCHAIN_PATH:-${pkgs.gcc-arm-embedded}}"
              export CUBS2_WORKSPACE_ROOT="$workspace"

              if [ -d "$workspace/zephyr" ]; then
                export ZEPHYR_BASE="$workspace/zephyr"
              fi
            }

            zephyr_app_require_workspace() {
              local app="$1"
              local workspace
              local source_workspace
              local expected_manifest
              local actual_manifest
              workspace="$(zephyr_app_workspace "$app")"
              source_workspace="$(zephyr_app_source_workspace "$app")"
              expected_manifest="$(realpath "$app/west.yml")"

              if [ ! -d "$workspace/.west" ]; then
                printf 'error: missing west workspace metadata at %s/.west\n' "$workspace" >&2
                printf 'initialize/update it with: nix run .#west-update\n' >&2
                return 1
              fi

              actual_manifest="$(zephyr_app_active_manifest "$workspace")"
              if [ -z "$actual_manifest" ]; then
                printf 'error: could not read the active west manifest in %s\n' "$workspace" >&2
                return 1
              fi
              actual_manifest="$(realpath "$actual_manifest")"

              if [ "$workspace" = "$source_workspace" ] &&
                 [ "$actual_manifest" != "$expected_manifest" ] &&
                 [ "''${CUBS2_ALLOW_FOREIGN_WEST:-0}" != "1" ]; then
                printf 'error: active west manifest is %s\n' "$actual_manifest" >&2
                printf '       ${appDirectory} expects %s\n' "$expected_manifest" >&2
                printf 'run nix run .#west-update to create/update the managed ${appDisplayName} workspace\n' >&2
                return 1
              fi

              if [ -z "''${ZEPHYR_BASE:-}" ] || [ ! -d "$ZEPHYR_BASE" ]; then
                printf 'error: missing Zephyr checkout; expected %s/zephyr or ZEPHYR_BASE\n' "$workspace" >&2
                printf 'run from the app checkout: nix run .#west-update\n' >&2
                return 1
              fi

              zephyr_app_require_module_paths "$workspace"
            }

            zephyr_app_run_logged() {
              local log_file="$1"
              shift

              if [ "''${CUBS2_QUIET_LOGS:-0}" != "1" ]; then
                "$@"
                return $?
              fi

              mkdir -p "$(dirname "$log_file")"
              "$@" >"$log_file" 2>&1
              local rc=$?
              if [ "$rc" -eq 0 ]; then
                printf 'wrote log: %s\n' "$log_file"
                return 0
              fi

              printf 'command failed with status %d: %s\n' "$rc" "$*" >&2
              printf 'last %s lines from %s:\n' "''${CUBS2_LOG_TAIL_LINES:-200}" "$log_file" >&2
              tail -n "''${CUBS2_LOG_TAIL_LINES:-200}" "$log_file" >&2 || true
              return "$rc"
            }
          '';

          mkWestApp =
            name: extraInputs: text:
            pkgs.writeShellApplication {
              inherit name;
              runtimeInputs = baseTools ++ extraInputs;
              inherit text;
            };

          cubs2-build = mkWestApp "cubs2-build" [ ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            zephyr_app_require_workspace "$app"
            workspace="$CUBS2_WORKSPACE_ROOT"

            export ZEPHYR_TOOLCHAIN_VARIANT="''${ZEPHYR_TOOLCHAIN_VARIANT:-gnuarmemb}"

            board="''${CUBS2_BOARD:-${defaultBoard}}"
            board_slug="''${board//\//_}"
            build_dir="''${CUBS2_BUILD_DIR:-$app/build-$board_slug}"

            cd "$workspace"
            zephyr_app_run_logged "$build_dir/west-build.log" \
              west build -b "$board" -d "$build_dir" "$app" "$@"
          '';

          cubs2-build-native-sim = mkWestApp "cubs2-build-native-sim" [ ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            zephyr_app_require_workspace "$app"
            workspace="$CUBS2_WORKSPACE_ROOT"

            export ZEPHYR_TOOLCHAIN_VARIANT="''${ZEPHYR_TOOLCHAIN_VARIANT:-host}"

            board="''${CUBS2_NATIVE_SIM_BOARD:-${defaultNativeSimBoard}}"
            board_slug="''${board//\//_}"
            build_dir="''${CUBS2_NATIVE_SIM_BUILD_DIR:-$app/build-$board_slug}"

            cd "$workspace"
            zephyr_app_run_logged "$build_dir/west-build.log" \
              west build -b "$board" -d "$build_dir" "$app" "$@"
          '';

          cubs2-build-native-sim-64 = mkWestApp "cubs2-build-native-sim-64" [ ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            zephyr_app_require_workspace "$app"
            workspace="$CUBS2_WORKSPACE_ROOT"

            export ZEPHYR_TOOLCHAIN_VARIANT="''${ZEPHYR_TOOLCHAIN_VARIANT:-host}"

            board="''${CUBS2_NATIVE_SIM_64_BOARD:-${defaultNativeSim64Board}}"
            board_slug="''${board//\//_}"
            build_dir="''${CUBS2_NATIVE_SIM_64_BUILD_DIR:-$app/build-$board_slug}"

            cd "$workspace"
            zephyr_app_run_logged "$build_dir/west-build.log" \
              west build -b "$board" -d "$build_dir" "$app" "$@"
          '';

          cubs2-flash = mkWestApp "cubs2-flash" [ ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            zephyr_app_require_workspace "$app"
            workspace="$CUBS2_WORKSPACE_ROOT"

            export ZEPHYR_TOOLCHAIN_VARIANT="''${ZEPHYR_TOOLCHAIN_VARIANT:-gnuarmemb}"

            board="''${CUBS2_BOARD:-${defaultBoard}}"
            board_slug="''${board//\//_}"
            build_dir="''${CUBS2_BUILD_DIR:-$app/build-$board_slug}"
            runner="''${CUBS2_FLASH_RUNNER:-pyocd}"
            runner_args=()

            if [ -n "$runner" ]; then
              runner_args=(--runner "$runner")
            fi

            cd "$workspace"
            exec west flash -d "$build_dir" "''${runner_args[@]}" "$@"
          '';

          cubs2-menuconfig = mkWestApp "cubs2-menuconfig" [ ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            zephyr_app_require_workspace "$app"
            workspace="$CUBS2_WORKSPACE_ROOT"

            export ZEPHYR_TOOLCHAIN_VARIANT="''${ZEPHYR_TOOLCHAIN_VARIANT:-gnuarmemb}"

            board="''${CUBS2_BOARD:-${defaultBoard}}"
            board_slug="''${board//\//_}"
            build_dir="''${CUBS2_BUILD_DIR:-$app/build-$board_slug}"

            cd "$workspace"
            exec west build -b "$board" -d "$build_dir" -t menuconfig "$app" "$@"
          '';

          cubs2-west-update = mkWestApp "cubs2-west-update" [ ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            workspace="$(zephyr_app_workspace "$app")"
            source_workspace="$(zephyr_app_source_workspace "$app")"
            expected_manifest="$(realpath "$app/west.yml")"

            if [ "$workspace" != "$source_workspace" ]; then
              printf 'using managed ${appDisplayName} west workspace: %s\n' "$workspace" >&2
              zephyr_app_prepare_managed_workspace "$app" "$workspace"
              cd "$workspace"
              exec env -u ZEPHYR_BASE -u WEST_TOPDIR west update "$@"
            fi

            if [ ! -d "$source_workspace/.west" ]; then
              cd "$source_workspace"
              env -u ZEPHYR_BASE -u WEST_TOPDIR west init -l "$app"
            else
              actual_manifest="$(zephyr_app_active_manifest "$source_workspace")"
              if [ -z "$actual_manifest" ]; then
                printf 'error: could not read the active west manifest in %s\n' "$source_workspace" >&2
                exit 1
              fi
              actual_manifest="$(realpath "$actual_manifest")"
              if [ "$actual_manifest" != "$expected_manifest" ]; then
                printf 'error: refusing to update source workspace with foreign manifest %s\n' "$actual_manifest" >&2
                printf '       expected %s\n' "$expected_manifest" >&2
                printf 'unset CUBS2_WORKSPACE_ROOT/CUBS2_ALLOW_FOREIGN_WEST to use the managed workspace fallback\n' >&2
                exit 1
              fi
            fi

            cd "$source_workspace"
            exec env -u ZEPHYR_BASE -u WEST_TOPDIR west update "$@"
          '';

          rumoca-check = pkgs.writeShellApplication {
            name = "rumoca-check";
            runtimeInputs = [
              pkgs.coreutils
              pythonEnv
            ];
            text = ''
              model="''${1:-src/FixedWingOuterLoop.mo}"

              if [ ! -f "$model" ]; then
                printf 'error: Modelica file not found: %s\n' "$model" >&2
                exit 1
              fi

              exec "${pythonEnv}/bin/python" - "$model" <<'PY'
              from pathlib import Path
              import sys

              import rumoca as rum

              model = Path(sys.argv[1])
              source = model.read_text(encoding="utf-8")
              formatted = rum.format(source, filename=str(model))
              if formatted != source:
                  print(f"error: Modelica formatting differs: {model}", file=sys.stderr)
                  raise SystemExit(1)

              try:
                  rum.Session().load(str(model), model=model.stem)
              except rum.RumocaError as exc:
                  print(f"{model}: error: {exc}", file=sys.stderr)
                  raise SystemExit(1) from exc
              PY
            '';
          };

          cubs2-flight-sil-test = mkWestApp "cubs2-flight-sil-test" [ flightPythonEnv ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            zephyr_app_require_workspace "$app"
            cd "$app"
            exec "${flightPythonEnv}/bin/python" tests/flight/run_cubs2_flight_sil.py "$@"
          '';

          cubs2-native-sim-sil-test = pkgs.writeShellApplication {
            name = "cubs2-native-sim-sil-test";
            runtimeInputs = [
              pkgs.coreutils
              cubs2-build-native-sim
              cubs2-native-sim-sil-run
            ];
            text = ''
              ${commonScript}

              app="$(zephyr_app_find_app)"
              zephyr_app_export_common "$app"
              zephyr_app_require_workspace "$app"
              "${cubs2-build-native-sim}/bin/cubs2-build-native-sim"
              board="''${CUBS2_NATIVE_SIM_BOARD:-${defaultNativeSimBoard}}"
              board_slug="''${board//\//_}"
              sim="''${CUBS2_NATIVE_SIM_BUILD_DIR:-$app/build-$board_slug}/zephyr/zephyr.exe"

              cd "$app"
              exec "${cubs2-native-sim-sil-run}/bin/cubs2-native-sim-sil-run" \
                --sim "$sim" \
                "$@"
            '';
          };

          cubs2-native-sim-sil-run = pkgs.writeShellApplication {
            name = "cubs2-native-sim-sil-run";
            runtimeInputs = [
              pkgs.coreutils
              pkgs.stdenv.cc
              pkgs.zip
              pkgs.zenoh
              nativeSimSilPythonEnv
            ];
            text = ''
              find_app() {
                local dir
                dir="$(pwd -P)"

                while [ "$dir" != "/" ]; do
                  if [ -f "$dir/west.yml" ] && [ -f "$dir/prj.conf" ] && [ -f "$dir/CMakeLists.txt" ]; then
                    printf '%s\n' "$dir"
                    return 0
                  fi

                  if [ -f "$dir/${appDirectory}/west.yml" ] && [ -f "$dir/${appDirectory}/prj.conf" ]; then
                    printf '%s\n' "$dir/${appDirectory}"
                    return 0
                  fi

                  dir="$(dirname "$dir")"
                done

                printf 'error: could not find the ${appDirectory} app from %s\n' "$(pwd -P)" >&2
                return 1
              }

              app="$(find_app)"
              cd "$app"
              exec "${nativeSimSilPythonEnv}/bin/python" \
                tests/zephyr/run_native_sim_zenoh_sil.py "$@"
            '';
          };

          csyn = mkWestApp "csyn" [
            pkgs.cargo
            pkgs.pkg-config
            pkgs.rustc
          ] ''
            ${commonScript}

            app="$(zephyr_app_find_app)"
            zephyr_app_export_common "$app"
            workspace="$CUBS2_WORKSPACE_ROOT"
            manifest="$workspace/modules/lib/csyn/rust/Cargo.toml"

            if [ ! -f "$manifest" ]; then
              printf 'error: csyn Rust CLI not found at %s\n' "$manifest" >&2
              printf 'run from the app checkout: nix run .#west-update\n' >&2
              exit 1
            fi

            exec cargo run --quiet --manifest-path "$manifest" -- "$@"
          '';

          cubs2-native-sim-64-sil-run = pkgs.writeShellApplication {
            name = "cubs2-native-sim-64-sil-run";
            runtimeInputs = [
              pkgs.coreutils
              cubs2-native-sim-sil-run
            ];
            text = ''
              ${commonScript}

              app="$(zephyr_app_find_app)"
              board="''${CUBS2_NATIVE_SIM_64_BOARD:-${defaultNativeSim64Board}}"
              board_slug="''${board//\//_}"
              sim="''${CUBS2_NATIVE_SIM_64_BUILD_DIR:-$app/build-$board_slug}/zephyr/zephyr.exe"
              artifacts="''${CUBS2_NATIVE_SIM_64_ARTIFACTS:-artifacts/native-sim-64-sil}"

              cd "$app"
              exec "${cubs2-native-sim-sil-run}/bin/cubs2-native-sim-sil-run" \
                --sim "$sim" \
                --artifacts "$artifacts" \
                "$@"
            '';
          };

          cubs2-native-sim-64-sil-test = pkgs.writeShellApplication {
            name = "cubs2-native-sim-64-sil-test";
            runtimeInputs = [
              pkgs.coreutils
              cubs2-build-native-sim-64
              cubs2-native-sim-64-sil-run
            ];
            text = ''
              ${commonScript}

              app="$(zephyr_app_find_app)"
              zephyr_app_export_common "$app"
              zephyr_app_require_workspace "$app"
              "${cubs2-build-native-sim-64}/bin/cubs2-build-native-sim-64"

              cd "$app"
              exec "${cubs2-native-sim-64-sil-run}/bin/cubs2-native-sim-64-sil-run" "$@"
            '';
          };

          host-tools = pkgs.buildEnv {
            name = "cerebri-cubs2-host-tools";
            paths = baseTools ++ [
              cubs2-build
              cubs2-build-native-sim
              cubs2-build-native-sim-64
              cubs2-flash
              cubs2-menuconfig
              cubs2-west-update
              cubs2-flight-sil-test
              cubs2-native-sim-sil-test
              cubs2-native-sim-sil-run
              cubs2-native-sim-64-sil-test
              cubs2-native-sim-64-sil-run
              csyn
              rumoca-check
            ];
          };
        in
        {
          inherit
            host-tools
            rumocaPythonPackage
            synapseFbsPythonPackage
            cubs2-build
            cubs2-build-native-sim
            cubs2-build-native-sim-64
            cubs2-flash
            cubs2-menuconfig
            cubs2-west-update
            cubs2-flight-sil-test
            cubs2-native-sim-sil-test
            cubs2-native-sim-sil-run
            cubs2-native-sim-64-sil-test
            cubs2-native-sim-64-sil-run
            csyn
            rumoca-check
            ;

          rumoca = rumocaPythonPackage;
          synapse-fbs = synapseFbsPythonPackage;
          default = host-tools;
        }
      );

      apps = forAllSystems (
        system:
        let
          packages = self.packages.${system};
        in
        {
          build = {
            type = "app";
            program = "${packages.cubs2-build}/bin/cubs2-build";
            meta.description = "Build ${appDisplayName} firmware for ${defaultBoard}";
          };

          build-native-sim = {
            type = "app";
            program = "${packages.cubs2-build-native-sim}/bin/cubs2-build-native-sim";
            meta.description = "Build ${appDisplayName} for ${defaultNativeSimBoard}";
          };

          build-native-sim-64 = {
            type = "app";
            program = "${packages.cubs2-build-native-sim-64}/bin/cubs2-build-native-sim-64";
            meta.description = "Build ${appDisplayName} for ${defaultNativeSim64Board}";
          };

          flash = {
            type = "app";
            program = "${packages.cubs2-flash}/bin/cubs2-flash";
            meta.description = "Flash the ${appDisplayName} firmware build";
          };

          menuconfig = {
            type = "app";
            program = "${packages.cubs2-menuconfig}/bin/cubs2-menuconfig";
            meta.description = "Run Zephyr menuconfig for ${appDisplayName}";
          };

          west-update = {
            type = "app";
            program = "${packages.cubs2-west-update}/bin/cubs2-west-update";
            meta.description = "Initialize or update the ${appDisplayName} west workspace";
          };

          rumoca-check = {
            type = "app";
            program = "${packages.rumoca-check}/bin/rumoca-check";
            meta.description = "Run Rumoca formatting and lint checks";
          };

          flight-sil-test = {
            type = "app";
            program = "${packages.cubs2-flight-sil-test}/bin/cubs2-flight-sil-test";
            meta.description = "Run staged CUBS2 flight SIL checks and generate a track plot";
          };

          native-sim-sil-test = {
            type = "app";
            program = "${packages.cubs2-native-sim-sil-test}/bin/cubs2-native-sim-sil-test";
            meta.description = "Run CUBS2 Zephyr native_sim SIL through Zenoh topics";
          };

          native-sim-sil-run = {
            type = "app";
            program = "${packages.cubs2-native-sim-sil-run}/bin/cubs2-native-sim-sil-run";
            meta.description = "Run CUBS2 Zephyr native_sim SIL against an existing native_sim executable";
          };

          native-sim-64-sil-test = {
            type = "app";
            program = "${packages.cubs2-native-sim-64-sil-test}/bin/cubs2-native-sim-64-sil-test";
            meta.description = "Run CUBS2 Zephyr native_sim/native/64 SIL through Zenoh topics";
          };

          native-sim-64-sil-run = {
            type = "app";
            program = "${packages.cubs2-native-sim-64-sil-run}/bin/cubs2-native-sim-64-sil-run";
            meta.description = "Run CUBS2 Zephyr native_sim/native/64 SIL against an existing executable";
          };

          csyn = {
            type = "app";
            program = "${packages.csyn}/bin/csyn";
            meta.description = "Run the csyn Synapse Zenoh CLI from the CUBS2 west workspace";
          };
        }
      );

      devShells = forAllSystems (
        system:
        let
          pkgs = pkgsFor system;
          rumocaPythonPackage = self.packages.${system}.rumocaPythonPackage;
          synapseFbsPythonPackage = self.packages.${system}.synapseFbsPythonPackage;
          pythonEnv = mkPythonEnv pkgs rumocaPythonPackage synapseFbsPythonPackage;
          packages = self.packages.${system};
        in
        {
          default = pkgs.mkShell {
            packages = [ packages.host-tools ];

            # The native_simulator runner compiles at -O0 (NSI_OPT default);
            # nixpkgs' fortify hardening injects _FORTIFY_SOURCE, which errors
            # without optimization, breaking fresh native_sim builds.
            hardeningDisable = [ "fortify" ];

            shellHook = ''
              export WEST_PYTHON="''${WEST_PYTHON:-${pythonEnv}/bin/python}"
              export CUBS2_RUMOCA_PYTHON="''${CUBS2_RUMOCA_PYTHON:-${pythonEnv}/bin/python}"
              export GNUARMEMB_TOOLCHAIN_PATH="''${GNUARMEMB_TOOLCHAIN_PATH:-${pkgs.gcc-arm-embedded}}"
              export ZEPHYR_TOOLCHAIN_VARIANT="''${ZEPHYR_TOOLCHAIN_VARIANT:-gnuarmemb}"

              zephyr_app_shell_find_app() {
                local dir
                dir="$(pwd -P)"

                while [ "$dir" != "/" ]; do
                  if [ -f "$dir/west.yml" ] && [ -f "$dir/prj.conf" ] && [ -f "$dir/CMakeLists.txt" ]; then
                    printf '%s\n' "$dir"
                    return 0
                  fi

                  if [ -f "$dir/${appDirectory}/west.yml" ] && [ -f "$dir/${appDirectory}/prj.conf" ]; then
                    printf '%s\n' "$dir/${appDirectory}"
                    return 0
                  fi

                  dir="$(dirname "$dir")"
                done

                return 1
              }

              if app="$(zephyr_app_shell_find_app 2>/dev/null)"; then
                source_workspace="$(dirname "$app")"
                expected_manifest="$(realpath "$app/west.yml")"
                actual_manifest="$(env -u ZEPHYR_BASE -u WEST_TOPDIR west manifest --path 2>/dev/null || true)"

                if [ -n "$actual_manifest" ]; then
                  actual_manifest="$(realpath "$actual_manifest")"
                fi

                if [ -n "''${CUBS2_WORKSPACE_ROOT:-}" ]; then
                  workspace="$(realpath -m "$CUBS2_WORKSPACE_ROOT")"
                elif [ -z "$actual_manifest" ] ||
                     [ "$actual_manifest" = "$expected_manifest" ] ||
                     [ "''${CUBS2_ALLOW_FOREIGN_WEST:-0}" = "1" ]; then
                  workspace="$source_workspace"
                elif [ -n "''${CUBS2_WEST_WORKSPACE:-}" ]; then
                  workspace="$(realpath -m "$CUBS2_WEST_WORKSPACE")"
                else
                  key="$(printf '%s' "$app" | sha256sum | cut -c1-16)"
                  workspace="''${XDG_CACHE_HOME:-$HOME/.cache}/${appCacheName}/west-$key"
                fi

                export CUBS2_WORKSPACE_ROOT="$workspace"
                if [ -d "$workspace/zephyr" ]; then
                  export ZEPHYR_BASE="$workspace/zephyr"
                elif [ -z "''${ZEPHYR_BASE:-}" ]; then
                  printf '${appDisplayName} Nix shell: run cubs2-west-update before raw west builds\n' >&2
                fi
              elif [ -z "''${ZEPHYR_BASE:-}" ] && [ -d "$PWD/zephyr" ]; then
                export ZEPHYR_BASE="$PWD/zephyr"
              fi

              echo "${appDisplayName} Nix shell: cubs2-west-update, cubs2-build, cubs2-build-native-sim, cubs2-build-native-sim-64, cubs2-flight-sil-test, cubs2-native-sim-sil-test, cubs2-native-sim-64-sil-test, cubs2-native-sim-sil-run, cubs2-native-sim-64-sil-run, csyn, cubs2-flash"
            '';
          };
        }
      );

      nixosModules.default =
        {
          config,
          lib,
          pkgs,
          ...
        }:
        let
          cfg = config.programs.cerebri-cubs2;
          system = pkgs.stdenv.hostPlatform.system;
          hostTools = self.packages.${system}.host-tools;
        in
        {
          options.programs.cerebri-cubs2 = {
            enable = lib.mkEnableOption "host tools and device access for cerebri_cubs2 development";

            users = lib.mkOption {
              type = lib.types.listOf lib.types.str;
              default = [ ];
              example = [ "alice" ];
              description = "Local users that should be allowed to access serial ports and debug probes.";
            };

            probeGroup = lib.mkOption {
              type = lib.types.str;
              default = "dialout";
              description = "Group assigned to supported USB debug probes.";
            };

            enableUdevRules = lib.mkOption {
              type = lib.types.bool;
              default = true;
              description = "Install udev rules for common MIMXRT1064 debug and serial adapters.";
            };

            enableNixSettings = lib.mkOption {
              type = lib.types.bool;
              default = true;
              description = "Enable the Nix CLI features required by this flake and select a recent Nix package by default.";
            };

            nixPackage = lib.mkOption {
              type = lib.types.package;
              default = pkgs.nixVersions.latest;
              defaultText = lib.literalExpression "pkgs.nixVersions.latest";
              description = "Nix package to use when enableNixSettings is true.";
            };
          };

          config = lib.mkIf cfg.enable {
            nix = lib.mkIf cfg.enableNixSettings {
              package = lib.mkDefault cfg.nixPackage;
              settings.experimental-features = [
                "nix-command"
                "flakes"
              ];
            };

            environment.systemPackages = [ hostTools ];

            users.groups.${cfg.probeGroup} = { };

            users.users = lib.genAttrs cfg.users (_: {
              extraGroups = [
                cfg.probeGroup
                "dialout"
              ];
            });

            services.udev.extraRules = lib.mkIf cfg.enableUdevRules ''
              # SEGGER J-Link
              SUBSYSTEM=="usb", ATTR{idVendor}=="1366", MODE="0660", GROUP="${cfg.probeGroup}", TAG+="uaccess"

              # ARM/DAPLink CMSIS-DAP probes
              SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", MODE="0660", GROUP="${cfg.probeGroup}", TAG+="uaccess"

              # NXP ROM bootloader and MCU-Link family
              SUBSYSTEM=="usb", ATTR{idVendor}=="1fc9", MODE="0660", GROUP="${cfg.probeGroup}", TAG+="uaccess"
              SUBSYSTEM=="usb", ATTR{idVendor}=="15a2", MODE="0660", GROUP="${cfg.probeGroup}", TAG+="uaccess"

              # Common USB serial adapters used during bench bring-up
              SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0660", GROUP="dialout", TAG+="uaccess"
              SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0660", GROUP="dialout", TAG+="uaccess"
              SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0660", GROUP="dialout", TAG+="uaccess"
            '';
          };
        };
      nixosModules.cerebri-cubs2 = self.nixosModules.default;
    };
}
