{ self }:
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
}
