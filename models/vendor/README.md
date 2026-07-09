# Vendored Modelica Libraries

This directory documents Modelica source dependencies used by CI simulation
tests. The dependencies themselves are populated by west from `west.yml`; run
`nix run .#west-update` before local simulation tests.

## CMM-v0.0.2

- West project: `modelica_models`
- Source: https://github.com/CogniPilot/modelica_models
- Revision: `62ea9f97cec28e092c8e67c9f4a0dbb842f6233b` (`v0.0.2`)
- Path: `<west workspace>/models/vendor/CMM-v0.0.2`
- License: Apache-2.0, included in the west checkout.

## Rumoca FixedWingSIL

- Source: https://github.com/CogniPilot/rumoca/blob/main/examples/interactive/fixedwing/FixedWingSIL.mo
- Local copy: `../plant/FixedWingSIL.mo`
