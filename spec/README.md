# CUBS2 Architecture Notes

CUBS2 follows the CEREBRI Zephyr app layout, with a deliberately small
contract:

- `subsys/csyn` stores latest Zenoh/csyn payloads.
- `src/csyn_zros_bridge.c` is the only csyn/zros boundary.
- `src/main.c` consumes zros manual-control and mocap topics.
- `src/main.c` publishes zros control output and flight snapshot topics.
- csyn republishes control output and flight snapshot over Zenoh.
- `flight_snapshot` is a fixed-layout struct payload for controller and
  waypoint diagnostics.

Local RC, sensor, actuator, storage, DTS, driver, schema, include, and host-tool
directories are outside the CUBS2 app scope.
