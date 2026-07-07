#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Smoke-test the Zephyr native_sim app through the csyn Zenoh transport."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import struct
import subprocess
import sys
import time
from typing import Iterable

import zenoh


MANUAL_TOPIC = "synapse/v1/topic/manual_control_command"
MOCAP_TOPIC = "synapse/v1/topic/mocap_frame"
PWM_TOPIC = "synapse/v1/topic/pwm_signal_outputs"

MANUAL_AXES_REQUIRED = 0x000F
MANUAL_FLAG_ARM_SWITCH = 0x01
MANUAL_FLAG_VALID = 0x08
MANUAL_FLIGHT_MODE_AUTO = 1

MANUAL_STRUCT = struct.Struct("<QI H 10h BB 4x")
MOCAP_SHORTCUT_STRUCT = struct.Struct("<7f")
PWM_STRUCT = struct.Struct("<QIBx16H2x")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sim",
        default="build-native_sim/zephyr/zephyr.exe",
        help="native_sim executable to launch",
    )
    parser.add_argument(
        "--artifacts",
        default="artifacts/native-sim-sil",
        help="directory for logs, CSV, and summary artifacts",
    )
    parser.add_argument(
        "--locator",
        default="udp/127.0.0.1:7447",
        help="Zenoh router locator; must match CONFIG_CSYN_ZENOH_LOCATOR",
    )
    parser.add_argument("--duration-s", type=float, default=6.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--startup-timeout-s", type=float, default=4.0)
    return parser.parse_args()


def tail(path: Path, line_count: int = 80) -> str:
    if not path.exists():
        return f"{path} does not exist"

    lines = path.read_text(errors="replace").splitlines()
    return "\n".join(lines[-line_count:])


def start_process(name: str, argv: list[str], log_path: Path) -> subprocess.Popen[bytes]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("wb")
    try:
        process = subprocess.Popen(
            argv,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except Exception:
        log.close()
        raise

    process._cubs2_log = log  # type: ignore[attr-defined]
    process._cubs2_name = name  # type: ignore[attr-defined]
    process._cubs2_log_path = log_path  # type: ignore[attr-defined]
    return process


def stop_process(process: subprocess.Popen[bytes] | None) -> None:
    if process is None:
        return

    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)

    log = getattr(process, "_cubs2_log", None)
    if log is not None:
        log.close()


def require_running(process: subprocess.Popen[bytes], log_path: Path, name: str) -> None:
    rc = process.poll()
    if rc is None:
        return

    raise RuntimeError(f"{name} exited early with status {rc}\n\n{tail(log_path)}")


def make_zenoh_config(locator: str) -> zenoh.Config:
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", f'["{locator}"]')
    return config


def open_zenoh_session(locator: str, timeout_s: float) -> zenoh.Session:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None

    while time.monotonic() < deadline:
        try:
            return zenoh.open(make_zenoh_config(locator))
        except Exception as exc:  # pragma: no cover - exercised by CI environment.
            last_error = exc
            time.sleep(0.1)

    raise RuntimeError(f"could not open Zenoh session to {locator}: {last_error}")


def pack_manual_control(timestamp_us: int) -> bytes:
    neutral_axes = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    flags = MANUAL_FLAG_ARM_SWITCH | MANUAL_FLAG_VALID

    return MANUAL_STRUCT.pack(
        timestamp_us,
        0,
        MANUAL_AXES_REQUIRED,
        *neutral_axes,
        MANUAL_FLIGHT_MODE_AUTO,
        flags,
    )


def pack_mocap(timestamp_s: float) -> bytes:
    # Keep the vehicle airborne and near the first route segment. The 7-float
    # mocap shortcut is decoded by csyn into a valid rigid body sample.
    x_m = -0.25 * timestamp_s
    y_m = -0.2 * timestamp_s
    z_m = 1.0

    # csyn's shortcut maps raw quaternion fields to the Synapse quaternion as:
    # qw=-qy_raw, qx=-qz_raw, qy=qw_raw, qz=qx_raw. This encodes identity.
    return MOCAP_SHORTCUT_STRUCT.pack(x_m, y_m, z_m, 0.0, 0.0, -1.0, 0.0)


def decode_pwm(payload: bytes, monotonic_s: float) -> dict[str, int | float]:
    if len(payload) != PWM_STRUCT.size:
        raise ValueError(f"expected {PWM_STRUCT.size} PWM bytes, got {len(payload)}")

    timestamp_us, active_mask, port, *outputs = PWM_STRUCT.unpack(payload)
    row: dict[str, int | float] = {
        "monotonic_s": monotonic_s,
        "timestamp_us": timestamp_us,
        "active_mask": active_mask,
        "port": port,
    }
    row.update({f"output{i}_us": value for i, value in enumerate(outputs)})
    return row


def drain_pwm(subscriber: object, rows: list[dict[str, int | float]]) -> None:
    while True:
        sample = subscriber.try_recv()
        if sample is None:
            return
        rows.append(decode_pwm(bytes(sample.payload), time.monotonic()))


def write_csv(path: Path, rows: Iterable[dict[str, int | float]]) -> None:
    fieldnames = [
        "monotonic_s",
        "timestamp_us",
        "active_mask",
        "port",
        *[f"output{i}_us" for i in range(16)],
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(
    path: Path, locator: str, sim: Path, rows: list[dict[str, int | float]]
) -> None:
    max_speed_signal = max((int(row["output6_us"]) for row in rows), default=0)
    max_throttle = max((int(row["output2_us"]) for row in rows), default=0)
    max_bank_signal = max((abs(int(row["output7_us"])) for row in rows), default=0)

    text = "\n".join(
        [
            "# Zephyr Native Sim Zenoh SIL",
            "",
            f"- Zenoh router locator: `{locator}`",
            f"- native_sim executable: `{sim}`",
            f"- PWM samples received: `{len(rows)}`",
            f"- Max `output6_us` controller speed signal: `{max_speed_signal}`",
            f"- Max throttle PWM `output2_us`: `{max_throttle}`",
            f"- Max absolute roll-command signal `output7_us`: `{max_bank_signal}`",
            "",
            f"This test starts `zenohd`, runs `{sim}`,",
            "publishes manual-control and mocap inputs on Synapse Zenoh topics,",
            "and verifies that the Zephyr app publishes non-idle PWM outputs.",
            "",
        ]
    )
    path.write_text(text)


def main() -> int:
    args = parse_args()
    artifact_dir = Path(args.artifacts)
    artifact_dir.mkdir(parents=True, exist_ok=True)

    sim = Path(args.sim)
    if not sim.exists():
        raise FileNotFoundError(f"native_sim executable not found: {sim}")

    router_log = artifact_dir / "zenohd.log"
    sim_log = artifact_dir / "native-sim.log"
    csv_path = artifact_dir / "native-sim-pwm.csv"
    summary_path = artifact_dir / "native-sim-summary.md"

    router: subprocess.Popen[bytes] | None = None
    zephyr: subprocess.Popen[bytes] | None = None
    session: zenoh.Session | None = None

    try:
        router = start_process("zenohd", ["zenohd", "-l", args.locator], router_log)
        time.sleep(0.5)
        require_running(router, router_log, "zenohd")

        zephyr = start_process("native_sim", [os.fspath(sim)], sim_log)
        session = open_zenoh_session(args.locator, args.startup_timeout_s)
        pwm_subscriber = session.declare_subscriber(PWM_TOPIC)

        rows: list[dict[str, int | float]] = []
        period_s = 1.0 / args.rate_hz
        start_s = time.monotonic()
        next_publish_s = start_s

        while time.monotonic() - start_s < args.duration_s:
            require_running(router, router_log, "zenohd")
            require_running(zephyr, sim_log, "native_sim")

            now_s = time.monotonic()
            if now_s >= next_publish_s:
                elapsed_s = now_s - start_s
                timestamp_us = int(elapsed_s * 1_000_000)
                session.put(MANUAL_TOPIC, pack_manual_control(timestamp_us))
                session.put(MOCAP_TOPIC, pack_mocap(elapsed_s))
                next_publish_s += period_s

            drain_pwm(pwm_subscriber, rows)
            time.sleep(0.002)

        drain_pwm(pwm_subscriber, rows)
        write_csv(csv_path, rows)
        write_summary(summary_path, args.locator, sim, rows)

        if not rows:
            raise RuntimeError(f"no PWM outputs received over {PWM_TOPIC}\n\n{tail(sim_log)}")

        max_speed_signal = max(int(row["output6_us"]) for row in rows)
        if max_speed_signal <= 2500:
            raise RuntimeError(
                "native_sim stayed idle or manual; expected autonomous controller output "
                f"signal output6_us > 2500, got {max_speed_signal}\n\n{tail(sim_log)}"
            )

        print(f"received {len(rows)} PWM samples from {PWM_TOPIC}")
        print(f"max controller speed signal output6_us={max_speed_signal}")
        print(f"wrote {csv_path}")
        print(f"wrote {summary_path}")
        return 0
    finally:
        if session is not None:
            session.close()
        stop_process(zephyr)
        stop_process(router)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
