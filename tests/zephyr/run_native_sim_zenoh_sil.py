#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Run native_sim against a Rumoca/CMM plant through real Synapse Zenoh topics."""

from __future__ import annotations

import argparse
import base64
import csv
from dataclasses import dataclass
import html
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import time
from typing import Iterable

import flatbuffers
import matplotlib
import numpy as np
import zenoh

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]

SYNAPSE_MOCAP_TOPIC = "synapse/v1/topic/mocap_frame"
SYNAPSE_PWM_TOPIC = "synapse/v1/topic/pwm_signal_outputs"
SYNAPSE_ATTITUDE_COMMAND_TOPIC = "synapse/v1/topic/attitude_command"
RUMOCA_MOCAP_TOPIC = "cubs2/sil/mocap_sample"
RUMOCA_PWM_TOPIC = "cubs2/sil/pwm_outputs"

NATIVE_IO_SCHEMA = ROOT / "tests" / "zephyr" / "native_sil_io.fbs"
DEFAULT_SCENARIO = ROOT / "tests" / "zephyr" / "rumoca-scenario.native-sim.toml"

PWM_STRUCT = struct.Struct("<QIBx16H2x")
ATTITUDE_COMMAND_STRUCT = struct.Struct("<Q4f3ffB7x")

ROUTE_WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (-4.0, -5.0, 3.0),
    (-3.0, 2.0, 3.0),
    (16.20, 2.0, 3.0),
    (16.0, -4.22, 3.0),
    (6.88, -5.1, 3.0),
    (-4.0, -5.0, 3.0),
]


@dataclass
class MocapSample:
    timestamp_us: int
    frame_number: int
    x_m: float
    y_m: float
    z_m: float
    qw: float
    qx: float
    qy: float
    qz: float
    tracking_valid: bool


@dataclass
class BridgeLog:
    mocap_rows: list[dict[str, float | int | bool]]
    pwm_rows: list[dict[str, float | int]]
    attitude_rows: list[dict[str, float | int]]
    error: Exception | None = None


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
        help="directory for logs, CSV, plots, and reports",
    )
    parser.add_argument(
        "--locator",
        default="udp/127.0.0.1:7447",
        help="Zenoh router locator; must match CONFIG_CSYN_ZENOH_LOCATOR",
    )
    parser.add_argument(
        "--scenario",
        default=os.fspath(DEFAULT_SCENARIO),
        help="Rumoca scenario that drives the CMM plant",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="optional simulation duration override for local debugging",
    )
    parser.add_argument("--startup-timeout-s", type=float, default=6.0)
    parser.add_argument("--shutdown-timeout-s", type=float, default=4.0)
    return parser.parse_args()


def tail(path: Path, line_count: int = 100) -> str:
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


def stop_process(process: subprocess.Popen[bytes] | None, timeout_s: float = 2.0) -> None:
    if process is None:
        return

    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=timeout_s)

    log = getattr(process, "_cubs2_log", None)
    if log is not None:
        log.close()


def require_running(process: subprocess.Popen[bytes], log_path: Path, name: str) -> None:
    rc = process.poll()
    if rc is None:
        return
    raise RuntimeError(f"{name} exited early with status {rc}\n\n{tail(log_path)}")


def run_checked(cmd: list[str], *, cwd: Path = ROOT) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


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
        except Exception as exc:
            last_error = exc
            time.sleep(0.1)
    raise RuntimeError(f"could not open Zenoh session to {locator}: {last_error}")


def generate_bfbs(artifact_dir: Path) -> Path:
    bfbs = artifact_dir / "native_sil_io.bfbs"
    run_checked(["flatc", "--binary", "--schema", "-o", os.fspath(artifact_dir), os.fspath(NATIVE_IO_SCHEMA)])
    if not bfbs.exists():
        raise FileNotFoundError(f"flatc did not write {bfbs}")
    return bfbs


def scenario_with_duration(scenario: Path, artifact_dir: Path, t_end: float | None) -> Path:
    if t_end is None:
        return scenario

    text = scenario.read_text()
    updated, count = re.subn(r"(?m)^t_end\s*=\s*[0-9.]+", f"t_end = {t_end}", text, count=1)
    if count != 1:
        raise RuntimeError(f"could not override t_end in {scenario}")
    updated = updated.replace(
        'file = "Cubs2NativeSimSIL.mo"',
        f'file = "{(ROOT / "tests" / "zephyr" / "Cubs2NativeSimSIL.mo").as_posix()}"',
    )
    updated = updated.replace(
        '"../../models/vendor/CMM-v0.0.2"',
        f'"{(ROOT / "models" / "vendor" / "CMM-v0.0.2").as_posix()}"',
    )
    updated = updated.replace(
        '"../../models/plant"',
        f'"{(ROOT / "models" / "plant").as_posix()}"',
    )
    path = artifact_dir / scenario.name
    path.write_text(updated)
    return path


def table_start(buf: bytes) -> int:
    return struct.unpack_from("<I", buf, 0)[0]


def read_flatbuffer_field(buf: bytes, field_id: int, fmt: str, default: float | int | bool) -> float | int | bool:
    table = table_start(buf)
    vtable = table - struct.unpack_from("<i", buf, table)[0]
    vtable_size = struct.unpack_from("<H", buf, vtable)[0]
    slot = vtable + 4 + 2 * field_id
    if slot + 2 > vtable + vtable_size:
        return default
    offset = struct.unpack_from("<H", buf, slot)[0]
    if offset == 0:
        return default
    return struct.unpack_from(fmt, buf, table + offset)[0]


def decode_mocap_sample(payload: bytes) -> MocapSample:
    return MocapSample(
        timestamp_us=int(read_flatbuffer_field(payload, 0, "<Q", 0)),
        frame_number=int(read_flatbuffer_field(payload, 1, "<I", 0)),
        x_m=float(read_flatbuffer_field(payload, 2, "<f", 0.0)),
        y_m=float(read_flatbuffer_field(payload, 3, "<f", 0.0)),
        z_m=float(read_flatbuffer_field(payload, 4, "<f", 0.0)),
        qw=float(read_flatbuffer_field(payload, 5, "<f", 1.0)),
        qx=float(read_flatbuffer_field(payload, 6, "<f", 0.0)),
        qy=float(read_flatbuffer_field(payload, 7, "<f", 0.0)),
        qz=float(read_flatbuffer_field(payload, 8, "<f", 0.0)),
        tracking_valid=bool(read_flatbuffer_field(payload, 9, "<B", 0)),
    )


def prepend_mocap_rigid_body_sample(builder: flatbuffers.Builder, sample: MocapSample) -> int:
    builder.Prep(4, 40)
    builder.Pad(3)
    builder.PrependBool(sample.tracking_valid)
    builder.PrependFloat32(0.0)
    builder.PrependFloat32(sample.qz)
    builder.PrependFloat32(sample.qy)
    builder.PrependFloat32(sample.qx)
    builder.PrependFloat32(sample.qw)
    builder.PrependFloat32(sample.z_m)
    builder.PrependFloat32(sample.y_m)
    builder.PrependFloat32(sample.x_m)
    builder.PrependInt32(1)
    return builder.Offset()


def pack_synapse_mocap_frame(sample: MocapSample) -> bytes:
    builder = flatbuffers.Builder(128)
    builder.StartVector(40, 1, 4)
    prepend_mocap_rigid_body_sample(builder, sample)
    rigid_bodies = builder.EndVector()

    builder.StartObject(6)
    builder.PrependUOffsetTRelativeSlot(4, rigid_bodies, 0)
    builder.PrependUint32Slot(1, sample.frame_number, 0)
    builder.PrependUint64Slot(0, sample.timestamp_us, 0)
    frame = builder.EndObject()
    builder.Finish(frame)
    return bytes(builder.Output())


def decode_pwm_outputs(payload: bytes, sim_time_s: float) -> dict[str, float | int]:
    if len(payload) != PWM_STRUCT.size:
        raise ValueError(f"expected {PWM_STRUCT.size} PWM bytes, got {len(payload)}")
    timestamp_us, active_mask, port, *outputs = PWM_STRUCT.unpack(payload)
    row: dict[str, float | int] = {
        "sim_time_s": sim_time_s,
        "timestamp_us": timestamp_us,
        "active_mask": active_mask,
        "port": port,
    }
    row.update({f"output{i}_us": value for i, value in enumerate(outputs)})
    return row


def pack_rumoca_pwm_outputs(row: dict[str, float | int]) -> bytes:
    # Match Rumoca PackCodec's deterministic all-inline table layout for
    # cubs2.sil.PwmOutputs. This avoids a compact builder layout whose size can
    # differ from the receive codec's expected 104-byte packet.
    buf = bytearray(104)
    vtable_off = 4
    table_off = 48
    field_offsets = [
        8,   # timestamp_us
        16,  # active_mask
        20,  # port
        22,  # output0_us
        24,
        26,
        28,
        30,
        32,
        34,
        36,
        38,
        40,
        42,
        44,
        46,
        48,
        50,
        52,  # output15_us
    ]
    struct.pack_into("<I", buf, 0, table_off)
    struct.pack_into("<HH", buf, vtable_off, 42, 56)
    for field_id, offset in enumerate(field_offsets):
        struct.pack_into("<H", buf, vtable_off + 4 + 2 * field_id, offset)
    struct.pack_into("<I", buf, table_off, table_off - vtable_off)
    struct.pack_into("<Q", buf, table_off + field_offsets[0], int(row["timestamp_us"]))
    struct.pack_into("<I", buf, table_off + field_offsets[1], int(row["active_mask"]))
    struct.pack_into("<B", buf, table_off + field_offsets[2], int(row["port"]))
    for idx in range(16):
        struct.pack_into("<H", buf, table_off + field_offsets[3 + idx], int(row[f"output{idx}_us"]))
    return bytes(buf)


def euler_from_quat(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * ((qw * qx) + (qy * qz))
    cosr_cosp = 1.0 - (2.0 * ((qx * qx) + (qy * qy)))
    sinp = 2.0 * ((qw * qy) - (qz * qx))
    siny_cosp = 2.0 * ((qw * qz) + (qx * qy))
    cosy_cosp = 1.0 - (2.0 * ((qy * qy) + (qz * qz)))
    roll = math.atan2(sinr_cosp, cosr_cosp)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def decode_attitude_command(payload: bytes, sim_time_s: float) -> dict[str, float | int]:
    if len(payload) != ATTITUDE_COMMAND_STRUCT.size:
        raise ValueError(
            f"expected {ATTITUDE_COMMAND_STRUCT.size} attitude-command bytes, got {len(payload)}"
        )
    values = ATTITUDE_COMMAND_STRUCT.unpack(payload)
    timestamp_us = values[0]
    qw, qx, qy, qz = values[1:5]
    roll, pitch, yaw = euler_from_quat(qw, qx, qy, qz)
    rate_roll, rate_pitch, rate_yaw = values[5:8]
    thrust = values[8]
    type_mask = values[9]
    return {
        "sim_time_s": sim_time_s,
        "timestamp_us": timestamp_us,
        "roll_cmd_rad": roll,
        "pitch_cmd_rad": pitch,
        "yaw_cmd_rad": yaw,
        "rate_roll_cmd_rad_s": rate_roll,
        "rate_pitch_cmd_rad_s": rate_pitch,
        "rate_yaw_cmd_rad_s": rate_yaw,
        "thrust_cmd": thrust,
        "type_mask": type_mask,
    }


def payload_bytes(sample: object) -> bytes:
    return bytes(sample.payload)


def bridge_topics(locator: str, stop: threading.Event, logs: BridgeLog, startup_timeout_s: float) -> None:
    session: zenoh.Session | None = None
    try:
        session = open_zenoh_session(locator, startup_timeout_s)
        mocap_subscriber = session.declare_subscriber(RUMOCA_MOCAP_TOPIC)
        pwm_subscriber = session.declare_subscriber(SYNAPSE_PWM_TOPIC)
        attitude_subscriber = session.declare_subscriber(SYNAPSE_ATTITUDE_COMMAND_TOPIC)
        latest_sim_time_s = 0.0
        mocap_forwarded = False
        control_forwarded = False

        while not stop.is_set():
            did_work = False

            while True:
                sample = mocap_subscriber.try_recv()
                if sample is None:
                    break
                mocap = decode_mocap_sample(payload_bytes(sample))
                latest_sim_time_s = mocap.timestamp_us / 1_000_000.0
                session.put(SYNAPSE_MOCAP_TOPIC, pack_synapse_mocap_frame(mocap))
                mocap_forwarded = True
                logs.mocap_rows.append(
                    {
                        "sim_time_s": latest_sim_time_s,
                        "timestamp_us": mocap.timestamp_us,
                        "frame_number": mocap.frame_number,
                        "x_m": mocap.x_m,
                        "y_m": mocap.y_m,
                        "z_m": mocap.z_m,
                        "qw": mocap.qw,
                        "qx": mocap.qx,
                        "qy": mocap.qy,
                        "qz": mocap.qz,
                        "tracking_valid": mocap.tracking_valid,
                    }
                )
                did_work = True

            while True:
                sample = pwm_subscriber.try_recv()
                if sample is None:
                    break
                row = decode_pwm_outputs(payload_bytes(sample), latest_sim_time_s)
                real_control = int(row["output2_us"]) > 1100 or int(row["output6_us"]) > 1000
                forward_to_plant = control_forwarded or (mocap_forwarded and real_control)
                row["forwarded_to_plant"] = int(forward_to_plant)
                if forward_to_plant:
                    session.put(RUMOCA_PWM_TOPIC, pack_rumoca_pwm_outputs(row))
                    control_forwarded = True
                logs.pwm_rows.append(row)
                did_work = True

            while True:
                sample = attitude_subscriber.try_recv()
                if sample is None:
                    break
                logs.attitude_rows.append(decode_attitude_command(payload_bytes(sample), latest_sim_time_s))
                did_work = True

            if not did_work:
                time.sleep(0.001)
    except Exception as exc:
        logs.error = exc
        stop.set()
    finally:
        if session is not None:
            session.close()


def write_csv(path: Path, rows: Iterable[dict[str, float | int | bool]]) -> None:
    rows = list(rows)
    if rows:
        fieldnames = list(rows[0].keys())
    else:
        fieldnames = []
    with path.open("w", newline="") as f:
        if fieldnames:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


def load_csv(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        return [
            {normalise_column(key): float(value) for key, value in row.items() if value != ""}
            for row in reader
        ]


def normalise_column(name: str) -> str:
    if name.startswith("model:"):
        return name.split(":", 1)[1]
    return name


def nearest_rows(rows: list[dict[str, float | int]], time_key: str = "sim_time_s") -> callable:
    idx = 0

    def nearest(t: float) -> dict[str, float | int]:
        nonlocal idx
        if not rows:
            return {}
        while idx + 1 < len(rows) and abs(float(rows[idx + 1][time_key]) - t) <= abs(float(rows[idx][time_key]) - t):
            idx += 1
        return rows[idx]

    return nearest


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    offset = 0.0
    previous = values[0]
    for value in values[1:]:
        delta = value - previous
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        result.append(value + offset)
        previous = value
    return result


def route_error(x_m: float, y_m: float, z_m: float, waypoint: int) -> tuple[float, float, float]:
    segments = range(1, len(ROUTE_WAYPOINTS))
    if waypoint in segments:
        candidates = [waypoint]
    else:
        candidates = list(segments)

    best: tuple[float, float, float, float] | None = None
    for idx in candidates:
        start = ROUTE_WAYPOINTS[idx - 1]
        end = ROUTE_WAYPOINTS[idx]
        sx, sy, sz = start
        ex, ey, ez = end
        vx = ex - sx
        vy = ey - sy
        length = max(math.hypot(vx, vy), 1e-6)
        ux = vx / length
        uy = vy / length
        dx = x_m - sx
        dy = y_m - sy
        along = dx * ux + dy * uy
        progress = max(0.0, min(1.0, along / length))
        path_altitude = sz + progress * (ez - sz)
        cross = dx * (-uy) + dy * ux
        distance = abs(cross) if 0.0 <= along <= length else min(
            math.hypot(x_m - sx, y_m - sy),
            math.hypot(x_m - ex, y_m - ey),
        )
        candidate = (distance, cross, path_altitude, along)
        if best is None or candidate[0] < best[0]:
            best = candidate

    assert best is not None
    return best[1], best[2], best[3]


def merge_flight_rows(
    plant_rows: list[dict[str, float]],
    pwm_rows: list[dict[str, float | int]],
    attitude_rows: list[dict[str, float | int]],
) -> list[dict[str, float]]:
    pwm_nearest = nearest_rows(pwm_rows)
    attitude_nearest = nearest_rows(attitude_rows)
    merged: list[dict[str, float]] = []

    for plant in plant_rows:
        t = plant["time"]
        pwm = pwm_nearest(t)
        attitude = attitude_nearest(t)
        waypoint = int(round(float(pwm.get("output5_us", plant.get("current_waypoint", 0.0)))))
        crosstrack_m, altitude_cmd_m, _along = route_error(plant["x_m"], plant["y_m"], plant["z_m"], waypoint)
        vx = plant["vx_m_s"]
        vy = plant["vy_m_s"]
        ground_track = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.2 else plant["yaw_rad"]
        row = dict(plant)
        row.update(
            {
                "time_s": t,
                "heading_rad": plant["yaw_rad"],
                "ground_track_rad": ground_track,
                "heading_cmd_rad": float(attitude.get("yaw_cmd_rad", 0.0)),
                "bank_cmd_rad": float(attitude.get("roll_cmd_rad", 0.0)),
                "pitch_cmd_rad": float(attitude.get("pitch_cmd_rad", 0.0)),
                "thrust_cmd": float(attitude.get("thrust_cmd", 0.0)),
                "altitude_cmd_m": altitude_cmd_m,
                "crosstrack_error_m": crosstrack_m,
                "speed_cmd_m_s": float(pwm.get("output6_us", plant.get("desired_speed_m_s", 0.0))) / 1000.0,
                "current_waypoint": float(waypoint),
                "pwm_output0_us": float(pwm.get("output0_us", 1500.0)),
                "pwm_output1_us": float(pwm.get("output1_us", 1500.0)),
                "pwm_output2_us": float(pwm.get("output2_us", 1000.0)),
                "pwm_output3_us": float(pwm.get("output3_us", 1500.0)),
                "pwm_output4_us": float(pwm.get("output4_us", 1000.0)),
                "pwm_forwarded_to_plant": float(pwm.get("forwarded_to_plant", 0.0)),
            }
        )
        merged.append(row)

    return merged


def values(rows: list[dict[str, float]], key: str) -> list[float]:
    return [row[key] for row in rows]


def decimate(rows: list[dict[str, float]], max_points: int = 6000) -> list[dict[str, float]]:
    if len(rows) <= max_points:
        return rows
    step = max(1, len(rows) // max_points)
    return rows[::step]


def percentile_abs(rows: list[dict[str, float]], key: str, pct: float) -> float:
    if not rows:
        return float("nan")
    return float(np.percentile(np.abs(np.array(values(rows, key))), pct))


def max_abs(rows: list[dict[str, float]], key: str) -> float:
    return max((abs(row[key]) for row in rows), default=float("nan"))


def count_laps(rows: list[dict[str, float]]) -> int:
    laps = 0
    previous = int(rows[0]["current_waypoint"]) if rows else 1
    for row in rows[1:]:
        current = int(row["current_waypoint"])
        if previous >= 5 and current <= 2:
            laps += 1
        previous = current
    return laps


def save_plot(fig: plt.Figure, artifact_dir: Path, name: str) -> Path:
    path = artifact_dir / name
    fig.savefig(path, dpi=170)
    plt.close(fig)
    print(f"wrote {path}")
    return path


def plot_flight(rows: list[dict[str, float]], artifact_dir: Path) -> list[Path]:
    plot_rows = decimate(rows)
    t = values(plot_rows, "time_s")
    route_x = [wp[0] for wp in ROUTE_WAYPOINTS]
    route_y = [wp[1] for wp in ROUTE_WAYPOINTS]
    route_z = [wp[2] for wp in ROUTE_WAYPOINTS]
    paths: list[Path] = []

    fig, axes = plt.subplots(3, 2, figsize=(15, 12), constrained_layout=True)
    ax_track, ax_alt, ax_heading, ax_velocity, ax_bank, ax_cross = axes.flat
    ax_track.plot(values(plot_rows, "x_m"), values(plot_rows, "y_m"), label="flight")
    ax_track.plot(route_x, route_y, "k--", linewidth=1.0, label="waypoints")
    ax_track.scatter(route_x, route_y, color="black", s=22)
    for idx, (x, y, _z) in enumerate(ROUTE_WAYPOINTS, start=1):
        ax_track.annotate(str(idx), (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax_track.set_title("Top-Down Track")
    ax_track.set_xlabel("x [m]")
    ax_track.set_ylabel("y [m]")
    ax_track.axis("equal")
    ax_track.grid(True)
    ax_track.legend(loc="best")

    ax_alt.plot(t, values(plot_rows, "altitude_cmd_m"), "k--", label="cmd")
    ax_alt.plot(t, values(plot_rows, "z_m"), label="actual")
    ax_alt.set_title("Altitude")
    ax_alt.set_xlabel("time [s]")
    ax_alt.set_ylabel("m")
    ax_alt.grid(True)
    ax_alt.legend(loc="best")

    ax_heading.plot(t, np.degrees(unwrap(values(plot_rows, "heading_cmd_rad"))), "k--", label="cmd")
    ax_heading.plot(t, np.degrees(unwrap(values(plot_rows, "heading_rad"))), label="yaw")
    ax_heading.plot(t, np.degrees(unwrap(values(plot_rows, "ground_track_rad"))), label="track", alpha=0.75)
    ax_heading.set_title("Heading")
    ax_heading.set_xlabel("time [s]")
    ax_heading.set_ylabel("deg")
    ax_heading.grid(True)
    ax_heading.legend(loc="best")

    ax_velocity.plot(t, values(plot_rows, "speed_cmd_m_s"), "k--", label="cmd")
    ax_velocity.plot(t, values(plot_rows, "airspeed_m_s"), label="airspeed")
    ax_velocity.set_title("Velocity")
    ax_velocity.set_xlabel("time [s]")
    ax_velocity.set_ylabel("m/s")
    ax_velocity.grid(True)
    ax_velocity.legend(loc="best")

    ax_bank.plot(t, np.degrees(values(plot_rows, "bank_cmd_rad")), "k--", label="cmd")
    ax_bank.plot(t, np.degrees(values(plot_rows, "roll_rad")), label="actual")
    ax_bank.set_title("Bank")
    ax_bank.set_xlabel("time [s]")
    ax_bank.set_ylabel("deg")
    ax_bank.grid(True)
    ax_bank.legend(loc="best")

    ax_cross.plot(t, values(plot_rows, "crosstrack_error_m"))
    ax_cross.axhline(0.0, color="black", linewidth=0.8)
    ax_cross.set_title("Crosstrack Error")
    ax_cross.set_xlabel("time [s]")
    ax_cross.set_ylabel("m")
    ax_cross.grid(True)
    paths.append(save_plot(fig, artifact_dir, "native-sim-overview.png"))

    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    ax.plot(values(plot_rows, "x_m"), values(plot_rows, "y_m"), label="flight path")
    ax.plot(route_x, route_y, "k--", label="waypoint route")
    ax.scatter(route_x, route_y, color="black", s=28)
    ax.set_title("Top-Down Route Tracking")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")
    paths.append(save_plot(fig, artifact_dir, "native-sim-topdown.png"))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    axes[0].plot(t, values(plot_rows, "thrust_cmd"), "k--", label="thrust command")
    axes[0].plot(t, values(plot_rows, "throttle_cmd"), label="plant throttle")
    axes[0].set_title("Throttle")
    axes[0].set_ylabel("normalized")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(t, values(plot_rows, "elevator_cmd"), label="plant elevator")
    axes[1].set_title("Elevator")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("normalized")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    paths.append(save_plot(fig, artifact_dir, "native-sim-actuators.png"))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    axes[0].plot(t, np.degrees(values(plot_rows, "bank_cmd_rad")), "k--", label="bank command")
    axes[0].plot(t, np.degrees(values(plot_rows, "roll_rad")), label="bank actual")
    axes[0].set_title("Bank Command Response")
    axes[0].set_ylabel("deg")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(t, np.degrees(values(plot_rows, "pitch_cmd_rad")), "k--", label="attitude pitch command")
    axes[1].plot(t, np.degrees(values(plot_rows, "pitch_rad")), label="pitch actual")
    axes[1].set_title("Pitch Response")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("deg")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    paths.append(save_plot(fig, artifact_dir, "native-sim-attitude.png"))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    axes[0].plot(t, values(plot_rows, "altitude_cmd_m"), "k--", label="altitude command")
    axes[0].plot(t, values(plot_rows, "z_m"), label="altitude")
    axes[0].set_ylabel("m")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(t, values(plot_rows, "speed_cmd_m_s"), "k--", label="speed command")
    axes[1].plot(t, values(plot_rows, "airspeed_m_s"), label="airspeed")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("m/s")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    paths.append(save_plot(fig, artifact_dir, "native-sim-energy.png"))

    return paths


def flight_metrics(rows: list[dict[str, float]], logs: BridgeLog) -> dict[str, float | int]:
    after_takeoff = [row for row in rows if row["time_s"] >= 10.0]
    tracking_rows = after_takeoff or rows
    laps = count_laps(rows)
    mean_altitude_error = float(np.mean(np.abs(np.array(values(tracking_rows, "z_m")) - np.array(values(tracking_rows, "altitude_cmd_m")))))
    mean_speed = float(np.mean(values(tracking_rows, "airspeed_m_s")))
    mean_speed_error = float(np.mean(np.abs(np.array(values(tracking_rows, "airspeed_m_s")) - np.array(values(tracking_rows, "speed_cmd_m_s")))))
    return {
        "mocap_samples": len(logs.mocap_rows),
        "pwm_samples": len(logs.pwm_rows),
        "attitude_command_samples": len(logs.attitude_rows),
        "duration_s": rows[-1]["time_s"] if rows else 0.0,
        "laps": laps,
        "max_altitude_m": max(values(rows, "z_m"), default=0.0),
        "final_altitude_m": rows[-1]["z_m"] if rows else 0.0,
        "mean_abs_altitude_error_m": mean_altitude_error,
        "mean_airspeed_m_s": mean_speed,
        "mean_abs_speed_error_m_s": mean_speed_error,
        "p95_abs_crosstrack_m": percentile_abs(tracking_rows, "crosstrack_error_m", 95.0),
        "max_abs_bank_deg": math.degrees(max_abs(tracking_rows, "roll_rad")),
        "max_abs_pitch_deg": math.degrees(max_abs(tracking_rows, "pitch_rad")),
    }


def run_checks(metrics: dict[str, float | int]) -> list[tuple[str, str, str]]:
    checks = [
        ("mocap published", int(metrics["mocap_samples"]) > 100, f"{metrics['mocap_samples']} samples"),
        ("pwm received", int(metrics["pwm_samples"]) > 50, f"{metrics['pwm_samples']} samples"),
        (
            "attitude command received",
            int(metrics["attitude_command_samples"]) > 50,
            f"{metrics['attitude_command_samples']} samples",
        ),
        ("takeoff altitude", float(metrics["max_altitude_m"]) > 2.0, f"max {metrics['max_altitude_m']:.2f} m"),
        ("route laps", int(metrics["laps"]) >= 2, f"{metrics['laps']} laps"),
        (
            "altitude tracking",
            float(metrics["mean_abs_altitude_error_m"]) < 1.5,
            f"mean abs error {metrics['mean_abs_altitude_error_m']:.2f} m",
        ),
        (
            "velocity tracking",
            2.0 < float(metrics["mean_airspeed_m_s"]) < 8.0
            and float(metrics["mean_abs_speed_error_m_s"]) < 3.0,
            f"mean {metrics['mean_airspeed_m_s']:.2f} m/s, mean abs error {metrics['mean_abs_speed_error_m_s']:.2f} m/s",
        ),
        (
            "crosstrack tracking",
            float(metrics["p95_abs_crosstrack_m"]) < 10.0,
            f"p95 abs {metrics['p95_abs_crosstrack_m']:.2f} m",
        ),
        ("bank bounded", float(metrics["max_abs_bank_deg"]) < 80.0, f"max abs {metrics['max_abs_bank_deg']:.1f} deg"),
        ("pitch bounded", float(metrics["max_abs_pitch_deg"]) < 60.0, f"max abs {metrics['max_abs_pitch_deg']:.1f} deg"),
    ]
    return [(name, "PASS" if ok else "FAIL", detail) for name, ok, detail in checks]


def write_merged_csv(path: Path, rows: list[dict[str, float]]) -> None:
    if not rows:
        path.write_text("")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def image_data_uri(path: Path) -> str:
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def write_reports(
    artifact_dir: Path,
    metrics: dict[str, float | int],
    checks: list[tuple[str, str, str]],
    plot_paths: list[Path],
) -> None:
    summary = artifact_dir / "native-sim-summary.md"
    lines = [
        "# CUBS2 Zephyr Native SIL",
        "",
        "This run uses the Zephyr `native_sim` binary, Rumoca/CMM SportCub plant dynamics, and real Synapse Zenoh topics. The bridge publishes a real `MocapFrame` table on `synapse/v1/topic/mocap_frame` and consumes the app's fixed-layout `pwm_signal_outputs` and `attitude_command` topics.",
        "",
        "While this test is running, the same traffic can be inspected with:",
        "",
        "```sh",
        "csyn --connect udp/127.0.0.1:7447 topic echo mocap_frame",
        "csyn --connect udp/127.0.0.1:7447 topic hz pwm_signal_outputs",
        "csyn --connect udp/127.0.0.1:7447 topic echo attitude_command",
        "```",
        "",
        "## Checks",
        "",
        "| check | status | detail |",
        "| --- | --- | --- |",
    ]
    lines.extend(f"| {name} | {status} | {detail} |" for name, status, detail in checks)
    lines.extend(["", "## Metrics", "", "| metric | value |", "| --- | ---: |"])
    for key, value in metrics.items():
        if isinstance(value, float):
            lines.append(f"| {key} | {value:.3f} |")
        else:
            lines.append(f"| {key} | {value} |")
    lines.extend(
        [
            "",
            "The native app currently publishes `attitude_command` with roll and heading command plus thrust; pitch is not commanded there, so pitch tracking is plotted against that zero attitude-command pitch while elevator response is shown separately.",
            "",
            "Open `native-sim-report.html` or the PNG artifacts for the flight plots.",
            "",
        ]
    )
    summary.write_text("\n".join(lines))

    html_lines = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<title>CUBS2 Zephyr Native SIL</title>",
        "<style>body{font-family:sans-serif;margin:2rem;max-width:1100px} img{max-width:100%;height:auto;border:1px solid #ccc;margin:1rem 0} table{border-collapse:collapse} td,th{border:1px solid #ccc;padding:.35rem .55rem;text-align:left}</style>",
        "</head><body>",
        "<h1>CUBS2 Zephyr Native SIL</h1>",
        "<h2>Checks</h2><table><tr><th>check</th><th>status</th><th>detail</th></tr>",
    ]
    for name, status, detail in checks:
        html_lines.append(f"<tr><td>{html.escape(name)}</td><td>{status}</td><td>{html.escape(detail)}</td></tr>")
    html_lines.append("</table><h2>Metrics</h2><table><tr><th>metric</th><th>value</th></tr>")
    for key, value in metrics.items():
        rendered = f"{value:.3f}" if isinstance(value, float) else str(value)
        html_lines.append(f"<tr><td>{html.escape(key)}</td><td>{rendered}</td></tr>")
    html_lines.append("</table><h2>Plots</h2>")
    for path in plot_paths:
        html_lines.append(f"<h3>{html.escape(path.name)}</h3><img src='{image_data_uri(path)}' alt='{html.escape(path.name)}'>")
    html_lines.append("</body></html>")
    (artifact_dir / "native-sim-report.html").write_text("\n".join(html_lines))


def main() -> int:
    args = parse_args()
    artifact_dir = (ROOT / args.artifacts).resolve() if not Path(args.artifacts).is_absolute() else Path(args.artifacts)
    artifact_dir.mkdir(parents=True, exist_ok=True)

    sim = Path(args.sim)
    if not sim.is_absolute():
        sim = ROOT / sim
    if not sim.exists():
        raise FileNotFoundError(f"native_sim executable not found: {sim}")

    scenario = Path(args.scenario)
    if not scenario.is_absolute():
        scenario = ROOT / scenario
    if not scenario.exists():
        raise FileNotFoundError(f"Rumoca scenario not found: {scenario}")

    router_log = artifact_dir / "zenohd.log"
    sim_log = artifact_dir / "native-sim.log"
    rumoca_log = artifact_dir / "rumoca-python.log"
    plant_csv = artifact_dir / "native-sim-plant.csv"
    mocap_csv = artifact_dir / "native-sim-mocap.csv"
    pwm_csv = artifact_dir / "native-sim-pwm.csv"
    attitude_csv = artifact_dir / "native-sim-attitude-command.csv"
    merged_csv = artifact_dir / "native-sim-flight.csv"

    router: subprocess.Popen[bytes] | None = None
    zephyr: subprocess.Popen[bytes] | None = None
    rumoca_python: subprocess.Popen[bytes] | None = None
    stop_bridge = threading.Event()
    bridge_log = BridgeLog(mocap_rows=[], pwm_rows=[], attitude_rows=[])
    bridge_thread: threading.Thread | None = None

    try:
        generate_bfbs(artifact_dir)

        router = start_process("zenohd", ["zenohd", "-l", args.locator], router_log)
        time.sleep(0.5)
        require_running(router, router_log, "zenohd")

        zephyr = start_process("native_sim", [os.fspath(sim)], sim_log)
        time.sleep(0.3)
        require_running(zephyr, sim_log, "native_sim")

        bridge_thread = threading.Thread(
            target=bridge_topics,
            args=(args.locator, stop_bridge, bridge_log, args.startup_timeout_s),
            name="native-sil-bridge",
            daemon=True,
        )
        bridge_thread.start()

        scenario_to_run = scenario_with_duration(scenario, artifact_dir, args.t_end)
        rumoca_cmd = [
            sys.executable,
            os.fspath(ROOT / "scripts" / "rumoca_scenario.py"),
            "-c",
            os.fspath(scenario_to_run),
        ]
        rumoca_python = start_process("rumoca-python", rumoca_cmd, rumoca_log)

        while rumoca_python.poll() is None:
            require_running(router, router_log, "zenohd")
            require_running(zephyr, sim_log, "native_sim")
            if bridge_log.error is not None:
                raise RuntimeError(f"native SIL bridge failed: {bridge_log.error}")
            time.sleep(0.1)

        if rumoca_python.returncode != 0:
            raise RuntimeError(
                f"Rumoca Python scenario runner exited with status {rumoca_python.returncode}"
                f"\n\n{tail(rumoca_log)}"
            )

        stop_bridge.set()
        if bridge_thread is not None:
            bridge_thread.join(timeout=args.shutdown_timeout_s)

        if bridge_log.error is not None:
            raise RuntimeError(f"native SIL bridge failed: {bridge_log.error}")

        write_csv(mocap_csv, bridge_log.mocap_rows)
        write_csv(pwm_csv, bridge_log.pwm_rows)
        write_csv(attitude_csv, bridge_log.attitude_rows)

        if not plant_csv.exists():
            raise RuntimeError(f"Rumoca plant trace was not written: {plant_csv}\n\n{tail(rumoca_log)}")

        plant_rows = load_csv(plant_csv)
        if not plant_rows:
            raise RuntimeError(f"Rumoca plant trace is empty: {plant_csv}\n\n{tail(rumoca_log)}")

        merged_rows = merge_flight_rows(plant_rows, bridge_log.pwm_rows, bridge_log.attitude_rows)
        write_merged_csv(merged_csv, merged_rows)
        plot_paths = plot_flight(merged_rows, artifact_dir)
        metrics = flight_metrics(merged_rows, bridge_log)
        checks = run_checks(metrics)
        write_reports(artifact_dir, metrics, checks, plot_paths)

        failed = [f"{name}: {detail}" for name, status, detail in checks if status != "PASS"]
        if failed:
            raise RuntimeError("native SIL flight checks failed:\n- " + "\n- ".join(failed))

        print(f"wrote {merged_csv}")
        print(f"wrote {artifact_dir / 'native-sim-summary.md'}")
        print(f"wrote {artifact_dir / 'native-sim-report.html'}")
        return 0
    finally:
        stop_bridge.set()
        if bridge_thread is not None and bridge_thread.is_alive():
            bridge_thread.join(timeout=args.shutdown_timeout_s)
        stop_process(rumoca_python, args.shutdown_timeout_s)
        stop_process(zephyr, args.shutdown_timeout_s)
        stop_process(router, args.shutdown_timeout_s)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
