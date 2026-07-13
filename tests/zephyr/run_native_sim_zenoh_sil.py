#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Run native_sim against a Rumoca/CMM plant through fixed Synapse payloads."""

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
import shlex
import shutil
import subprocess
import struct
import sys
import threading
import time
from typing import Callable, Iterable
import xml.etree.ElementTree as ET

import matplotlib
import numpy as np
import flatbuffers
from synapse.topic.AttitudeCommandData import AttitudeCommandData
from synapse.topic.ExternalOdometry import ExternalOdometry
from synapse.topic.ExternalOdometryData import ExternalOdometryData
from synapse.topic.ExternalOdometryFlags import ExternalOdometryFlags
from synapse.topic.ExternalOdometryStatus import ExternalOdometryStatus
from synapse.topic.Odometry import Odometry, OdometryAddData, OdometryEnd, OdometryStart
from synapse.topic.OdometryData import CreateOdometryData, OdometryData
from synapse.topic.PwmSignalOutputsData import PwmSignalOutputsData
from synapse.topic_catalog import topic_by_name
from synapse.types.Quaternionf import Quaternionf
from synapse.types.RateTriplet import RateTriplet
from synapse.types.Vec3f import Vec3f
from synapse import topic_catalog
import zenoh

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = Path(os.environ.get("CUBS2_WORKSPACE_ROOT", ROOT.parent)).resolve()

SYNAPSE_ODOMETRY_TOPIC = "odom"
SYNAPSE_PWM_TOPIC = "pwm"
SYNAPSE_ATTITUDE_COMMAND_TOPIC = "att_sp"
RUMOCA_EXTERNAL_ODOMETRY_TOPIC = "cubs2/sil/external_odometry"
RUMOCA_PWM_TOPIC = "cubs2/sil/pwm_signal_outputs"


def catalog_encoding(topic_name):
    """Mandatory synapse_fbs value contract for a catalog topic (0.6.0+)."""
    topic = topic_catalog.topic_by_name(topic_name)
    media = (
        "application/x-synapse-struct" if topic.fixed_layout else "application/x-flatbuffers"
    )
    return f"{media};type={topic.wire_type};schema=sha256-128:{topic.schema_hash}"

DEFAULT_SCENARIO = ROOT / "tests" / "zephyr" / "rumoca-scenario.native-sim.toml"
DEFAULT_T_END = 40.0
RUMOCA_SCENARIO_CHECK_CODE = (
    "import rumoca as rum; "
    "runner = getattr(rum.Session(), 'run_scenario', None); "
    "assert callable(runner), 'Rumoca Python Session.run_scenario is required'"
)
RUMOCA_RUN_SCENARIO_CODE = "import sys; import rumoca as rum; rum.Session().run_scenario(sys.argv[1])"
FMI_MODEL_FILE = ROOT / "tests" / "zephyr" / "Cubs2NativeSimSIL.mo"
FMI_MODEL_NAME = "Cubs2NativeSimSIL"

TRACE_OUTPUT_NAMES = (
    "x_m",
    "y_m",
    "z_m",
    "vx_m_s",
    "vy_m_s",
    "vz_m_s",
    "airspeed_m_s",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "aileron_cmd",
    "elevator_cmd",
    "throttle_cmd",
    "rudder_cmd",
    "stick_roll_cmd",
    "stick_pitch_cmd",
    "stick_throttle_cmd",
    "stick_yaw_cmd",
    "armed_cmd",
    "current_waypoint",
    "desired_speed_m_s",
    "roll_command_rad",
    "course_error_rad",
)

ODOMETRY_OUTPUT_NAMES = (
    "odometry_timestamp_us",
    "odometry_x_m",
    "odometry_y_m",
    "odometry_z_m",
    "odometry_qw",
    "odometry_qx",
    "odometry_qy",
    "odometry_qz",
    "odometry_vx_m_s",
    "odometry_vy_m_s",
    "odometry_vz_m_s",
    "odometry_roll_rate_rad_s",
    "odometry_pitch_rate_rad_s",
    "odometry_yaw_rate_rad_s",
    "odometry_flags",
    "odometry_status",
    "odometry_source_id",
    "odometry_id",
)

RUMOCA_PWM_TABLE_SIZE = 68
RUMOCA_PWM_STRUCT_OFFSET = 20
PWM_STRUCT_FORMAT = "<QIBx16H2x"
NATIVE_SIL_SHARED_MAGIC = 0x43554253
NATIVE_SIL_SHARED_SIZE = 184

EXTERNAL_ODOMETRY_VALID_FLAGS = (
    ExternalOdometryFlags.PositionValid
    | ExternalOdometryFlags.AttitudeValid
    | ExternalOdometryFlags.LinearVelocityValid
    | ExternalOdometryFlags.AngularVelocityValid
)

ROUTE_WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (12.0, 0.0, 3.0),
    (30.0, 0.0, 3.0),
    (30.0, 20.0, 3.0),
    (0.0, 20.0, 3.0),
    (0.0, 0.0, 3.0),
    (12.0, 0.0, 3.0),
]


@dataclass
class BridgeLog:
    odometry_rows: list[dict[str, float | int | bool]]
    pwm_rows: list[dict[str, float | int]]
    attitude_rows: list[dict[str, float | int]]
    plant_step_wall_s: float = 0.0
    plant_simulated_s: float = 0.0
    error: Exception | None = None


@dataclass(frozen=True)
class Fmi3Artifact:
    library_path: Path
    model_description: Path
    source_dir: Path
    instantiation_token: str
    variables: dict[str, int]


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
        "--plant-backend",
        choices=("fmi3", "rumoca"),
        default="fmi3",
        help="FMI 3 Co-Simulation plant (default) or interpreted Rumoca scenario",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="optional simulation duration override for local debugging",
    )
    parser.add_argument("--startup-timeout-s", type=float, default=6.0)
    parser.add_argument("--shutdown-timeout-s", type=float, default=4.0)
    parser.add_argument(
        "--sim-speed",
        type=float,
        default=1000.0,
        help="simulation rate relative to wall time for native_sim and the compiled FMI plant",
    )
    parser.add_argument(
        "--lockstep-check-target-s",
        type=float,
        default=1.0,
        help="lockstep advance target, in seconds, used for the native_sim boot-time regression",
    )
    parser.add_argument(
        "--lockstep-check-tolerance-s",
        type=float,
        default=0.05,
        help="allowed Zephyr timestamp error for the lockstep boot-time acknowledgement",
    )
    parser.add_argument(
        "--lockstep-regression-only",
        action="store_true",
        help="run only the short native_sim lockstep timing regression and skip flight checks",
    )
    return parser.parse_args()


def tail(path: Path, line_count: int = 100) -> str:
    if not path.exists():
        return f"{path} does not exist"
    lines = path.read_text(errors="replace").splitlines()
    return "\n".join(lines[-line_count:])


def start_process(
    name: str,
    argv: list[str],
    log_path: Path,
    *,
    env: dict[str, str] | None = None,
) -> subprocess.Popen[bytes]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("wb")
    try:
        process = subprocess.Popen(
            argv,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=env,
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


def build_fmi3_plant(artifact_dir: Path, log_path: Path) -> Fmi3Artifact:
    import rumoca as rum

    generated_dir = artifact_dir / "rumoca-fmi3"
    shutil.rmtree(generated_dir, ignore_errors=True)
    generated_dir.mkdir(parents=True, exist_ok=True)
    source_roots = [
        WORKSPACE_ROOT / "models" / "vendor" / "CMM-v0.0.2",
        ROOT / "models" / "plant",
    ]
    missing = [path for path in source_roots if not path.exists()]
    if missing:
        raise FileNotFoundError(f"FMI plant source root is missing: {missing[0]}")

    previous_modelica_path = os.environ.get("MODELICAPATH")
    os.environ["MODELICAPATH"] = os.pathsep.join(os.fspath(path) for path in source_roots)
    try:
        generated = rum.Session().codegen_file(
            os.fspath(FMI_MODEL_FILE),
            FMI_MODEL_NAME,
            "fmi3",
            os.fspath(generated_dir),
        )
    finally:
        if previous_modelica_path is None:
            os.environ.pop("MODELICAPATH", None)
        else:
            os.environ["MODELICAPATH"] = previous_modelica_path

    build_script = generated_dir / "build.sh"
    if not build_script.exists():
        raise RuntimeError("Rumoca FMI 3 target did not generate build.sh")
    command = ["sh", os.fspath(build_script)]
    build_env = os.environ.copy()
    build_env["CFLAGS"] = (
        "-O3 -DRUMOCA_FMI3_COSIM_FIXED_RK4 "
        "-DRUMOCA_FMI3_COSIM_RK4_MAX_STEP=4.0e-3"
    )
    completed = subprocess.run(
        command,
        cwd=generated_dir,
        env=build_env,
        capture_output=True,
        text=True,
    )
    log_path.write_text(
        "Rumoca FMI 3 Co-Simulation backend\n"
        + "generated:\n"
        + "\n".join(f"  {path}" for path in generated)
        + "\ncommand:\n  "
        + " ".join(command)
        + "\nCFLAGS:\n  "
        + build_env["CFLAGS"]
        + "\nstdout:\n"
        + completed.stdout
        + "\nstderr:\n"
        + completed.stderr
    )
    if completed.returncode != 0:
        raise RuntimeError(f"Rumoca FMI 3 build failed\n\n{tail(log_path)}")

    libraries = sorted((generated_dir / "binaries").glob(f"**/{FMI_MODEL_NAME}.so"))
    if len(libraries) != 1:
        raise RuntimeError(f"expected one packaged FMI shared library, found {len(libraries)}")
    fmu = generated_dir / f"{FMI_MODEL_NAME}.fmu"
    if not fmu.exists():
        raise RuntimeError(f"Rumoca FMI 3 build did not create {fmu}")
    model_description = generated_dir / "modelDescription.xml"
    root = ET.parse(model_description).getroot()
    model_variables = root.find("ModelVariables")
    if model_variables is None:
        raise RuntimeError("FMI modelDescription has no ModelVariables")
    variables = {
        variable.attrib["name"]: int(variable.attrib["valueReference"])
        for variable in model_variables
        if variable.tag == "Float64"
    }
    required = (
        *(f"pwm{index}_us" for index in range(9)),
        *TRACE_OUTPUT_NAMES,
        *ODOMETRY_OUTPUT_NAMES,
    )
    missing_variables = [name for name in required if name not in variables]
    if missing_variables:
        raise RuntimeError(f"FMI modelDescription is missing variable {missing_variables[0]}")
    return Fmi3Artifact(
        library_path=libraries[0],
        model_description=model_description,
        source_dir=generated_dir / "sources",
        instantiation_token=root.attrib["instantiationToken"],
        variables=variables,
    )


def write_fmi3_runner_config(artifact: Fmi3Artifact, path: Path) -> None:
    input_names = tuple(f"pwm{index}_us" for index in range(9))
    lines = [
        f"token={artifact.instantiation_token}",
        "input_vrs=" + ",".join(str(artifact.variables[name]) for name in input_names),
        "trace_names=" + ",".join(TRACE_OUTPUT_NAMES),
        "trace_vrs=" + ",".join(str(artifact.variables[name]) for name in TRACE_OUTPUT_NAMES),
        "odometry_vrs="
        + ",".join(str(artifact.variables[name]) for name in ODOMETRY_OUTPUT_NAMES),
    ]
    path.write_text("\n".join(lines) + "\n")


def build_fmi3_runner(
    artifact_dir: Path, sim: Path, artifact: Fmi3Artifact, log_path: Path
) -> Path:
    source = ROOT / "tests" / "zephyr" / "fmi3_lockstep_runner.c"
    synapse_include = synapse_c_root_for_sim(sim) / "include"
    if not synapse_include.exists():
        raise FileNotFoundError(f"Synapse C headers not found: {synapse_include}")
    function_types = artifact.source_dir / "fmi3FunctionTypes.h"
    if not function_types.exists():
        raise FileNotFoundError(f"Rumoca FMI 3 headers not found: {function_types}")

    runner = artifact_dir / "fmi3-lockstep-runner"
    command = [
        os.environ.get("CC", "cc"),
        "-O3",
        "-std=gnu11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{artifact.source_dir}",
        f"-I{synapse_include}",
        f"-I{ROOT / 'subsys' / 'lockstep'}",
        os.fspath(source),
        "-o",
        os.fspath(runner),
        "-ldl",
        "-lm",
    ]
    completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    log_path.write_text(
        "command:\n  "
        + " ".join(shlex.quote(part) for part in command)
        + "\nstdout:\n"
        + completed.stdout
        + "\nstderr:\n"
        + completed.stderr
    )
    if completed.returncode != 0:
        raise RuntimeError(f"FMI 3 lockstep runner build failed\n\n{tail(log_path)}")
    return runner


def load_fmi3_runner_metrics(path: Path, logs: BridgeLog) -> None:
    values: dict[str, float] = {}
    for line in path.read_text().splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key] = float(value)
    logs.plant_step_wall_s = values.get("plant_step_wall_s", 0.0)
    logs.plant_simulated_s = values.get("plant_simulated_s", 0.0)

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


def synapse_c_root_for_sim(sim: Path) -> Path:
    configured = os.environ.get("CUBS2_SYNAPSE_C_ROOT")
    if configured:
        root = Path(configured).expanduser()
        return root if root.is_absolute() else ROOT / root
    return sim.parent.parent / "_deps" / "synapse_fbs_c-src"


def synapse_bfbs_for_sim(sim: Path) -> Path:
    bfbs = synapse_c_root_for_sim(sim) / "bfbs" / "all.bfbs"
    if not bfbs.exists():
        raise FileNotFoundError(
            f"Synapse schema BFBS not found: {bfbs}\n"
            "Generate the workspace Synapse packages or build native_sim so csyn can "
            "unpack the pinned synapse_fbs release."
        )
    return bfbs


def scenario_for_run(scenario: Path, artifact_dir: Path, t_end: float | None, synapse_bfbs: Path) -> Path:
    text = scenario.read_text()
    replacements = {
        'file = "Cubs2NativeSimSIL.mo"': f'file = "{(ROOT / "tests" / "zephyr" / "Cubs2NativeSimSIL.mo").as_posix()}"',
        '"../../../models/vendor/CMM-v0.0.2"': f'"{(WORKSPACE_ROOT / "models" / "vendor" / "CMM-v0.0.2").as_posix()}"',
        '"../../models/plant"': f'"{(ROOT / "models" / "plant").as_posix()}"',
        'output = "artifacts/native-sim-sil/native-sim-rumoca.html"': f'output = "{(artifact_dir / "native-sim-rumoca.html").as_posix()}"',
        'bfbs = ["build-native_sim/_deps/synapse_fbs_c-src/bfbs/all.bfbs"]': f'bfbs = ["{synapse_bfbs.as_posix()}"]',
        'path = "artifacts/native-sim-sil/native-sim-plant.csv"': f'path = "{(artifact_dir / "native-sim-plant.csv").as_posix()}"',
    }

    updated = text
    for old, new in replacements.items():
        if old not in updated:
            raise RuntimeError(f"could not rewrite {old!r} in {scenario}")
        updated = updated.replace(old, new, 1)

    if t_end is not None:
        updated, count = re.subn(r"(?m)^t_end\s*=\s*[0-9.]+", f"t_end = {t_end}", updated, count=1)
        if count != 1:
            raise RuntimeError(f"could not override t_end in {scenario}")

    path = artifact_dir / scenario.name
    path.write_text(updated)
    return path


def fixed_struct_payload(data: object, size: int) -> bytes:
    table = getattr(data, "_tab", None)
    if table is None:
        raise ValueError(f"{type(data).__name__} is not initialized")
    end = table.Pos + size
    if len(table.Bytes) < end:
        raise ValueError(f"{type(data).__name__} expected {size} bytes at {table.Pos}, got {len(table.Bytes)}")
    return bytes(table.Bytes[table.Pos:end])


def decode_external_odometry(payload: bytes) -> ExternalOdometryData:
    data = ExternalOdometry.GetRootAs(payload).Data()
    if data is None:
        raise ValueError("ExternalOdometry payload has no data field")
    return data


def odometry_contract() -> str:
    info = topic_by_name("Odometry")
    if info is None or not info.fixed_layout:
        raise RuntimeError("synapse_fbs catalog has no fixed-layout Odometry topic")
    return (
        f"application/x-synapse-struct;type={info.wire_type};"
        f"schema=sha256-128:{info.schema_hash}"
    )


def encode_odometry(data: ExternalOdometryData) -> bytes:
    position = data.PositionEnuM(Vec3f())
    attitude = data.Attitude(Quaternionf())
    velocity = data.LinearVelocityEnuMS(Vec3f())
    rates = data.AngularVelocityFluRadS(RateTriplet())
    builder = flatbuffers.Builder(128)

    OdometryStart(builder)
    offset = CreateOdometryData(
        builder,
        data.TimestampUs(),
        position.X(),
        position.Y(),
        position.Z(),
        attitude.W(),
        attitude.X(),
        attitude.Y(),
        attitude.Z(),
        velocity.X(),
        velocity.Y(),
        velocity.Z(),
        rates.Roll(),
        rates.Pitch(),
        rates.Yaw(),
        data.Flags(),
        data.Status(),
        data.Id(),
        data.SourceId(),
        100,
    )
    OdometryAddData(builder, offset)
    root = OdometryEnd(builder)
    builder.Finish(root)
    odometry = Odometry.GetRootAs(builder.Output()).Data()
    if odometry is None:
        raise RuntimeError("generated Odometry builder produced no payload")
    return fixed_struct_payload(odometry, OdometryData.SizeOf())


def tracking_valid(flags: int, status: int) -> bool:
    return (
        (flags & EXTERNAL_ODOMETRY_VALID_FLAGS) == EXTERNAL_ODOMETRY_VALID_FLAGS
        and (flags & ExternalOdometryFlags.Lost) == 0
        and status != ExternalOdometryStatus.Lost
    )


def external_odometry_row(data: ExternalOdometryData) -> dict[str, float | int | bool]:
    position = data.PositionEnuM(Vec3f())
    attitude = data.Attitude(Quaternionf())
    velocity = data.LinearVelocityEnuMS(Vec3f())
    rates = data.AngularVelocityFluRadS(RateTriplet())
    flags = int(data.Flags())
    status = int(data.Status())
    timestamp_us = int(data.TimestampUs())
    return {
        "sim_time_s": timestamp_us / 1_000_000.0,
        "timestamp_us": timestamp_us,
        "x_m": float(position.X()),
        "y_m": float(position.Y()),
        "z_m": float(position.Z()),
        "qw": float(attitude.W()),
        "qx": float(attitude.X()),
        "qy": float(attitude.Y()),
        "qz": float(attitude.Z()),
        "vx_m_s": float(velocity.X()),
        "vy_m_s": float(velocity.Y()),
        "vz_m_s": float(velocity.Z()),
        "roll_rate_rad_s": float(rates.Roll()),
        "pitch_rate_rad_s": float(rates.Pitch()),
        "yaw_rate_rad_s": float(rates.Yaw()),
        "flags": flags,
        "status": status,
        "source_id": int(data.SourceId()),
        "id": int(data.Id()),
        "tracking_valid": tracking_valid(flags, status),
    }


def decode_pwm_outputs(payload: bytes, sim_time_s: float) -> dict[str, float | int]:
    if len(payload) != PwmSignalOutputsData.SizeOf():
        raise ValueError(f"expected {PwmSignalOutputsData.SizeOf()} PWM bytes, got {len(payload)}")
    data = PwmSignalOutputsData()
    data.Init(payload, 0)
    row: dict[str, float | int] = {
        "sim_time_s": sim_time_s,
        "timestamp_us": int(data.TimestampUs()),
        "active_mask": int(data.ActiveMask()),
        "port": int(data.Port()),
    }
    row.update({f"output{i}_us": int(getattr(data, f"Output{i}Us")()) for i in range(16)})
    return row


def pack_rumoca_pwm_outputs(row: dict[str, float | int]) -> bytes:
    payload = struct.pack(
        PWM_STRUCT_FORMAT,
        int(row["timestamp_us"]),
        int(row["active_mask"]),
        int(row["port"]),
        *(int(row[f"output{idx}_us"]) for idx in range(16)),
    )
    if len(payload) != PwmSignalOutputsData.SizeOf():
        raise ValueError(f"expected {PwmSignalOutputsData.SizeOf()} PWM struct bytes, got {len(payload)}")

    # Rumoca's FlatBuffers codec expects its deterministic
    # table-with-inline-struct layout, which is larger than Python
    # flatbuffers' compact builder output for this schema.
    table = bytearray(RUMOCA_PWM_TABLE_SIZE)
    struct.pack_into("<I", table, 0, 12)
    struct.pack_into("<HHH", table, 4, 6, 56, 8)
    struct.pack_into("<I", table, 12, 8)
    table[RUMOCA_PWM_STRUCT_OFFSET : RUMOCA_PWM_STRUCT_OFFSET + len(payload)] = payload
    return bytes(table)


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
    if len(payload) != AttitudeCommandData.SizeOf():
        raise ValueError(f"expected {AttitudeCommandData.SizeOf()} attitude-command bytes, got {len(payload)}")
    data = AttitudeCommandData()
    data.Init(payload, 0)
    attitude = data.Attitude(Quaternionf())
    rates = data.BodyRateFluRadS(RateTriplet())
    roll, pitch, yaw = euler_from_quat(attitude.W(), attitude.X(), attitude.Y(), attitude.Z())
    return {
        "sim_time_s": sim_time_s,
        "timestamp_us": int(data.TimestampUs()),
        "roll_cmd_rad": roll,
        "pitch_cmd_rad": pitch,
        "yaw_cmd_rad": yaw,
        "rate_roll_cmd_rad_s": float(rates.Roll()),
        "rate_pitch_cmd_rad_s": float(rates.Pitch()),
        "rate_yaw_cmd_rad_s": float(rates.Yaw()),
        "thrust_cmd": float(data.Thrust()),
        "type_mask": int(data.TypeMask()),
    }


def payload_bytes(sample: object) -> bytes:
    return bytes(sample.payload)


def bridge_topics(locator: str, stop: threading.Event, logs: BridgeLog, startup_timeout_s: float) -> None:
    session: zenoh.Session | None = None
    try:
        session = open_zenoh_session(locator, startup_timeout_s)
        odometry_subscriber = session.declare_subscriber(RUMOCA_EXTERNAL_ODOMETRY_TOPIC)
        pwm_subscriber = session.declare_subscriber(SYNAPSE_PWM_TOPIC)
        attitude_subscriber = session.declare_subscriber(SYNAPSE_ATTITUDE_COMMAND_TOPIC)
        wall_start = time.perf_counter()
        latest_sim_time_s = 0.0
        odometry_forwarded = False
        control_forwarded = False
        bridge_seq = 0

        while not stop.is_set():
            did_work = False

            while True:
                sample = odometry_subscriber.try_recv()
                if sample is None:
                    break
                odometry = decode_external_odometry(payload_bytes(sample))
                odometry_row = external_odometry_row(odometry)
                odometry_row["bridge_wall_s"] = time.perf_counter() - wall_start
                latest_sim_time_s = float(odometry_row["sim_time_s"])
                bridge_seq += 1
                odometry_row["bridge_seq"] = bridge_seq
                session.put(
                    SYNAPSE_ODOMETRY_TOPIC,
                    encode_odometry(odometry),
                    encoding=odometry_contract(),
                )
                odometry_forwarded = True
                logs.odometry_rows.append(odometry_row)
                did_work = True

            while True:
                sample = pwm_subscriber.try_recv()
                if sample is None:
                    break
                row = decode_pwm_outputs(payload_bytes(sample), latest_sim_time_s)
                row["bridge_wall_s"] = time.perf_counter() - wall_start
                bridge_seq += 1
                row["bridge_seq"] = bridge_seq
                row["lockstep_timestamp_us"] = int(round(latest_sim_time_s * 1_000_000.0))
                real_control = int(row["output2_us"]) > 1100 or int(row["output6_us"]) > 1000
                forward_to_plant = control_forwarded or (odometry_forwarded and real_control)
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
                row = decode_attitude_command(payload_bytes(sample), latest_sim_time_s)
                row["bridge_wall_s"] = time.perf_counter() - wall_start
                bridge_seq += 1
                row["bridge_seq"] = bridge_seq
                row["lockstep_timestamp_us"] = int(round(latest_sim_time_s * 1_000_000.0))
                logs.attitude_rows.append(row)
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


def nearest_rows(
    rows: list[dict[str, float | int]], time_key: str = "sim_time_s"
) -> Callable[[float], dict[str, float | int]]:
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
        if previous == len(ROUTE_WAYPOINTS) - 1 and current == 1:
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
        "external_odometry_samples": len(logs.odometry_rows),
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


def lockstep_metrics(logs: BridgeLog, target_s: float, tolerance_s: float) -> dict[str, float | int]:
    target_us = int(round(target_s * 1_000_000.0))
    tolerance_us = int(round(tolerance_s * 1_000_000.0))
    target_odometry = next(
        (row for row in logs.odometry_rows if int(row["timestamp_us"]) >= target_us),
        None,
    )

    simulated_s = float(logs.odometry_rows[-1]["sim_time_s"]) if logs.odometry_rows else 0.0
    wall_s = float(logs.odometry_rows[-1].get("bridge_wall_s", 0.0)) if logs.odometry_rows else 0.0
    metrics: dict[str, float | int] = {
        "lockstep_simulated_s": simulated_s,
        "lockstep_wall_s": wall_s,
        "lockstep_speed_x": simulated_s / wall_s if wall_s > 0.0 else 0.0,
        "plant_step_wall_s": logs.plant_step_wall_s,
        "plant_step_speed_x": (
            logs.plant_simulated_s / logs.plant_step_wall_s
            if logs.plant_step_wall_s > 0.0
            else 0.0
        ),
        "lockstep_target_s": target_s,
        "lockstep_tolerance_s": tolerance_s,
        "lockstep_target_us": target_us,
        "lockstep_target_advance_seen": int(target_odometry is not None),
        "lockstep_target_advance_bridge_seq": 0,
        "lockstep_pre_target_pwm_samples": 0,
        "lockstep_pre_target_max_boot_s": 0.0,
        "lockstep_first_post_target_pwm_boot_s": 0.0,
        "lockstep_ack_seen": 0,
        "lockstep_ack_boot_s": 0.0,
        "lockstep_ack_error_s": float("nan"),
    }

    if target_odometry is None:
        return metrics

    target_seq = int(target_odometry.get("bridge_seq", 0))
    metrics["lockstep_target_advance_bridge_seq"] = target_seq

    pre_target_pwm = [
        row
        for row in logs.pwm_rows
        if int(row.get("bridge_seq", 0)) < target_seq
        and float(row.get("sim_time_s", 0.0)) < target_s
    ]
    if pre_target_pwm:
        max_boot_us = max(int(row["timestamp_us"]) for row in pre_target_pwm)
        metrics["lockstep_pre_target_pwm_samples"] = len(pre_target_pwm)
        metrics["lockstep_pre_target_max_boot_s"] = max_boot_us / 1_000_000.0

    post_target_pwm = [
        row
        for row in logs.pwm_rows
        if int(row.get("forwarded_to_plant", 0)) != 0
        and int(row.get("bridge_seq", 0)) > target_seq
    ]
    if post_target_pwm:
        metrics["lockstep_first_post_target_pwm_boot_s"] = (
            int(post_target_pwm[0]["timestamp_us"]) / 1_000_000.0
        )

    ack = next(
        (row for row in post_target_pwm if int(row["timestamp_us"]) >= target_us - tolerance_us),
        None,
    )
    if ack is None:
        return metrics

    ack_us = int(ack["timestamp_us"])
    metrics["lockstep_ack_seen"] = 1
    metrics["lockstep_ack_boot_s"] = ack_us / 1_000_000.0
    metrics["lockstep_ack_error_s"] = (ack_us - target_us) / 1_000_000.0
    return metrics


def run_lockstep_checks(metrics: dict[str, float | int]) -> list[tuple[str, bool, str]]:
    target_s = float(metrics["lockstep_target_s"])
    tolerance_s = float(metrics["lockstep_tolerance_s"])
    ack_error_s = float(metrics["lockstep_ack_error_s"])
    ack_error_ok = math.isfinite(ack_error_s) and abs(ack_error_s) <= tolerance_s
    pre_target_max_boot_s = float(metrics["lockstep_pre_target_max_boot_s"])
    pre_target_ok = pre_target_max_boot_s <= target_s + tolerance_s

    return [
        (
            "lockstep target advance observed",
            int(metrics["lockstep_target_advance_seen"]) != 0,
            f"target {target_s:.3f} s",
        ),
        (
            "lockstep pre-target boot bounded",
            pre_target_ok,
            (
                f"max boot {pre_target_max_boot_s:.3f} s before target, "
                f"tolerance {tolerance_s:.3f} s"
            ),
        ),
        (
            "lockstep ack received",
            int(metrics["lockstep_ack_seen"]) != 0,
            f"ack boot {float(metrics['lockstep_ack_boot_s']):.3f} s",
        ),
        (
            "lockstep ack boot time",
            ack_error_ok,
            f"error {ack_error_s:.6f} s, tolerance {tolerance_s:.3f} s",
        ),
    ]


def run_checks(metrics: dict[str, float | int]) -> list[tuple[str, str, str]]:
    def warn_if(ok: bool) -> str:
        return "PASS" if ok else "WARN"

    checks = [
        (
            "external odometry published",
            int(metrics["external_odometry_samples"]) > 100,
            f"{metrics['external_odometry_samples']} samples",
        ),
        ("pwm received", int(metrics["pwm_samples"]) > 50, f"{metrics['pwm_samples']} samples"),
        (
            "attitude command received",
            int(metrics["attitude_command_samples"]) > 50,
            f"{metrics['attitude_command_samples']} samples",
        ),
        ("takeoff altitude", float(metrics["max_altitude_m"]) > 1.5, f"max {metrics['max_altitude_m']:.2f} m"),
        (
            "route laps",
            int(metrics["laps"]) >= 1,
            f"{metrics['laps']} laps",
        ),
        (
            "altitude tracking",
            warn_if(float(metrics["mean_abs_altitude_error_m"]) < 1.5),
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
            float(metrics["p95_abs_crosstrack_m"]) < 15.0,
            f"p95 abs {metrics['p95_abs_crosstrack_m']:.2f} m",
        ),
        ("bank bounded", float(metrics["max_abs_bank_deg"]) < 80.0, f"max abs {metrics['max_abs_bank_deg']:.1f} deg"),
        ("pitch bounded", float(metrics["max_abs_pitch_deg"]) < 60.0, f"max abs {metrics['max_abs_pitch_deg']:.1f} deg"),
    ]
    checks.extend(run_lockstep_checks(metrics))

    rendered = []
    for name, status_or_ok, detail in checks:
        if isinstance(status_or_ok, str):
            status = status_or_ok
        else:
            status = "PASS" if status_or_ok else "FAIL"
        rendered.append((name, status, detail))
    return rendered


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
        "This run uses the Zephyr `native_sim` binary, a Rumoca/CMM SportCub plant, and fixed-layout Synapse odometry, PWM, and attitude payloads. The FMI path keeps the lockstep loop in compiled C; the interpreted reference backend uses the routed integration path.",
        "",
        "With `--plant-backend rumoca`, traffic can be inspected while the test runs:",
        "",
        "```sh",
        "csyn --connect udp/127.0.0.1:7447 topic echo odom",
        "csyn --connect udp/127.0.0.1:7447 topic hz pwm",
        "csyn --connect udp/127.0.0.1:7447 topic echo att_sp",
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
            "CI gates transport, lockstep timing, exact route-lap completion, route tracking, generated traces, and bounded-flight checks; remaining `WARN` rows are diagnostic.",
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
    if plot_paths:
        html_lines.append("</table><h2>Plots</h2>")
        for path in plot_paths:
            html_lines.append(f"<h3>{html.escape(path.name)}</h3><img src='{image_data_uri(path)}' alt='{html.escape(path.name)}'>")
    else:
        html_lines.append("</table>")
    html_lines.append("</body></html>")
    (artifact_dir / "native-sim-report.html").write_text("\n".join(html_lines))


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.sim_speed) or args.sim_speed <= 0.0:
        raise ValueError("--sim-speed must be a positive finite number")
    if args.t_end is not None and (not math.isfinite(args.t_end) or args.t_end <= 0.0):
        raise ValueError("--t-end must be a positive finite number")
    if not math.isfinite(args.startup_timeout_s) or args.startup_timeout_s <= 0.0:
        raise ValueError("--startup-timeout-s must be a positive finite number")
    if not math.isfinite(args.shutdown_timeout_s) or args.shutdown_timeout_s <= 0.0:
        raise ValueError("--shutdown-timeout-s must be a positive finite number")
    if not math.isfinite(args.lockstep_check_target_s) or args.lockstep_check_target_s <= 0.0:
        raise ValueError("--lockstep-check-target-s must be a positive finite number")
    if (
        not math.isfinite(args.lockstep_check_tolerance_s)
        or args.lockstep_check_tolerance_s < 0.0
    ):
        raise ValueError("--lockstep-check-tolerance-s must be a finite non-negative number")
    if args.lockstep_regression_only and args.t_end is None:
        args.t_end = max(1.25, args.lockstep_check_target_s + 0.25)

    artifact_dir = (ROOT / args.artifacts).resolve() if not Path(args.artifacts).is_absolute() else Path(args.artifacts)
    artifact_dir.mkdir(parents=True, exist_ok=True)

    sim = Path(args.sim)
    if not sim.is_absolute():
        sim = ROOT / sim
    if not sim.exists():
        raise FileNotFoundError(f"native_sim executable not found: {sim}")

    router_log = artifact_dir / "zenohd.log"
    sim_log = artifact_dir / "native-sim.log"
    rumoca_log = artifact_dir / "rumoca.log"
    fmi3_runner_build_log = artifact_dir / "fmi3-runner-build.log"
    fmi3_runner_log = artifact_dir / "fmi3-runner.log"
    fmi3_runner_config = artifact_dir / "fmi3-runner.conf"
    fmi3_runner_metrics = artifact_dir / "fmi3-runner.metrics"
    fmi3_shared_memory = artifact_dir / "fmi3-lockstep.shm"
    plant_csv = artifact_dir / "native-sim-plant.csv"
    odometry_csv = artifact_dir / "native-sim-external-odometry.csv"
    pwm_csv = artifact_dir / "native-sim-pwm.csv"
    attitude_csv = artifact_dir / "native-sim-attitude-command.csv"
    merged_csv = artifact_dir / "native-sim-flight.csv"

    effective_t_end = args.t_end if args.t_end is not None else DEFAULT_T_END
    rumoca_cmd: list[str] | None = None
    fmi3_plant: Fmi3Artifact | None = None
    fmi3_runner: Path | None = None
    if args.plant_backend == "fmi3":
        fmi3_plant = build_fmi3_plant(artifact_dir, rumoca_log)
        write_fmi3_runner_config(fmi3_plant, fmi3_runner_config)
        fmi3_runner = build_fmi3_runner(
            artifact_dir, sim, fmi3_plant, fmi3_runner_build_log
        )
        with fmi3_shared_memory.open("wb") as shared:
            shared.truncate(NATIVE_SIL_SHARED_SIZE)
            shared.write(struct.pack("<I", NATIVE_SIL_SHARED_MAGIC))
    else:
        run_checked([sys.executable, "-c", RUMOCA_SCENARIO_CHECK_CODE])
        scenario = Path(args.scenario)
        if not scenario.is_absolute():
            scenario = ROOT / scenario
        if not scenario.exists():
            raise FileNotFoundError(f"Rumoca scenario not found: {scenario}")
        synapse_bfbs = synapse_bfbs_for_sim(sim)
        scenario_to_run = scenario_for_run(scenario, artifact_dir, args.t_end, synapse_bfbs)
        rumoca_cmd = [sys.executable, "-c", RUMOCA_RUN_SCENARIO_CODE, os.fspath(scenario_to_run)]

    router: subprocess.Popen[bytes] | None = None
    zephyr: subprocess.Popen[bytes] | None = None
    rumoca_process: subprocess.Popen[bytes] | None = None
    fmi3_runner_process: subprocess.Popen[bytes] | None = None
    stop_bridge = threading.Event()
    bridge_log = BridgeLog(odometry_rows=[], pwm_rows=[], attitude_rows=[])
    bridge_thread: threading.Thread | None = None

    try:
        router = start_process("zenohd", ["zenohd", "-l", args.locator], router_log)
        time.sleep(0.5)
        require_running(router, router_log, "zenohd")

        sim_cmd = [os.fspath(sim), f"-rt-ratio={args.sim_speed:g}"]
        sim_env = None
        if fmi3_plant is not None:
            sim_env = os.environ.copy()
            sim_env["CUBS2_NATIVE_SIL_SHM"] = os.fspath(fmi3_shared_memory)
            if args.sim_speed <= 1.0:
                sim_env["CUBS2_NATIVE_SIL_COOPERATIVE"] = "1"
        zephyr = start_process("native_sim", sim_cmd, sim_log, env=sim_env)
        time.sleep(0.3)
        require_running(zephyr, sim_log, "native_sim")

        if fmi3_plant is None:
            bridge_thread = threading.Thread(
                target=bridge_topics,
                args=(args.locator, stop_bridge, bridge_log, args.startup_timeout_s),
                name="native-sil-bridge",
                daemon=True,
            )
            bridge_thread.start()
        else:
            assert fmi3_runner is not None
            fmi3_runner_process = start_process(
                "fmi3-runner",
                [
                    os.fspath(fmi3_runner),
                    os.fspath(fmi3_plant.library_path),
                    os.fspath(fmi3_runner_config),
                    f"shm:{fmi3_shared_memory}",
                    f"{effective_t_end:.17g}",
                    f"{args.startup_timeout_s:.17g}",
                    f"{args.sim_speed:.17g}",
                    os.fspath(plant_csv),
                    os.fspath(odometry_csv),
                    os.fspath(pwm_csv),
                    os.fspath(attitude_csv),
                    os.fspath(fmi3_runner_metrics),
                ],
                fmi3_runner_log,
            )

        if rumoca_cmd is not None:
            rumoca_process = start_process("rumoca", rumoca_cmd, rumoca_log)
            while rumoca_process.poll() is None:
                require_running(router, router_log, "zenohd")
                require_running(zephyr, sim_log, "native_sim")
                if bridge_log.error is not None:
                    raise RuntimeError(f"native SIL bridge failed: {bridge_log.error}")
                time.sleep(0.1)
            if rumoca_process.returncode != 0:
                raise RuntimeError(
                    f"Rumoca scenario runner exited with status {rumoca_process.returncode}"
                    f"\n\n{tail(rumoca_log)}"
                )
        elif fmi3_runner_process is not None:
            while fmi3_runner_process.poll() is None:
                require_running(router, router_log, "zenohd")
                require_running(zephyr, sim_log, "native_sim")
                time.sleep(0.01)
            if fmi3_runner_process.returncode != 0:
                raise RuntimeError(
                    "FMI 3 runner exited with status "
                    f"{fmi3_runner_process.returncode}\n\n{tail(fmi3_runner_log)}"
                )
        else:
            assert bridge_thread is not None
            while bridge_thread.is_alive():
                require_running(router, router_log, "zenohd")
                require_running(zephyr, sim_log, "native_sim")
                if bridge_log.error is not None:
                    raise RuntimeError(f"native SIL FMI loop failed: {bridge_log.error}")
                time.sleep(0.01)

        stop_bridge.set()
        if bridge_thread is not None:
            bridge_thread.join(timeout=args.shutdown_timeout_s)
            if bridge_thread.is_alive():
                raise RuntimeError("native SIL bridge did not stop before the shutdown timeout")

        if bridge_log.error is not None:
            raise RuntimeError(f"native SIL plant loop failed: {bridge_log.error}")

        if fmi3_plant is not None:
            bridge_log.odometry_rows = load_csv(odometry_csv)
            bridge_log.pwm_rows = load_csv(pwm_csv)
            bridge_log.attitude_rows = load_csv(attitude_csv)
            load_fmi3_runner_metrics(fmi3_runner_metrics, bridge_log)
        else:
            write_csv(odometry_csv, bridge_log.odometry_rows)
            write_csv(pwm_csv, bridge_log.pwm_rows)
            write_csv(attitude_csv, bridge_log.attitude_rows)

        lockstep = lockstep_metrics(
            bridge_log,
            args.lockstep_check_target_s,
            args.lockstep_check_tolerance_s,
        )

        if args.lockstep_regression_only:
            checks = []
            for name, ok, detail in run_lockstep_checks(lockstep):
                checks.append((name, "PASS" if ok else "FAIL", detail))
            write_reports(artifact_dir, lockstep, checks, [])
            failed = [f"{name}: {detail}" for name, status, detail in checks if status == "FAIL"]
            if failed:
                raise RuntimeError("native SIL lockstep checks failed:\n- " + "\n- ".join(failed))

            print(f"wrote {odometry_csv}")
            print(f"wrote {pwm_csv}")
            print(f"wrote {artifact_dir / 'native-sim-summary.md'}")
            print(f"wrote {artifact_dir / 'native-sim-report.html'}")
            return 0

        if not plant_csv.exists():
            raise RuntimeError(f"Rumoca plant trace was not written: {plant_csv}\n\n{tail(rumoca_log)}")

        plant_rows = load_csv(plant_csv)
        if not plant_rows:
            raise RuntimeError(f"Rumoca plant trace is empty: {plant_csv}\n\n{tail(rumoca_log)}")

        merged_rows = merge_flight_rows(plant_rows, bridge_log.pwm_rows, bridge_log.attitude_rows)
        write_merged_csv(merged_csv, merged_rows)
        plot_paths = plot_flight(merged_rows, artifact_dir)
        metrics = flight_metrics(merged_rows, bridge_log)
        metrics.update(lockstep)
        checks = run_checks(metrics)
        write_reports(artifact_dir, metrics, checks, plot_paths)

        failed = [f"{name}: {detail}" for name, status, detail in checks if status == "FAIL"]
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
        stop_process(fmi3_runner_process, args.shutdown_timeout_s)
        stop_process(rumoca_process, args.shutdown_timeout_s)
        stop_process(zephyr, args.shutdown_timeout_s)
        stop_process(router, args.shutdown_timeout_s)
        if fmi3_plant is not None:
            fmi3_shared_memory.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
