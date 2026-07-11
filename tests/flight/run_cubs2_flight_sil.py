#!/usr/bin/env python3
"""Run the CUBS2 staged flight SIL test through the Rumoca Python binding."""

from __future__ import annotations

import argparse
import base64
import csv
from dataclasses import dataclass
import html
import math
from pathlib import Path
import sys
import tomllib
from typing import Callable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rumoca as rum


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_DIR = ROOT / "artifacts" / "flight"
SCENARIO_DIR = ROOT / "tests" / "flight"
PATTERN_WAYPOINTS = [
    (0.0, 0.0, 3.0),
    (30.0, 0.0, 3.0),
    (30.0, 20.0, 3.0),
    (0.0, 20.0, 3.0),
]


@dataclass(frozen=True)
class ScenarioConfig:
    path: Path
    dt: float
    t_end: float
    output: Path


def stage_end_time(mode: str) -> float:
    return {"takeoff": 8.0, "altitude": 16.0, "heading": 24.0}.get(mode, 150.0)


def scenario_path(mode: str) -> Path:
    return SCENARIO_DIR / f"rumoca-scenario.{mode}.toml"


def load_scenario_config(mode: str) -> ScenarioConfig:
    path = scenario_path(mode)
    if not path.exists():
        return ScenarioConfig(path=path, dt=0.02, t_end=stage_end_time(mode), output=ARTIFACT_DIR / f"{mode}.csv")

    data = tomllib.loads(path.read_text(encoding="utf-8"))
    sim = data.get("sim", {})
    output = Path(sim.get("output", f"artifacts/flight/{mode}.csv"))
    if not output.is_absolute():
        output = ROOT / output
    return ScenarioConfig(
        path=path,
        dt=float(sim.get("dt", 0.02)),
        t_end=float(sim.get("t_end", stage_end_time(mode))),
        output=output,
    )


def csv_fields() -> list[str]:
    return [
        "time", "mode", "x", "y", "z", "roll", "pitch", "yaw", "airspeed",
        "stick_roll", "stick_pitch", "stick_yaw", "stick_throttle",
        "surface_ail", "surface_elev", "surface_rud", "surface_thr",
        "current_waypoint", "laps", "desired_heading", "desired_altitude", "desired_speed",
        "desired_flight_path_angle", "desired_acceleration", "heading",
        "course_error", "roll_command", "inner_roll_command", "pitch_command",
        "tecs_pitch_command", "tecs_thrust_command", "mission_phase",
        "cross_track_error", "remaining_along_track", "course_alignment_error",
    ]


def result_columns(result: rum.Result) -> dict[str, list[float]]:
    columns = {"time": [float(value) for value in result.time]}
    for name in result.names:
        columns[name] = [float(value) for value in result[name]]
    return columns


def sample(columns: dict[str, list[float]], index: int, *keys: str, default: float = 0.0) -> float:
    for key in keys:
        values = columns.get(key)
        if values is not None:
            return values[index]
    return default


def normalize_rumoca_result(result: rum.Result, mode: str) -> list[dict[str, float | str]]:
    columns = result_columns(result)
    rows: list[dict[str, float | str]] = []
    count = len(columns["time"])

    for index in range(count):
        vx = sample(columns, index, "vehicle.velocity[1]", "vehicle.v_w[1]", "vx_m_s")
        vy = sample(columns, index, "vehicle.velocity[2]", "vehicle.v_w[2]", "vy_m_s")
        heading = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-12 else 0.0
        desired_altitude = sample(
            columns,
            index,
            "outerLoop.pathAltitude",
            "outerLoop.guidance.pathAltitude",
            "targetAltitude_m",
            default=3.0 if mode in {"altitude", "heading", "pattern"} else sample(columns, index, "z_m"),
        )

        rows.append({
            "time": sample(columns, index, "time_s", "time"),
            "mode": mode,
            "x": sample(columns, index, "x_m", "vehicle.position[1]", "vehicle.p[1]"),
            "y": sample(columns, index, "y_m", "vehicle.position[2]", "vehicle.p[2]"),
            "z": sample(columns, index, "z_m", "vehicle.position[3]", "vehicle.p[3]"),
            "roll": sample(columns, index, "roll_rad", "euler_rad[1]", "outerLoop.euler_rad[1]"),
            "pitch": sample(columns, index, "pitch_rad", "euler_rad[2]", "outerLoop.euler_rad[2]"),
            "yaw": sample(columns, index, "yaw_rad", "euler_rad[3]", "outerLoop.euler_rad[3]"),
            "airspeed": sample(columns, index, "airspeed_m_s", "vehicle.airspeed", "vehicle.Vt"),
            "stick_roll": sample(columns, index, "innerLoop.stick_roll", "roll_cmd"),
            "stick_pitch": sample(columns, index, "innerLoop.stick_pitch", "pitch_cmd"),
            "stick_yaw": sample(columns, index, "innerLoop.stick_yaw"),
            "stick_throttle": sample(columns, index, "innerLoop.stick_throttle", "throttle_cmd"),
            "surface_ail": sample(columns, index, "vehicle.ail", "innerLoop.ail"),
            "surface_elev": sample(columns, index, "vehicle.elev", "innerLoop.elev"),
            "surface_rud": sample(columns, index, "vehicle.rud", "innerLoop.rud"),
            "surface_thr": sample(columns, index, "vehicle.thr", "innerLoop.thr"),
            "current_waypoint": sample(columns, index, "current_waypoint", "outerLoop.currentWaypoint", default=1.0),
            "laps": sample(columns, index, "laps", "lapCount"),
            "desired_heading": sample(columns, index, "desired_heading_rad", "outerLoop.desiredHeading", "outerLoop.guidance.setpoints.heading"),
            "desired_altitude": desired_altitude,
            "desired_speed": sample(columns, index, "desired_speed_m_s", "outerLoop.desiredSpeed"),
            "desired_flight_path_angle": sample(columns, index, "outerLoop.desiredFlightPathAngle", "outerLoop.guidance.setpoints.flightPathAngle", "flightPathAngleSetpoint"),
            "desired_acceleration": sample(columns, index, "outerLoop.desiredAcceleration", "outerLoop.guidance.setpoints.acceleration", "accelerationSetpoint"),
            "heading": heading,
            "course_error": sample(columns, index, "outerLoop.courseError", "outerLoop.attitude.courseError"),
            "roll_command": sample(columns, index, "outerLoop.rollCommand", "outerLoop.attitude.rollCommand", "roll_cmd"),
            "inner_roll_command": sample(columns, index, "innerLoop.phi_sp"),
            "pitch_command": sample(columns, index, "innerLoop.theta_sp", "pitch_cmd"),
            "tecs_pitch_command": sample(columns, index, "outerLoop.tecs.pitchCommand", "tecs.pitchCommand"),
            "tecs_thrust_command": sample(columns, index, "outerLoop.tecs.thrustCommand", "tecs.thrustCommand"),
            "mission_phase": sample(columns, index, "mission_phase", default=1.0),
            "cross_track_error": sample(columns, index, "cross_track_error_m", "outerLoop.crossTrackError", "outerLoop.guidance.crossTrackError"),
            "remaining_along_track": sample(columns, index, "remaining_along_track_m", "outerLoop.remainingAlongTrackDistance", "outerLoop.guidance.remainingAlongTrackDistance"),
            "course_alignment_error": sample(columns, index, "course_alignment_error_rad", "outerLoop.courseAlignmentError", "outerLoop.guidance.courseAlignmentError"),
        })

    return rows


def write_csv(path: Path, rows: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=csv_fields())
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {path}")


def run_rumoca_stage(
    mode: str,
    t_end: float | None = None,
    params: dict[str, float] | None = None,
) -> list[dict[str, float | str]]:
    scenario = load_scenario_config(mode)
    if not scenario.path.exists():
        raise FileNotFoundError(f"Rumoca scenario not found: {scenario.path}")

    print(f"simulate {scenario.path} with Rumoca Python binding", flush=True)
    _session, model, sim_config = rum.Session.from_scenario(str(scenario.path))
    result = model.simulate(
        t=(0.0, t_end if t_end is not None else scenario.t_end),
        config=sim_config,
        params=params,
    )
    rows = normalize_rumoca_result(result, mode)
    write_csv(scenario.output, rows)
    return rows


def f(row: dict[str, float | str], key: str) -> float:
    return float(row[key])


def max_abs(rows: list[dict[str, float | str]], key: str) -> float:
    return max(abs(f(row, key)) for row in rows)


def final(rows: list[dict[str, float | str]], key: str) -> float:
    return f(rows[-1], key)


def values(rows: list[dict[str, float | str]], key: str) -> list[float]:
    return [f(row, key) for row in rows]


def rms(rows: list[dict[str, float | str]], key: str) -> float:
    samples = values(rows, key)
    return math.sqrt(sum(sample * sample for sample in samples) / len(samples))


def percentile_abs(rows: list[dict[str, float | str]], key: str, percentile: float) -> float:
    samples = sorted(abs(sample) for sample in values(rows, key))
    index = min(round((len(samples) - 1) * percentile / 100.0), len(samples) - 1)
    return samples[index]


def degrees(samples: list[float]) -> list[float]:
    return [math.degrees(sample) for sample in samples]


def wrapped_degrees(samples: list[float]) -> list[float]:
    return [math.degrees(math.atan2(math.sin(sample), math.cos(sample))) for sample in samples]


def unwrap(samples: list[float]) -> list[float]:
    if not samples:
        return []
    result = [samples[0]]
    offset = 0.0
    previous = samples[0]
    for sample_value in samples[1:]:
        delta = sample_value - previous
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        result.append(sample_value + offset)
        previous = sample_value
    return result


def assert_takeoff(rows: list[dict[str, float | str]]) -> None:
    assert final(rows, "z") > 2.0, f"takeoff: final altitude too low: {final(rows, 'z'):.2f} m"
    assert final(rows, "x") > 8.0, f"takeoff: did not accelerate down runway: x={final(rows, 'x'):.2f}"
    assert final(rows, "airspeed") > 3.0, f"takeoff: final airspeed too low: {final(rows, 'airspeed'):.2f}"
    assert max_abs(rows, "roll") < 0.45, f"takeoff: wings not level, max |roll|={max_abs(rows, 'roll'):.2f}"


def assert_altitude(rows: list[dict[str, float | str]]) -> None:
    tail = rows[len(rows) // 2 :]
    mean_altitude = sum(f(row, "z") for row in tail) / len(tail)
    assert abs(mean_altitude - 3.0) < 1.5, f"altitude: mean tail altitude {mean_altitude:.2f} m"
    assert min(f(row, "airspeed") for row in tail) > 2.0, "altitude: airspeed collapsed"


def assert_heading(rows: list[dict[str, float | str]]) -> None:
    assert abs(final(rows, "yaw")) < 0.7, f"heading: final yaw error too large: {final(rows, 'yaw'):.2f} rad"
    assert final(rows, "x") > 10.0, f"heading: insufficient forward progress: x={final(rows, 'x'):.2f}"


def assert_pattern(rows: list[dict[str, float | str]]) -> None:
    laps = max(values(rows, "laps"))
    assert f(rows[0], "z") < 0.3, "pattern: mission did not start on the ground"
    assert any(f(row, "mission_phase") == 1.0 for row in rows), "pattern: takeoff phase was not observed"
    assert laps >= 2.0, f"pattern: expected two laps, got {laps:.0f}"
    assert max(values(rows, "z")) > 2.0, "pattern: never climbed into pattern altitude"
    assert final(rows, "mission_phase") == 3.0, "pattern: landing phase did not start"
    assert final(rows, "z") < 0.3, f"pattern: did not land, final z={final(rows, 'z'):.2f}"
    tracking = [row for row in rows if f(row, "mission_phase") == 2.0]
    assert rms(tracking, "cross_track_error") < 4.0, "pattern: RMS cross-track error exceeds 4 m"
    assert percentile_abs(tracking, "cross_track_error", 95.0) < 8.0, "pattern: p95 cross-track error exceeds 8 m"
    level_start = next(
        (index for index, row in enumerate(tracking) if f(row, "z") >= 2.8),
        len(tracking),
    )
    level_tracking = tracking[level_start:]
    assert level_tracking, "pattern: never entered level-flight altitude tracking"
    altitude_rms = math.sqrt(
        sum((f(row, "z") - f(row, "desired_altitude")) ** 2 for row in level_tracking)
        / len(level_tracking)
    )
    assert altitude_rms < 0.5, f"pattern: cruise RMS altitude error {altitude_rms:.2f} m"
    assert min(values(tracking, "airspeed")) > 3.0, "pattern: minimum airspeed fell below 3 m/s"
    landing_tracking = [
        row for row in rows
        if f(row, "mission_phase") == 3.0 and f(row, "airspeed") > 0.5
    ]
    assert landing_tracking, "pattern: no guided landing segment was observed"
    assert rms(landing_tracking, "cross_track_error") < 4.0, (
        "pattern: landing RMS cross-track error exceeds 4 m"
    )

    transitions = [
        rows[index - 1]
        for index in range(1, len(rows))
        if f(rows[index], "current_waypoint") != f(rows[index - 1], "current_waypoint")
    ]
    expected_transitions = 2 * len(PATTERN_WAYPOINTS)
    assert len(transitions) >= expected_transitions, (
        f"pattern: expected at least {expected_transitions} leg transitions, got {len(transitions)}"
    )
def parse_waypoint(value: str) -> tuple[float, float, float]:
    try:
        coordinates = tuple(float(component.strip()) for component in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid waypoint {value!r}; expected x,y,z") from exc
    if len(coordinates) != 3:
        raise argparse.ArgumentTypeError(f"invalid waypoint {value!r}; expected x,y,z")
    return coordinates


def waypoint_parameters(waypoints: list[tuple[float, float, float]]) -> dict[str, float]:
    params = {"outerLoop.route.waypointCount": float(len(waypoints))}
    axis_names = ("X", "Y", "Z")
    for index, waypoint in enumerate(waypoints, start=1):
        for axis_name, coordinate in zip(axis_names, waypoint, strict=True):
            params[f"outerLoop.route.waypoint{index}{axis_name}"] = coordinate
    return params


def save_plot(fig: plt.Figure, name: str) -> Path:
    path = ARTIFACT_DIR / name
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)
    print(f"wrote {path}")
    return path


def plot_topdown(rows: list[dict[str, float | str]]) -> Path:
    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    ax.plot(values(rows, "x"), values(rows, "y"), color="#1f77b4", linewidth=1.5, label="flight path")
    closed_route = [*PATTERN_WAYPOINTS, PATTERN_WAYPOINTS[0]]
    route_x = [wp[0] for wp in closed_route]
    route_y = [wp[1] for wp in closed_route]
    ax.plot(route_x, route_y, "k--", linewidth=1.0, label="waypoint route")
    ax.scatter(route_x, route_y, color="black", s=28, zorder=3)
    for idx, (x, y, _z) in enumerate(PATTERN_WAYPOINTS, start=1):
        ax.annotate(str(idx), (x, y), textcoords="offset points", xytext=(5, 5), fontsize=9)
    ax.set_title("Top-Down Pattern Track")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")
    return save_plot(fig, "pattern-topdown.png")


def plot_altitude(rows: list[dict[str, float | str]]) -> Path:
    fig, ax = plt.subplots(figsize=(9, 4), constrained_layout=True)
    t = values(rows, "time")
    ax.plot(t, values(rows, "desired_altitude"), "k--", linewidth=1.2, label="altitude command")
    ax.plot(t, values(rows, "z"), color="#1f77b4", linewidth=1.4, label="altitude")
    ax.set_title("Altitude Command Response")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("altitude [m]")
    ax.grid(True)
    ax.legend(loc="best")
    return save_plot(fig, "pattern-altitude.png")


def plot_heading(rows: list[dict[str, float | str]]) -> Path:
    fig, ax = plt.subplots(figsize=(9, 4), constrained_layout=True)
    t = values(rows, "time")
    ax.plot(t, wrapped_degrees(values(rows, "desired_heading")), "k--", linewidth=1.2, label="heading command")
    ax.plot(t, wrapped_degrees(values(rows, "heading")), color="#1f77b4", linewidth=1.4, label="heading")
    ax.set_title("Heading Command Response")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("heading [deg]")
    ax.set_ylim(-180.0, 180.0)
    ax.set_yticks([-180, -90, 0, 90, 180])
    ax.grid(True)
    ax.legend(loc="best")
    return save_plot(fig, "pattern-heading.png")


def plot_safe_commands(rows: list[dict[str, float | str]]) -> Path:
    fig, ax = plt.subplots(figsize=(9, 4), constrained_layout=True)
    t = values(rows, "time")
    ax.plot(t, values(rows, "stick_throttle"), color="#1f77b4", linewidth=1.4, label="throttle to SAFE")
    ax.set_title("Throttle Command to SAFE")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("normalized")
    ax.grid(True)
    ax.legend(loc="best")
    return save_plot(fig, "pattern-safe-commands.png")


def plot_attitude(rows: list[dict[str, float | str]]) -> Path:
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, constrained_layout=True)
    t = values(rows, "time")
    axes[0].plot(t, degrees(values(rows, "roll_command")), "k--", linewidth=1.2, label="bank command")
    axes[0].plot(t, degrees(values(rows, "roll")), color="#1f77b4", linewidth=1.4, label="bank angle")
    axes[0].set_title("Bank Angle Command Response")
    axes[0].set_ylabel("bank [deg]")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(t, degrees([-value for value in values(rows, "tecs_pitch_command")]), "k--", linewidth=1.2, label="TECS pitch command (0 deg trim)")
    axes[1].plot(t, degrees(values(rows, "pitch")), color="#1f77b4", linewidth=1.4, label="pitch angle")
    axes[1].set_title("Pitch Angle Command Response")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("pitch [deg]")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    return save_plot(fig, "pattern-attitude.png")


def plot_overview(rows: list[dict[str, float | str]]) -> Path:
    fig, axes = plt.subplots(4, 2, figsize=(14, 16), constrained_layout=True)
    ax_track, ax_alt, ax_heading, ax_speed, ax_bank, ax_pitch, ax_track_error, ax_safe = axes.flat
    t = values(rows, "time")
    closed_route = [*PATTERN_WAYPOINTS, PATTERN_WAYPOINTS[0]]
    route_x = [wp[0] for wp in closed_route]
    route_y = [wp[1] for wp in closed_route]
    ax_track.plot(values(rows, "x"), values(rows, "y"), color="#1f77b4", linewidth=1.4, label="flight")
    ax_track.plot(route_x, route_y, "k--", linewidth=1.0, label="waypoints")
    ax_track.scatter(route_x, route_y, color="black", s=22)
    for index, (x, y, _z) in enumerate(PATTERN_WAYPOINTS, start=1):
        ax_track.annotate(str(index), (x, y), textcoords="offset points", xytext=(5, 5), fontsize=9)
    ax_track.set_title("Top-Down Track")
    ax_track.set_xlabel("x [m]")
    ax_track.set_ylabel("y [m]")
    ax_track.axis("equal")
    ax_track.grid(True)
    ax_track.legend(loc="best")
    ax_alt.plot(t, values(rows, "desired_altitude"), "k--", linewidth=1.0, label="cmd")
    ax_alt.plot(t, values(rows, "z"), color="#1f77b4", linewidth=1.2, label="actual")
    ax_alt.set_title("Altitude")
    ax_alt.set_xlabel("time [s]")
    ax_alt.set_ylabel("m")
    ax_alt.grid(True)
    ax_alt.legend(loc="best")
    ax_heading.plot(t, wrapped_degrees(values(rows, "desired_heading")), "k--", linewidth=1.0, label="cmd")
    ax_heading.plot(t, wrapped_degrees(values(rows, "heading")), color="#1f77b4", linewidth=1.2, label="actual")
    ax_heading.set_title("Heading")
    ax_heading.set_xlabel("time [s]")
    ax_heading.set_ylabel("deg")
    ax_heading.set_ylim(-180.0, 180.0)
    ax_heading.set_yticks([-180, -90, 0, 90, 180])
    ax_heading.grid(True)
    ax_heading.legend(loc="best")
    ax_speed.plot(t, values(rows, "desired_speed"), "k--", linewidth=1.0, label="cmd")
    ax_speed.plot(t, values(rows, "airspeed"), color="#1f77b4", linewidth=1.2, label="actual")
    ax_speed.set_title("Airspeed")
    ax_speed.set_xlabel("time [s]")
    ax_speed.set_ylabel("m/s")
    ax_speed.grid(True)
    ax_speed.legend(loc="best")
    ax_bank.plot(t, degrees(values(rows, "roll_command")), "k--", linewidth=1.0, label="cmd")
    ax_bank.plot(t, degrees(values(rows, "roll")), color="#1f77b4", linewidth=1.2, label="actual")
    ax_bank.set_title("Bank")
    ax_bank.set_xlabel("time [s]")
    ax_bank.set_ylabel("deg")
    ax_bank.grid(True)
    ax_bank.legend(loc="best")
    ax_pitch.plot(t, degrees([-value for value in values(rows, "tecs_pitch_command")]), "k--", linewidth=1.0, label="cmd (0 deg trim)")
    ax_pitch.plot(t, degrees(values(rows, "pitch")), color="#1f77b4", linewidth=1.2, label="actual")
    ax_pitch.set_title("Pitch")
    ax_pitch.set_xlabel("time [s]")
    ax_pitch.set_ylabel("deg")
    ax_pitch.grid(True)
    ax_pitch.legend(loc="best")
    ax_track_error.plot(t, values(rows, "cross_track_error"), color="#1f77b4", linewidth=1.2, label="cross-track")
    ax_track_error.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
    ax_track_error.set_title("Cross-Track Error")
    ax_track_error.set_xlabel("time [s]")
    ax_track_error.set_ylabel("m")
    ax_track_error.grid(True)
    ax_track_error.legend(loc="best")
    ax_safe.plot(t, values(rows, "stick_throttle"), color="#1f77b4", linewidth=1.2, label="throttle")
    ax_safe.set_title("Throttle Command to SAFE")
    ax_safe.set_xlabel("time [s]")
    ax_safe.set_ylabel("normalized command")
    ax_safe.grid(True)
    ax_safe.legend(loc="best")
    return save_plot(fig, "cubs2-flight-summary.png")


def plot(stages: dict[str, list[dict[str, float | str]]]) -> list[Path]:
    rows = stages["pattern"]
    paths = [
        plot_overview(rows),
        plot_topdown(rows),
        plot_altitude(rows),
        plot_heading(rows),
        plot_safe_commands(rows),
        plot_attitude(rows),
    ]
    legacy_path = ARTIFACT_DIR / "cubs2-track-sil.png"
    legacy_path.write_bytes(paths[0].read_bytes())
    paths.append(legacy_path)
    return paths


def run_checks(stages: dict[str, list[dict[str, float | str]]]) -> list[tuple[str, str, str]]:
    checks: list[tuple[str, Callable[[list[dict[str, float | str]]], None]]] = [
        ("takeoff", assert_takeoff),
        ("altitude", assert_altitude),
        ("heading", assert_heading),
        ("pattern", assert_pattern),
    ]
    results = []
    for name, check in checks:
        try:
            check(stages[name])
            results.append((name, "PASS", ""))
        except AssertionError as exc:
            results.append((name, "FAIL", str(exc)))
    return results


def write_markdown_report(stages: dict[str, list[dict[str, float | str]]], check_results: list[tuple[str, str, str]], plot_paths: list[Path]) -> Path:
    path = ARTIFACT_DIR / "flight-summary.md"
    pattern = stages["pattern"]
    tracking = [row for row in pattern if f(row, "mission_phase") == 2.0]
    level_start = next(
        (index for index, row in enumerate(tracking) if f(row, "z") >= 2.8),
        len(tracking),
    )
    level_tracking = tracking[level_start:]
    landing_tracking = [
        row for row in pattern
        if f(row, "mission_phase") == 3.0 and f(row, "airspeed") > 0.5
    ]
    transitions = [
        pattern[index - 1]
        for index in range(1, len(pattern))
        if f(pattern[index], "current_waypoint") != f(pattern[index - 1], "current_waypoint")
    ]
    lines = [
        "# CUBS2 Flight SIL",
        "",
        "Generated by the Python SIL harness. Python uses the Rumoca Python binding to compile each Modelica scenario and execute Rumoca simulation; Python only normalizes the returned trace for checks and plots.",
        "",
        "GitHub Actions job summaries cannot reliably embed local/generated images directly. Open the uploaded `cerebri-cubs2-flight-sil` artifact to view `flight-report.html` and the PNG plots.",
        "",
        "## Checks",
        "",
        "| scenario | result | detail |",
        "| --- | --- | --- |",
    ]
    for name, status, detail in check_results:
        lines.append(f"| {name} | {status} | {detail or '-'} |")
    lines.extend([
        "",
        "## Pattern Metrics",
        "",
        "| metric | value |",
        "| --- | ---: |",
        f"| duration [s] | {final(pattern, 'time'):.3f} |",
        f"| laps | {max(values(pattern, 'laps')):.3f} |",
        f"| max altitude [m] | {max(values(pattern, 'z')):.3f} |",
        f"| final altitude [m] | {final(pattern, 'z'):.3f} |",
        f"| final heading [deg] | {math.degrees(final(pattern, 'heading')):.3f} |",
        f"| final waypoint | {final(pattern, 'current_waypoint'):.0f} |",
        f"| RMS cross-track [m] | {rms(tracking, 'cross_track_error'):.3f} |",
        f"| p95 absolute cross-track [m] | {percentile_abs(tracking, 'cross_track_error', 95.0):.3f} |",
        f"| cruise RMS altitude error [m] | {math.sqrt(sum((f(row, 'z') - f(row, 'desired_altitude')) ** 2 for row in level_tracking) / len(level_tracking)):.3f} |",
        f"| mean airspeed [m/s] | {sum(values(tracking, 'airspeed')) / len(tracking):.3f} |",
        f"| minimum airspeed [m/s] | {min(values(tracking, 'airspeed')):.3f} |",
        f"| landing RMS cross-track [m] | {rms(landing_tracking, 'cross_track_error'):.3f} |" if landing_tracking else "| landing RMS cross-track [m] | n/a |",
        f"| landing max absolute cross-track [m] | {max_abs(landing_tracking, 'cross_track_error'):.3f} |" if landing_tracking else "| landing max absolute cross-track [m] | n/a |",
        f"| max transition cross-track [m] | {max(abs(f(row, 'cross_track_error')) for row in transitions):.3f} |" if transitions else "| max transition cross-track [m] | n/a |",
        f"| max transition course error [deg] | {math.degrees(max(abs(f(row, 'course_alignment_error')) for row in transitions)):.3f} |" if transitions else "| max transition course error [deg] | n/a |",
        "",
        "## Flight Plots",
        "",
    ])
    for plot_path in plot_paths:
        lines.append(f"- `{plot_path.name}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {path}")
    return path


def image_data_uri(path: Path) -> str:
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:image/png;base64,{data}"


def write_html_report(check_results: list[tuple[str, str, str]], plot_paths: list[Path]) -> Path:
    path = ARTIFACT_DIR / "flight-report.html"
    check_rows = "\n".join(
        f"<tr><td>{html.escape(name)}</td><td>{html.escape(status)}</td><td>{html.escape(detail or '-')}</td></tr>"
        for name, status, detail in check_results
    )
    plot_sections = "\n".join(
        f"<section><h2>{html.escape(plot_path.stem.replace('-', ' ').title())}</h2>"
        f"<img src=\"{image_data_uri(plot_path)}\" alt=\"{html.escape(plot_path.stem)}\"></section>"
        for plot_path in plot_paths
        if plot_path.name != "cubs2-track-sil.png"
    )
    path.write_text(
        "\n".join([
            "<!doctype html>",
            "<html lang=\"en\">",
            "<head>",
            "<meta charset=\"utf-8\">",
            "<title>CUBS2 Flight SIL Report</title>",
            "<style>",
            "body{font-family:system-ui,sans-serif;margin:2rem;line-height:1.45;color:#111827}",
            "table{border-collapse:collapse;margin:1rem 0}td,th{border:1px solid #d1d5db;padding:.4rem .6rem}",
            "img{max-width:100%;height:auto;border:1px solid #d1d5db}",
            "section{margin:2rem 0}",
            "</style>",
            "</head>",
            "<body>",
            "<h1>CUBS2 Flight SIL Report</h1>",
            "<h2>Checks</h2>",
            "<table><thead><tr><th>Scenario</th><th>Result</th><th>Detail</th></tr></thead><tbody>",
            check_rows,
            "</tbody></table>",
            plot_sections,
            "</body>",
            "</html>",
        ]),
        encoding="utf-8",
    )
    print(f"wrote {path}")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern-t-end", type=float, default=None)
    parser.add_argument(
        "--waypoint",
        action="append",
        type=parse_waypoint,
        help="runtime pattern waypoint x,y,z; repeat 2 to 4 times (default: checked-in four-point mission)",
    )
    args = parser.parse_args()

    if args.waypoint is not None and not 2 <= len(args.waypoint) <= 4:
        parser.error("--waypoint must be repeated between 2 and 4 times")
    if args.waypoint is not None:
        PATTERN_WAYPOINTS[:] = args.waypoint
    pattern_params = waypoint_parameters(PATTERN_WAYPOINTS)

    stages = {
        "takeoff": run_rumoca_stage("takeoff"),
        "altitude": run_rumoca_stage("altitude"),
        "heading": run_rumoca_stage("heading"),
        "pattern": run_rumoca_stage("pattern", t_end=args.pattern_t_end, params=pattern_params),
    }
    plot_paths = plot(stages)
    check_results = run_checks(stages)
    write_markdown_report(stages, check_results, plot_paths)
    write_html_report(check_results, plot_paths)

    failures = [detail for _name, status, detail in check_results if status != "PASS"]
    if failures:
        raise AssertionError("; ".join(failures))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"flight SIL assertion failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
