#!/usr/bin/env python3
"""Run the CUBS2 staged flight SIL test and generate CI plots."""

from __future__ import annotations

import argparse
import base64
import csv
import ctypes
from dataclasses import dataclass, field
import html
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tomllib
from typing import Callable

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_DIR = ROOT / "artifacts" / "flight"
GENERATED_DIR = ARTIFACT_DIR / "generated"
CONTROLLER_LIB = ARTIFACT_DIR / "libfixed_wing_outer_loop.so"
MODEL_FILE = ROOT / "src" / "FixedWingOuterLoop.mo"
SCENARIO_DIR = ROOT / "tests" / "flight"
PATTERN_WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (6.0, 0.0, 3.0),
    (18.0, 0.0, 3.0),
    (18.0, 12.0, 3.0),
    (0.0, 12.0, 3.0),
    (0.0, 0.0, 3.0),
    (6.0, 0.0, 3.0),
]


@dataclass(frozen=True)
class ScenarioConfig:
    dt: float
    t_end: float
    output: Path


@dataclass
class PlantState:
    p: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.10], dtype=float))
    v_b: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=float))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))


@dataclass
class PlantDerivative:
    p: np.ndarray
    v_b: np.ndarray
    q_dot: np.ndarray
    omega: np.ndarray


@dataclass
class SurfaceCommand:
    ail: float = 0.0
    elev: float = 0.0
    rud: float = 0.0
    thr: float = 0.0


@dataclass
class StickCommand:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0


@dataclass
class InnerLoopState:
    i_p: float = 0.0
    i_q: float = 0.0
    i_r: float = 0.0
    phi_sp: float = 0.0
    theta_sp: float = 0.0


@dataclass
class PlantOutput:
    position: np.ndarray
    velocity_w: np.ndarray
    euler: np.ndarray
    up_body: np.ndarray
    airspeed: float


@dataclass
class ControllerApi:
    state_type: type[ctypes.Structure]
    lib: ctypes.CDLL

    def new_state(self, mode: str) -> ctypes.Structure:
        state = self.state_type()
        self.lib.FixedWingOuterLoop_startup(ctypes.byref(state))
        configure_controller(state, mode)
        self.lib.FixedWingOuterLoop_recalibrate(ctypes.byref(state))
        return state

    def step(self, state: ctypes.Structure, y: PlantOutput) -> None:
        for i in range(3):
            state.position_m[i] = float(y.position[i])
            state.euler_rad[i] = float(y.euler[i])
        self.lib.FixedWingOuterLoop_dostep(ctypes.byref(state))


def run(cmd: list[str], *, cwd: Path = ROOT) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def codegen_controller() -> None:
    rumoca = os.environ.get("CUBS2_RUMOCA_EXECUTABLE", "rumoca")
    shutil.rmtree(GENERATED_DIR, ignore_errors=True)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    run([
        rumoca,
        "compile",
        str(MODEL_FILE),
        "--model",
        "FixedWingOuterLoop",
        "--target",
        "embedded-c-galec",
        "--output",
        str(GENERATED_DIR),
    ])


def build_controller_library() -> ControllerApi:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    codegen_controller()

    run([
        os.environ.get("CC", "cc"),
        "-std=c11",
        "-O2",
        "-fPIC",
        "-shared",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(GENERATED_DIR),
        str(GENERATED_DIR / "FixedWingOuterLoop.c"),
        "-lm",
        "-o",
        str(CONTROLLER_LIB),
    ])

    state_type = parse_controller_state(GENERATED_DIR / "FixedWingOuterLoop.h")
    lib = ctypes.CDLL(str(CONTROLLER_LIB))
    pointer = ctypes.POINTER(state_type)
    lib.FixedWingOuterLoop_startup.argtypes = [pointer]
    lib.FixedWingOuterLoop_startup.restype = None
    lib.FixedWingOuterLoop_recalibrate.argtypes = [pointer]
    lib.FixedWingOuterLoop_recalibrate.restype = None
    lib.FixedWingOuterLoop_dostep.argtypes = [pointer]
    lib.FixedWingOuterLoop_dostep.restype = None
    return ControllerApi(state_type=state_type, lib=lib)


def parse_controller_state(header: Path) -> type[ctypes.Structure]:
    text = header.read_text(encoding="utf-8")
    match = re.search(r"typedef struct \{(?P<body>.*?)\} FixedWingOuterLoopState;", text, re.S)
    if match is None:
        raise RuntimeError(f"could not find FixedWingOuterLoopState in {header}")

    fields = []
    for line in match.group("body").splitlines():
        decl = re.match(r"\s*(double|int32_t|bool)\s+([A-Za-z_][A-Za-z0-9_]*)([0-9\]\[]*)\s*;", line)
        if decl is None:
            continue
        c_type = {
            "double": ctypes.c_double,
            "int32_t": ctypes.c_int32,
            "bool": ctypes.c_bool,
        }[decl.group(1)]
        dims = [int(value) for value in re.findall(r"\[(\d+)\]", decl.group(3))]
        for dim in reversed(dims):
            c_type = c_type * dim
        fields.append((decl.group(2), c_type))

    class FixedWingOuterLoopState(ctypes.Structure):
        _fields_ = fields

    return FixedWingOuterLoopState


def set_route(state: ctypes.Structure, waypoints: list[tuple[float, float, float]], cruise_speed: float) -> None:
    state.route_nSegments = 6
    state.guidance_route_nSegments = 6
    state.route_cruiseSpeed = cruise_speed
    state.guidance_route_cruiseSpeed = cruise_speed
    state.route_altitudeToFlightPathGain = 2.0
    state.guidance_route_altitudeToFlightPathGain = 2.0
    state.route_altitudeLookaheadDistance = 8.0
    state.guidance_route_altitudeLookaheadDistance = 8.0
    state.route_flightPathAngleLimit = 0.12
    state.guidance_route_flightPathAngleLimit = 0.12
    state.route_speedToAccelerationGain = 1.0
    state.guidance_route_speedToAccelerationGain = 1.0
    state.route_crossTrackSteeringDistance = 2.0
    state.guidance_route_crossTrackSteeringDistance = 2.0
    state.route_waypointSwitchingDistance = 3.0
    state.guidance_route_waypointSwitchingDistance = 3.0
    for i, waypoint in enumerate(waypoints):
        for j, value in enumerate(waypoint):
            state.route_waypoints[i][j] = value
            state.guidance_route_waypoints[i][j] = value


def configure_controller(state: ctypes.Structure, mode: str) -> None:
    for prefix in ("vehicle", "tecs_vehicle", "attitude_vehicle"):
        setattr(state, f"{prefix}_mass", 0.065)
        setattr(state, f"{prefix}_thrustMax", 0.30)
        setattr(state, f"{prefix}_trimThrust", 0.10)
        setattr(state, f"{prefix}_envelopeDrag", 0.07)

    straight = [
        (0.0, 0.0, 3.0),
        (30.0, 0.0, 3.0),
        (60.0, 0.0, 3.0),
        (90.0, 0.0, 3.0),
        (120.0, 0.0, 3.0),
        (150.0, 0.0, 3.0),
        (180.0, 0.0, 3.0),
    ]
    set_route(state, PATTERN_WAYPOINTS if mode == "pattern" else straight, 4.0)


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    a, b, c, d = q
    return np.array([
        [1.0 - 2.0 * (c * c + d * d), 2.0 * (b * c - a * d), 2.0 * (b * d + a * c)],
        [2.0 * (b * c + a * d), 1.0 - 2.0 * (b * b + d * d), 2.0 * (c * d - a * b)],
        [2.0 * (b * d - a * c), 2.0 * (c * d + a * b), 1.0 - 2.0 * (b * b + c * c)],
    ])


def quat_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    a, b, c, d = q
    err = float(q @ q - 1.0)
    return np.array([
        0.5 * (-b * omega[0] - c * omega[1] - d * omega[2]) - err * a,
        0.5 * (a * omega[0] - d * omega[1] + c * omega[2]) - err * b,
        0.5 * (d * omega[0] + a * omega[1] - b * omega[2]) - err * c,
        0.5 * (-c * omega[0] + b * omega[1] + a * omega[2]) - err * d,
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else q


def euler_from_quat(q: np.ndarray) -> np.ndarray:
    a, b, c, d = q
    sinp = clamp(2.0 * (a * c - d * b), -1.0, 1.0)
    return np.array([
        math.atan2(2.0 * (a * b + c * d), 1.0 - 2.0 * (b * b + c * c)),
        math.asin(sinp),
        math.atan2(2.0 * (a * d + b * c), 1.0 - 2.0 * (c * c + d * d)),
    ])


def plant_outputs(x: PlantState) -> PlantOutput:
    rotation = quat_to_dcm(x.q)
    velocity_w = rotation @ x.v_b
    return PlantOutput(
        position=x.p.copy(),
        velocity_w=velocity_w,
        euler=euler_from_quat(x.q),
        up_body=rotation[2, :].copy(),
        airspeed=float(np.linalg.norm(x.v_b) + 1e-6),
    )


def sportcub_forces(x: PlantState, u: SurfaceCommand) -> tuple[np.ndarray, np.ndarray]:
    mass = 0.065
    rho = 1.225
    wing_area = 0.055
    span = 0.617
    cbar = 0.09
    wing_incidence = math.radians(6.0)
    thrust_max = 0.30
    cl0 = 0.5
    cla = 4.7
    cd0 = 0.06
    k_ind = 0.09
    cd0_fp = 0.30
    cy_fp_coef = 0.50
    cm0 = 0.0
    cma = -0.8
    cmq = -12.0
    cmde = 0.3
    cyb = -0.50
    cyda = 0.004
    cydr = -0.015
    cyp = -0.15
    cyr = 0.20
    clb = -0.25
    clp = -0.50
    clr = 0.15
    clda = 0.05
    cldr = 0.006
    cnb = 0.06
    cnp = 0.010
    cnr = -0.15
    cndr = 0.015
    cnda = 0.006
    alpha_stall = math.radians(20.0)
    blend_width = math.radians(5.0)
    max_defl_ail = math.radians(30.0)
    max_defl_elev = math.radians(24.0)
    max_defl_rud = math.radians(20.0)
    ground_wn = 350.0
    ground_zeta = 0.6
    ground_c_xy = 0.05
    ground_mu = 0.15
    ground_max_force_per_wheel = 20.0
    tailwheel_steer_gain = 0.03
    wheels = [
        np.array([0.1, 0.1, -0.1]),
        np.array([0.1, -0.1, -0.1]),
        np.array([-0.4, 0.0, 0.0]),
    ]

    body_u = x.v_b[0]
    body_v_frd = -x.v_b[1]
    body_w_frd = -x.v_b[2]
    v_total = math.sqrt(body_u * body_u + body_v_frd * body_v_frd + body_w_frd * body_w_frd) + 1e-6
    v_xz = math.sqrt(body_u * body_u + body_w_frd * body_w_frd) + 1e-6
    alpha = math.atan2(body_w_frd, body_u) + wing_incidence
    beta = math.atan2(body_v_frd, v_xz)
    qbar = 0.5 * rho * v_total * v_total
    p_frd = x.omega[0]
    q_frd = -x.omega[1]
    r_frd = -x.omega[2]

    wind_x = body_u / v_total
    wind_y = body_v_frd / v_total
    wind_z = body_w_frd / v_total
    ref_x = 0.0 if abs(wind_z) < abs(wind_x) else 1.0
    ref_z = 1.0 if abs(wind_z) < abs(wind_x) else 0.0
    ref_dot = ref_x * wind_x + ref_z * wind_z
    wind_zt = np.array([ref_x - ref_dot * wind_x, -ref_dot * wind_y, ref_z - ref_dot * wind_z])
    wind_z_axis = wind_zt / (np.linalg.norm(wind_zt) + 1e-6)
    wind_x_axis = np.array([wind_x, wind_y, wind_z])
    wind_y_axis = np.cross(wind_z_axis, wind_x_axis)

    ail_rad = clamp(max_defl_ail * u.ail, -max_defl_ail, max_defl_ail)
    elev_rad = clamp(max_defl_elev * u.elev, -max_defl_elev, max_defl_elev)
    rud_rad = clamp(-max_defl_rud * u.rud, -max_defl_rud, max_defl_rud)
    thr_out = clamp(u.thr, 0.0, 1.0)

    sigma = (1.0 + math.tanh((alpha - alpha_stall) / blend_width)) / 2.0
    cl_lin = cl0 + cla * alpha
    cl_fp = 2.0 * math.sin(alpha) * math.cos(alpha)
    lift_coef = (1.0 - sigma) * cl_lin + sigma * cl_fp
    cd_lin = cd0 + k_ind * cl_lin * cl_lin
    cd_fp = cd0_fp + 2.0 * math.sin(alpha) * math.sin(alpha)
    drag_coef = (1.0 - sigma) * cd_lin + sigma * cd_fp
    cy_lin = (
        cyb * beta
        + cyda * ail_rad
        + cydr * rud_rad
        + cyp * (span / (2.0 * v_total)) * p_frd
        + cyr * (span / (2.0 * v_total)) * r_frd
    )
    side_coef = (1.0 - sigma) * cy_lin + sigma * cy_fp_coef * math.sin(beta) * math.cos(alpha)
    roll_coef = (
        clda * ail_rad
        + cldr * rud_rad
        + clb * beta
        + clp * (span / (2.0 * v_total)) * p_frd
        + clr * (span / (2.0 * v_total)) * r_frd
    )
    pitch_coef = cm0 + cma * alpha + cmde * elev_rad + cmq * (cbar / (2.0 * v_total)) * q_frd
    yaw_coef = (
        cnb * beta
        + cndr * rud_rad
        + cnda * ail_rad
        + cnp * (span / (2.0 * v_total)) * p_frd
        + cnr * (span / (2.0 * v_total)) * r_frd
    )

    aero_frd = qbar * wing_area * (
        wind_x_axis * (-drag_coef) + wind_y_axis * side_coef + wind_z_axis * (-lift_coef)
    )
    moment_frd = qbar * wing_area * np.array([span * roll_coef, cbar * pitch_coef, span * yaw_coef])
    force_aero = np.array([aero_frd[0], -aero_frd[1], -aero_frd[2]])
    moment_aero = np.array([moment_frd[0], -moment_frd[1], -moment_frd[2]])
    force_thrust = np.array([thrust_max * thr_out, 0.0, 0.0])

    rotation = quat_to_dcm(x.q)
    ground_k = mass * ground_wn * ground_wn
    ground_c_vert = 2.0 * ground_zeta * mass * ground_wn
    force_ground = np.zeros(3)
    moment_ground = np.zeros(3)
    tailwheel_vx = 0.0
    tailwheel_height = 1.0

    for index, wheel in enumerate(wheels):
        wheel_w = rotation @ wheel
        wheel_height = x.p[2] + wheel_w[2]
        wheel_v_b = x.v_b + np.cross(x.omega, wheel)
        wheel_v_w = rotation @ wheel_v_b
        if index == 2:
            tailwheel_vx = wheel_v_w[0]
            tailwheel_height = wheel_height

        normal_unclamped = -wheel_height * ground_k - wheel_v_w[2] * ground_c_vert
        normal = clamp(normal_unclamped, 0.0, ground_max_force_per_wheel)
        lateral_xy = np.array([-wheel_v_w[0] * ground_c_xy, -wheel_v_w[1] * ground_c_xy])
        lateral_mag = float(np.linalg.norm(lateral_xy) + 1e-9)
        lateral_scale = min(1.0, ground_mu * normal / lateral_mag)
        force_w = np.array([lateral_scale * lateral_xy[0], lateral_scale * lateral_xy[1], normal])
        if wheel_height >= 0.0:
            force_w[:] = 0.0

        force_b = rotation.T @ force_w
        force_ground += force_b
        moment_ground += np.cross(wheel, force_b)

    if tailwheel_height < 0.0:
        rud_tail = max_defl_rud * clamp(u.rud, -1.0, 1.0)
        moment_ground[2] += tailwheel_steer_gain * rud_tail * (math.sqrt(tailwheel_vx * tailwheel_vx + 1.0) - 1.0)

    return force_aero + force_ground + force_thrust, moment_aero + moment_ground


def plant_derivative(x: PlantState, u: SurfaceCommand) -> PlantDerivative:
    mass = 0.065
    jx = 8.0e-4
    jy = 1.2e-3
    jz = 1.8e-3
    jxz = 1.0e-4
    rotation = quat_to_dcm(x.q)
    force_b, moment_b = sportcub_forces(x, u)
    gravity_b = rotation.T @ np.array([0.0, 0.0, -9.81])
    velocity_dot = force_b / mass + gravity_b - np.cross(x.omega, x.v_b)
    angular_momentum = np.array([
        jx * x.omega[0] + jxz * x.omega[2],
        jy * x.omega[1],
        jxz * x.omega[0] + jz * x.omega[2],
    ])
    body_moment = moment_b - np.cross(x.omega, angular_momentum)
    denominator = jx * jz - jxz * jxz
    omega_dot = np.array([
        (jz * body_moment[0] - jxz * body_moment[2]) / denominator,
        body_moment[1] / jy,
        (-jxz * body_moment[0] + jx * body_moment[2]) / denominator,
    ])
    return PlantDerivative(
        p=rotation @ x.v_b,
        v_b=velocity_dot,
        q_dot=quat_derivative(x.q, x.omega),
        omega=omega_dot,
    )


def add_scaled(x: PlantState, dx: PlantDerivative, scale: float) -> PlantState:
    return PlantState(
        p=x.p + scale * dx.p,
        v_b=x.v_b + scale * dx.v_b,
        q=quat_normalize(x.q + scale * dx.q_dot),
        omega=x.omega + scale * dx.omega,
    )


def plant_rk4_step(x: PlantState, u: SurfaceCommand, dt: float) -> PlantState:
    k1 = plant_derivative(x, u)
    k2 = plant_derivative(add_scaled(x, k1, 0.5 * dt), u)
    k3 = plant_derivative(add_scaled(x, k2, 0.5 * dt), u)
    k4 = plant_derivative(add_scaled(x, k3, dt), u)
    return PlantState(
        p=x.p + dt * (k1.p + 2.0 * k2.p + 2.0 * k3.p + k4.p) / 6.0,
        v_b=x.v_b + dt * (k1.v_b + 2.0 * k2.v_b + 2.0 * k3.v_b + k4.v_b) / 6.0,
        q=quat_normalize(x.q + dt * (k1.q_dot + 2.0 * k2.q_dot + 2.0 * k3.q_dot + k4.q_dot) / 6.0),
        omega=x.omega + dt * (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega) / 6.0,
    )


def inner_loop_step(state: InnerLoopState, stick: StickCommand, x: PlantState, y: PlantOutput, dt: float, armed: bool) -> SurfaceCommand:
    armed_value = 1.0 if armed else 0.0
    phi = math.atan2(y.up_body[1], y.up_body[2])
    theta = math.atan2(y.up_body[0], y.up_body[2])
    climb_auth = clamp((y.airspeed - 2.6) / (3.6 - 2.6), 0.0, 1.0)
    rate_phi = armed_value * stick.roll * 1.5
    pitch_rate = stick.pitch * 0.9 * (climb_auth if stick.pitch > 0.0 else 1.0)
    rate_theta = armed_value * pitch_rate

    phi_dot = min(0.0, rate_phi) if state.phi_sp > 0.90 else max(0.0, rate_phi) if state.phi_sp < -0.90 else rate_phi
    theta_dot = min(0.0, rate_theta) if state.theta_sp > 0.45 else max(0.0, rate_theta) if state.theta_sp < -0.45 else rate_theta
    state.phi_sp += phi_dot * dt
    state.theta_sp += theta_dot * dt

    theta_eff = min(state.theta_sp, -0.06 * (2.6 - y.airspeed)) if y.airspeed < 2.6 else state.theta_sp
    p_sp = clamp(5.0 * (state.phi_sp - phi), -4.0, 4.0)
    q_up_sp = clamp(5.0 * (theta_eff - theta), -2.5, 2.5)
    r_sp = stick.yaw
    e_p = p_sp - x.omega[0]
    e_q = q_up_sp - (-x.omega[1])
    e_r = r_sp - x.omega[2]

    state.i_p += (min(0.0, armed_value * e_p) if state.i_p > 1.0 else max(0.0, armed_value * e_p) if state.i_p < -1.0 else armed_value * e_p) * dt
    state.i_q += (min(0.0, armed_value * e_q) if state.i_q > 1.0 else max(0.0, armed_value * e_q) if state.i_q < -1.0 else armed_value * e_q) * dt
    state.i_r += (min(0.0, armed_value * e_r) if state.i_r > 0.6 else max(0.0, armed_value * e_r) if state.i_r < -0.6 else armed_value * e_r) * dt

    return SurfaceCommand(
        ail=0.45 * e_p + 0.30 * clamp(state.i_p, -1.0, 1.0),
        elev=0.55 * e_q + 0.40 * clamp(state.i_q, -1.0, 1.0),
        rud=0.40 * e_r + 0.10 * clamp(state.i_r, -0.6, 0.6),
        thr=armed_value * stick.throttle,
    )


def stage_end_time(mode: str) -> float:
    return {"takeoff": 8.0, "altitude": 14.0, "heading": 18.0}.get(mode, 150.0)


def load_scenario_config(mode: str) -> ScenarioConfig:
    path = SCENARIO_DIR / f"rumoca-scenario.{mode}.toml"
    if not path.exists():
        return ScenarioConfig(
            dt=0.02,
            t_end=stage_end_time(mode),
            output=ARTIFACT_DIR / f"{mode}.csv",
        )
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    sim = data.get("sim", {})
    output = Path(sim.get("output", f"artifacts/flight/{mode}.csv"))
    if not output.is_absolute():
        output = ROOT / output
    return ScenarioConfig(
        dt=float(sim.get("dt", 0.02)),
        t_end=float(sim.get("t_end", stage_end_time(mode))),
        output=output,
    )


def initial_plant(mode: str) -> PlantState:
    plant = PlantState()
    if mode in {"altitude", "heading"}:
        plant.p[2] = 3.0
        plant.v_b[0] = 4.0
    if mode == "heading":
        yaw0 = -0.5
        plant.q[0] = math.cos(yaw0 / 2.0)
        plant.q[3] = math.sin(yaw0 / 2.0)
    return plant


def csv_fields() -> list[str]:
    return [
        "time", "mode", "x", "y", "z", "roll", "pitch", "yaw", "airspeed",
        "stick_roll", "stick_pitch", "stick_yaw", "stick_throttle",
        "surface_ail", "surface_elev", "surface_rud", "surface_thr",
        "current_waypoint", "laps", "desired_heading", "desired_altitude",
        "desired_flight_path_angle", "desired_acceleration", "heading",
        "course_error", "roll_command", "inner_roll_command", "pitch_command",
        "tecs_pitch_command", "tecs_thrust_command", "mission_phase",
    ]


def simulate_stage(api: ControllerApi, mode: str, t_end: float | None = None) -> list[dict[str, float | str]]:
    scenario = load_scenario_config(mode)
    controller = api.new_state(mode)
    plant = initial_plant(mode)
    inner = InnerLoopState()
    stick = StickCommand()
    surface = SurfaceCommand()
    laps = 0
    previous_waypoint = 1
    landing = False
    armed = True
    plant_dt = 0.005
    inner_dt = 0.01
    outer_dt = scenario.dt
    log_dt = scenario.dt
    next_inner = 0.0
    next_outer = 0.0
    next_log = 0.0
    rows: list[dict[str, float | str]] = []

    t_final = t_end if t_end is not None else scenario.t_end
    for t in np.arange(0.0, t_final + 0.5 * plant_dt, plant_dt):
        y = plant_outputs(plant)
        if t + 1e-9 >= next_outer:
            api.step(controller, y)
            if controller.currentWaypoint < previous_waypoint:
                laps += 1
            previous_waypoint = controller.currentWaypoint
            landing = mode == "pattern" and laps >= 2
            next_outer += outer_dt

        if mode == "takeoff":
            stick = StickCommand(roll=0.0, pitch=0.0 if t < 0.8 else 0.55, yaw=0.0, throttle=1.0)
        elif landing:
            stick = StickCommand(roll=0.0, pitch=-0.25, yaw=0.0, throttle=0.0 if plant.p[2] < 0.35 else 0.12)
        else:
            stick = StickCommand(
                roll=controller.aileron,
                pitch=controller.elevator,
                yaw=controller.rudder,
                throttle=controller.throttle,
            )
        armed = not (landing and plant.p[2] < 0.25)

        if t + 1e-9 >= next_inner:
            surface = inner_loop_step(inner, stick, plant, y, inner_dt, armed)
            next_inner += inner_dt

        if t + 1e-9 >= next_log:
            mission_phase = 3.0 if landing else 2.0 if controller.airborne else 1.0
            rows.append({
                "time": float(t),
                "mode": mode,
                "x": float(y.position[0]),
                "y": float(y.position[1]),
                "z": float(y.position[2]),
                "roll": float(y.euler[0]),
                "pitch": float(y.euler[1]),
                "yaw": float(y.euler[2]),
                "airspeed": y.airspeed,
                "stick_roll": stick.roll,
                "stick_pitch": stick.pitch,
                "stick_yaw": stick.yaw,
                "stick_throttle": stick.throttle,
                "surface_ail": surface.ail,
                "surface_elev": surface.elev,
                "surface_rud": surface.rud,
                "surface_thr": surface.thr,
                "current_waypoint": float(controller.currentWaypoint),
                "laps": float(laps),
                "desired_heading": controller.desiredHeading,
                "desired_altitude": controller.guidance_pathAltitude,
                "desired_flight_path_angle": controller.desiredFlightPathAngle,
                "desired_acceleration": controller.desiredAcceleration,
                "heading": math.atan2(y.velocity_w[1], y.velocity_w[0]),
                "course_error": controller.courseError,
                "roll_command": controller.rollCommand,
                "inner_roll_command": inner.phi_sp,
                "pitch_command": inner.theta_sp,
                "tecs_pitch_command": controller.tecs_pitchCommand,
                "tecs_thrust_command": controller.tecs_thrustCommand,
                "mission_phase": mission_phase,
            })
            next_log += log_dt

        plant = plant_rk4_step(plant, surface, plant_dt)

    write_csv(scenario.output, rows)
    return rows


def write_csv(path: Path, rows: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=csv_fields())
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {path}")


def f(row: dict[str, float | str], key: str) -> float:
    return float(row[key])


def max_abs(rows: list[dict[str, float | str]], key: str) -> float:
    return max(abs(f(row, key)) for row in rows)


def final(rows: list[dict[str, float | str]], key: str) -> float:
    return f(rows[-1], key)


def values(rows: list[dict[str, float | str]], key: str) -> list[float]:
    return [f(row, key) for row in rows]


def degrees(samples: list[float]) -> list[float]:
    return [math.degrees(sample) for sample in samples]


def unwrap(samples: list[float]) -> list[float]:
    if not samples:
        return []
    result = [samples[0]]
    offset = 0.0
    previous = samples[0]
    for sample in samples[1:]:
        delta = sample - previous
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        result.append(sample + offset)
        previous = sample
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
    assert laps >= 2.0, f"pattern: expected two laps, got {laps:.0f}"
    assert max(values(rows, "z")) > 2.0, "pattern: never climbed into pattern altitude"
    assert final(rows, "mission_phase") == 3.0, "pattern: landing phase did not start"
    assert final(rows, "z") < 1.5, f"pattern: did not descend for landing, final z={final(rows, 'z'):.2f}"


def save_plot(fig: plt.Figure, name: str) -> Path:
    path = ARTIFACT_DIR / name
    fig.savefig(path, dpi=170)
    plt.close(fig)
    print(f"wrote {path}")
    return path


def plot_topdown(rows: list[dict[str, float | str]]) -> Path:
    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    ax.plot(values(rows, "x"), values(rows, "y"), color="#1f77b4", linewidth=1.5, label="flight path")
    route_x = [wp[0] for wp in PATTERN_WAYPOINTS]
    route_y = [wp[1] for wp in PATTERN_WAYPOINTS]
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
    ax.plot(t, degrees(unwrap(values(rows, "desired_heading"))), "k--", linewidth=1.2, label="heading command")
    ax.plot(t, degrees(unwrap(values(rows, "heading"))), color="#1f77b4", linewidth=1.4, label="heading")
    ax.set_title("Heading Command Response")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("heading [deg]")
    ax.grid(True)
    ax.legend(loc="best")
    return save_plot(fig, "pattern-heading.png")


def plot_actuators(rows: list[dict[str, float | str]]) -> Path:
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, constrained_layout=True)
    t = values(rows, "time")
    axes[0].plot(t, values(rows, "stick_throttle"), "k--", linewidth=1.2, label="throttle command")
    axes[0].plot(t, values(rows, "surface_thr"), color="#1f77b4", linewidth=1.4, label="throttle response")
    axes[0].set_title("Throttle Response")
    axes[0].set_ylabel("normalized")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(t, values(rows, "stick_pitch"), "k--", linewidth=1.2, label="elevator command")
    axes[1].plot(t, values(rows, "surface_elev"), color="#1f77b4", linewidth=1.4, label="elevator response")
    axes[1].set_title("Elevator Response")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("normalized")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    return save_plot(fig, "pattern-actuators.png")


def plot_attitude(rows: list[dict[str, float | str]]) -> Path:
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, constrained_layout=True)
    t = values(rows, "time")
    axes[0].plot(t, degrees(values(rows, "roll_command")), "k--", linewidth=1.2, label="bank command")
    axes[0].plot(t, degrees(values(rows, "inner_roll_command")), color="#ff7f0e", linewidth=1.0, label="inner bank setpoint")
    axes[0].plot(t, degrees(values(rows, "roll")), color="#1f77b4", linewidth=1.4, label="bank angle")
    axes[0].set_title("Bank Angle Command Response")
    axes[0].set_ylabel("bank [deg]")
    axes[0].grid(True)
    axes[0].legend(loc="best")
    axes[1].plot(t, degrees(values(rows, "pitch_command")), "k--", linewidth=1.2, label="pitch command")
    axes[1].plot(t, degrees(values(rows, "tecs_pitch_command")), color="#ff7f0e", linewidth=1.0, label="TECS pitch command")
    axes[1].plot(t, degrees(values(rows, "pitch")), color="#1f77b4", linewidth=1.4, label="pitch angle")
    axes[1].set_title("Pitch Angle Command Response")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("pitch [deg]")
    axes[1].grid(True)
    axes[1].legend(loc="best")
    return save_plot(fig, "pattern-attitude.png")


def plot_overview(rows: list[dict[str, float | str]]) -> Path:
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
    ax_track, ax_alt, ax_heading, ax_bank, ax_pitch, ax_act = axes.flat
    t = values(rows, "time")
    route_x = [wp[0] for wp in PATTERN_WAYPOINTS]
    route_y = [wp[1] for wp in PATTERN_WAYPOINTS]
    ax_track.plot(values(rows, "x"), values(rows, "y"), color="#1f77b4", linewidth=1.4, label="flight")
    ax_track.plot(route_x, route_y, "k--", linewidth=1.0, label="waypoints")
    ax_track.scatter(route_x, route_y, color="black", s=22)
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
    ax_heading.plot(t, degrees(unwrap(values(rows, "desired_heading"))), "k--", linewidth=1.0, label="cmd")
    ax_heading.plot(t, degrees(unwrap(values(rows, "heading"))), color="#1f77b4", linewidth=1.2, label="actual")
    ax_heading.set_title("Heading")
    ax_heading.set_xlabel("time [s]")
    ax_heading.set_ylabel("deg")
    ax_heading.grid(True)
    ax_heading.legend(loc="best")
    ax_bank.plot(t, degrees(values(rows, "roll_command")), "k--", linewidth=1.0, label="cmd")
    ax_bank.plot(t, degrees(values(rows, "roll")), color="#1f77b4", linewidth=1.2, label="actual")
    ax_bank.set_title("Bank")
    ax_bank.set_xlabel("time [s]")
    ax_bank.set_ylabel("deg")
    ax_bank.grid(True)
    ax_bank.legend(loc="best")
    ax_pitch.plot(t, degrees(values(rows, "pitch_command")), "k--", linewidth=1.0, label="cmd")
    ax_pitch.plot(t, degrees(values(rows, "pitch")), color="#1f77b4", linewidth=1.2, label="actual")
    ax_pitch.set_title("Pitch")
    ax_pitch.set_xlabel("time [s]")
    ax_pitch.set_ylabel("deg")
    ax_pitch.grid(True)
    ax_pitch.legend(loc="best")
    ax_act.plot(t, values(rows, "stick_throttle"), label="throttle cmd")
    ax_act.plot(t, values(rows, "surface_thr"), label="throttle response")
    ax_act.plot(t, values(rows, "stick_pitch"), label="elevator cmd")
    ax_act.plot(t, values(rows, "surface_elev"), label="elevator response")
    ax_act.set_title("Throttle / Elevator")
    ax_act.set_xlabel("time [s]")
    ax_act.set_ylabel("normalized")
    ax_act.grid(True)
    ax_act.legend(loc="best")
    return save_plot(fig, "cubs2-flight-summary.png")


def plot(stages: dict[str, list[dict[str, float | str]]]) -> list[Path]:
    rows = stages["pattern"]
    paths = [
        plot_overview(rows),
        plot_topdown(rows),
        plot_altitude(rows),
        plot_heading(rows),
        plot_actuators(rows),
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
    lines = [
        "# CUBS2 Flight SIL",
        "",
        "Generated by the Python SIL harness. The generated Rumoca controller C is loaded through `ctypes`; the plant, inner loop, checks, and plots are Python.",
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
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--pattern-t-end", type=float, default=None)
    args = parser.parse_args()

    api = build_controller_library() if not args.skip_build else ControllerApi(
        state_type=parse_controller_state(GENERATED_DIR / "FixedWingOuterLoop.h"),
        lib=ctypes.CDLL(str(CONTROLLER_LIB)),
    )
    stages = {
        "takeoff": simulate_stage(api, "takeoff"),
        "altitude": simulate_stage(api, "altitude"),
        "heading": simulate_stage(api, "heading"),
        "pattern": simulate_stage(api, "pattern", t_end=args.pattern_t_end),
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
