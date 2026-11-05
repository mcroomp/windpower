"""Tensioned Lua takeoff to at least 50 m payout and a steady-flight orientation.

The rotor starts pre-spun, not motor-driven. The elastic tether is preloaded
and governed throughout launch; there is no slack/free-spool phase.
"""
from types import SimpleNamespace
import math

import numpy as np
import pytest
from dynbem import RotorInputs, create_aero
from scipy.optimize import brentq

from groundstation.rawes_modes import MODE_STEADY, MODE_TAKEOFF, send_anchor_ned
from groundstation.winch_protocol import WinchCommand
from linkhub_client.messages import NamedValueFloat
from simulation.param_defaults import thrust_to_coll_rad
from simulation.controller import compute_bz_altitude_hold
from simulation.frames import build_orb_frame
from simulation.rawes_lua_harness import RawesLua
from simulation.tether import TetherProperties
from simulation.winch import TorqueWinchController
from simulation.winch_node import Anemometer, GovernedWinchNode
from tests.common.mock_ardupilot import MockArdupilot
from tests.simtests._rotor_helpers import load_default_rotor
from tests.simtests.simtest_runner import PhysicsRunner

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(900)]

_ROTOR = load_default_rotor()
DT = 1.0 / 400.0
LUA_EVERY = round(0.020 / DT)
PLANNER_EVERY = round(0.1 / DT)
WIND_NED = np.array([0.0, 10.0, 0.0])
OMEGA_START_RAD_S = 40.0
LAUNCH_TENSION_N = 100.0
LIFTOFF_ALT_M = 2.0
TAKEOFF_ALT_TARGET_M = 3.0
TAKEOFF_THRUST_SEED = 0.8
TAKEOFF_LEAN_RAD = math.radians(50.0)
HANDOFF_MIN_LENGTH_M = 8.0
HANDOFF_AXIS_TOL_DEG = 8.0
TETHER_ELEVATION_RAD = math.radians(25.0)
LIFTOFF_TIMEOUT = 20.0
TRANSITION_TIMEOUT = 60.0
TOTAL_RUNTIME = 150.0
MIN_TETHER_LENGTH_M = 50.0
WINCH_MAX_LENGTH_M = 100.0
WINCH_CRUISE_OUT = 0.8
STEADY_TIMEOUT_S = 45.0
STEADY_MIN_TILT_DEG = 30.0
STEADY_AXIS_TOL_DEG = 5.0
STEADY_MAX_RATE_RADS = 0.5
STEADY_SETTLE_S = 0.2


def _altitude_target(paid_out_m):
    return max(
        TAKEOFF_ALT_TARGET_M,
        min(paid_out_m, MIN_TETHER_LENGTH_M) * math.sin(TETHER_ELEVATION_RAD),
    )


@pytest.mark.parametrize(
    ("paid_out_m", "expected"),
    [(1.0, 3.0), (6.0, 3.0), (50.0, 50.0 * math.sin(math.radians(25.0))),
     (60.0, 50.0 * math.sin(math.radians(25.0)))],
)
def test_altitude_command_follows_tether_elevation_not_vertical_payout(paid_out_m, expected):
    assert _altitude_target(paid_out_m) == pytest.approx(expected)


def _handoff_ready(R, pos, gyro, paid_out_m):
    if paid_out_m < HANDOFF_MIN_LENGTH_M:
        return False
    length = float(np.linalg.norm(pos))
    elevation = math.asin(float(np.clip(-pos[2] / length, -1.0, 1.0)))
    goal = compute_bz_altitude_hold(pos, elevation, LAUNCH_TENSION_N, _ROTOR.inertia.mass_kg)
    return (
        goal[2] <= math.cos(math.radians(STEADY_MIN_TILT_DEG))
        and float(np.dot(R[:, 2], goal)) >= math.cos(math.radians(HANDOFF_AXIS_TOL_DEG))
        and float(np.linalg.norm(gyro)) <= STEADY_MAX_RATE_RADS
    )


def test_handoff_requires_inclined_geometry_not_just_climb_height():
    pos = np.array([0.0, 8.0, -3.0])
    elevation = math.atan2(3.0, 8.0)
    goal = compute_bz_altitude_hold(pos, elevation, LAUNCH_TENSION_N, _ROTOR.inertia.mass_kg)
    R = build_orb_frame(goal)
    assert _handoff_ready(R, pos, np.zeros(3), 8.5)
    assert not _handoff_ready(np.eye(3), pos, np.zeros(3), 8.5)
    assert not _handoff_ready(R, pos, np.zeros(3), 7.999)
    assert not _handoff_ready(R, pos, np.array([0.6, 0.0, 0.0]), 8.5)
    assert not _handoff_ready(np.eye(3), np.array([0.0, 0.0, -17.0]), np.zeros(3), 17.0)


def _steady_orientation_reached(R, target, gyro, paid_out_m) -> bool:
    if target is None:
        return False
    target = np.asarray(target, dtype=float)
    if not np.isfinite(target).all() or np.linalg.norm(target) < 0.5:
        return False
    target = target / np.linalg.norm(target)
    return (
        paid_out_m >= MIN_TETHER_LENGTH_M
        and target[2] <= math.cos(math.radians(STEADY_MIN_TILT_DEG))
        and float(np.dot(R[:, 2], target)) >= math.cos(math.radians(STEADY_AXIS_TOL_DEG))
        and float(np.linalg.norm(gyro)) <= STEADY_MAX_RATE_RADS
    )


@pytest.mark.parametrize(
    ("roll_deg", "goal_deg", "rate", "paid_out_m", "reached"),
    [
        (0.0, 0.0, 0.0, 50.0, False),
        (20.0, 20.0, 0.0, 50.0, False),
        (40.0, 40.0, 0.0, 50.0, True),
        (36.0, 40.0, 0.0, 50.0, True),
        (34.0, 40.0, 0.0, 50.0, False),
        (40.0, 40.0, 0.6, 50.0, False),
        (40.0, 40.0, 0.0, 49.999, False),
        (40.0, 40.0, 0.0, 60.0, True),
    ],
)
def test_steady_orientation_requires_length_tilt_tracking_and_low_rates(
    roll_deg, goal_deg, rate, paid_out_m, reached,
):
    roll = math.radians(roll_deg)
    goal = math.radians(goal_deg)
    R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(roll), -math.sin(roll)],
        [0.0, math.sin(roll), math.cos(roll)],
    ])
    target = np.array([0.0, -math.sin(goal), math.cos(goal)])
    assert _steady_orientation_reached(R, target, np.array([rate, 0.0, 0.0]), paid_out_m) == reached


def _build_ic() -> SimpleNamespace:
    aero = create_aero(_ROTOR, model="quasi_static")

    def vertical_force(thrust: float) -> float:
        inputs = RotorInputs(
            collective_rad=thrust_to_coll_rad(thrust),
            tilt_lon=0.0, tilt_lat=0.0, R_hub=np.eye(3),
            v_hub_world=np.zeros(3), wind_world=WIND_NED,
            omega_rad_s=OMEGA_START_RAD_S, rho_kg_m3=1.225,
        )
        result, _ = aero.step(inputs, aero.initial_rotor_state(), DT)
        return float(result.F_world[2]) + _ROTOR.inertia.mass_kg * 9.81 + LAUNCH_TENSION_N

    initial_alt = 1.0
    attachment_length = initial_alt - _ROTOR.control.axle_attachment_length_m
    return SimpleNamespace(
        pos=np.array([0.0, 0.0, -initial_alt]),
        vel=np.zeros(3), R0=np.eye(3),
        rest_length=attachment_length * (
            1.0 - LAUNCH_TENSION_N / TetherProperties().axial_stiffness_n
        ),
        eq_thrust=brentq(vertical_force, 0.0, 1.0),
        omega_spin=OMEGA_START_RAD_S,
    )


def _failure_reason(
    t: float, altitude: float, tension: float,
    liftoff_t: float | None, transition_t: float | None,
) -> str | None:
    if not math.isfinite(altitude) or not math.isfinite(tension):
        return f"Non-finite flight state at t={t:.2f} s"
    if altitude <= 0.0:
        return f"Hub hit the ground at t={t:.2f} s"
    if tension <= 0.0:
        return f"Tether went slack at t={t:.2f} s"
    if tension >= TetherProperties().break_load_n:
        return f"Tether load {tension:.1f} N reached breaking load at t={t:.2f} s"
    if liftoff_t is None and t >= LIFTOFF_TIMEOUT:
        return f"No liftoff within {LIFTOFF_TIMEOUT:.1f} simulated seconds"
    if transition_t is None and t >= TRANSITION_TIMEOUT:
        return f"No takeoff transition within {TRANSITION_TIMEOUT:.1f} simulated seconds"
    return None


@pytest.mark.parametrize(
    ("t", "altitude", "tension", "liftoff_t", "transition_t", "reason"),
    [
        (0.1, 0.0, 120.0, None, None, "hit the ground"),
        (3.0, 0.0, 120.0, 0.3, 2.5, "hit the ground"),
        (0.1, float("nan"), 120.0, None, None, "Non-finite"),
        (0.1, 1.0, float("nan"), None, None, "Non-finite"),
        (0.1, 1.0, 0.0, None, None, "went slack"),
        (0.1, 1.0, -1.0, None, None, "went slack"),
        (0.1, 1.0, 620.0, None, None, "breaking load"),
        (LIFTOFF_TIMEOUT, 1.0, 120.0, None, None, "No liftoff"),
        (TRANSITION_TIMEOUT, 5.0, 120.0, 0.3, None, "No takeoff transition"),
        (LIFTOFF_TIMEOUT - DT, 1.0, 120.0, None, None, None),
        (TRANSITION_TIMEOUT - DT, 5.0, 120.0, 0.3, None, None),
        (100.0, 50.0, 100.0, 0.3, 2.5, None),
        (100.0, 50.0, 50.0, 0.3, 2.5, None),
        (100.0, 50.0, 99.9, 0.3, 2.5, None),
    ],
)
def test_ground_liftoff_failure_limits(t, altitude, tension, liftoff_t, transition_t, reason):
    failure = _failure_reason(t, altitude, tension, liftoff_t, transition_t)
    if reason is None:
        assert failure is None
    else:
        assert failure is not None and reason in failure


def test_ground_liftoff(simtest_log):
    ic = _build_ic()
    sim = RawesLua(mode=MODE_TAKEOFF)
    sim.armed = True
    sim.healthy = True
    sim.vehicle_mode = 4
    winch = TorqueWinchController(
        rest_length=ic.rest_length, reflected_mass_kg=0.2, speed_damping_ns_m=20.0,
        min_length=0.5, max_length=WINCH_MAX_LENGTH_M,
    )
    node = GovernedWinchNode(winch, Anemometer())
    node.exchange(WinchCommand(cruise_v=0.0, tension_target=LAUNCH_TENSION_N))
    runner = PhysicsRunner(
        _ROTOR, ic, WIND_NED, z_floor=0.0, winch=winch, tether_model="spring_damper",
    )
    node.update_sensors(runner.anchor_tension, WIND_NED)
    lua = MockArdupilot.for_lua(sim, initial_thrust=ic.eq_thrust, wind=WIND_NED, dt=DT)
    lua.winch = winch
    phase = "takeoff"
    lua.tel_fn = lambda r, sr: {"phase": phase, "gnd_alt_cmd_m": _altitude_target(node.rest_length)}
    send_anchor_ned(sim, 0.0, 0.0, 0.0)

    liftoff_t = None
    transition_t = None
    payout_reached_t = None
    orientation_reached_t = None
    steady_t = None
    capture_thrust = ic.eq_thrust
    abort_reason = None
    min_tension = runner.tension_now
    max_tension = runner.tension_now
    max_alt = runner.altitude

    def inject(s, r):
        if transition_t is None:
            thrust = TAKEOFF_THRUST_SEED
            wind = node.telemetry().wind_ned
            if math.hypot(wind[0], wind[1]) < 1.0:
                raise RuntimeError("Wind-capture takeoff requires a valid horizontal anemometer reading")
            for name, value in (
                ("RAWES_LEAN", TAKEOFF_LEAN_RAD), ("RAWES_AZ", math.atan2(wind[1], wind[0])),
            ):
                s.send_message(NamedValueFloat(name=name, value=value))
        else:
            thrust = capture_thrust
        for name, value in (
            ("RAWES_ALT", _altitude_target(node.rest_length)), ("RAWES_THR", thrust),
            ("RAWES_TEN", LAUNCH_TENSION_N),
        ):
            s.send_message(NamedValueFloat(name=name, value=value))

    for i in range(round(TOTAL_RUNTIME / DT)):
        if i % PLANNER_EVERY == 0:
            node.exchange(WinchCommand(
                cruise_v=WINCH_CRUISE_OUT if liftoff_t is not None else 0.0,
                tension_target=LAUNCH_TENSION_N,
            ))
        if i % LUA_EVERY == 0:
            lua.tick(i * DT, runner, inject=inject)
        node.update_sensors(runner.anchor_tension, WIND_NED)
        sr = lua.step(runner, DT)
        lua.log(runner, sr)

        alt = runner.altitude
        tension = runner.tension_now
        min_tension = min(min_tension, sr["tension_min"])
        max_tension = max(max_tension, sr["tension_max"])
        max_alt = max(max_alt, alt)
        if payout_reached_t is None and node.rest_length >= MIN_TETHER_LENGTH_M:
            payout_reached_t = runner.t_sim
        if liftoff_t is None and alt >= LIFTOFF_ALT_M:
            liftoff_t = runner.t_sim
            phase = "wind-capture"
        obs = runner.observe()
        if transition_t is None and _handoff_ready(obs.R, obs.pos, obs.gyro, node.rest_length):
            transition_t = runner.t_sim
            capture_thrust = lua.thrust
            sim.set_param("RAWES_MODE", MODE_STEADY)
            phase = "reel-out"
        abort_reason = (
            _failure_reason(runner.t_sim, alt, sr["tension_min"], liftoff_t, transition_t)
            or _failure_reason(runner.t_sim, alt, sr["tension_max"], liftoff_t, transition_t)
        )
        if abort_reason is not None:
            break
        if transition_t is not None and sim.fns.el_initialized():
            goal = [sim.fns.diag_nvf(key) for key in ("BZG_N", "BZG_E", "BZG_D")]
            if all(value is not None for value in goal) and _steady_orientation_reached(
                obs.R, goal, obs.gyro, node.rest_length,
            ):
                if orientation_reached_t is None:
                    orientation_reached_t = runner.t_sim
                if runner.t_sim - orientation_reached_t >= STEADY_SETTLE_S:
                    steady_t = runner.t_sim
                    phase = "steady"
                    break
            else:
                orientation_reached_t = None
            if payout_reached_t is not None and runner.t_sim - payout_reached_t >= STEADY_TIMEOUT_S:
                abort_reason = (
                    f"No steady-flight orientation within {STEADY_TIMEOUT_S:.1f} s "
                    f"of reaching {MIN_TETHER_LENGTH_M:.1f} m payout"
                )
                break
        if i % round(10.0 / DT) == 0:
            print(
                f"t={runner.t_sim:.1f}s alt={alt:.2f}m spin={runner.omega_spin:.2f}rad/s "
                f"length={node.rest_length:.2f}m tension={tension:.1f}N",
                flush=True,
            )
            lua.write_telemetry(simtest_log.log_dir / "telemetry.csv")

    if abort_reason is None and steady_t is None:
        abort_reason = (
            f"Did not establish steady-flight orientation with at least "
            f"{MIN_TETHER_LENGTH_M:.1f} m payout within {TOTAL_RUNTIME:.1f} simulated seconds"
        )
    if not lua.telemetry or lua.telemetry[-1].t_sim != runner.t_sim:
        lua.log(runner, sr, force=True)
    lua.telemetry[-1].note = abort_reason or "steady_orientation_50m"
    final_length = float(np.linalg.norm(runner.hub_state["pos"]))
    lines = [
        f"t_final={runner.t_sim:.3f}s max_altitude={max_alt:.3f}m",
        f"liftoff_t={liftoff_t} transition_t={transition_t}",
        f"payout_reached_t={payout_reached_t} steady_t={steady_t}",
        f"final_length={final_length:.3f}m final_altitude={runner.altitude:.3f}m",
        f"paid_out_length={node.rest_length:.3f}m",
        f"tension_min={min_tension:.3f}N tension_max={max_tension:.3f}N",
        f"stopped={abort_reason or 'scenario complete'}",
    ]
    lua.write_telemetry(simtest_log.log_dir / "telemetry.csv")
    simtest_log.write(lines, "ground_liftoff")
    print("\n".join(lines), flush=True)

    assert abort_reason is None, abort_reason
    assert liftoff_t is not None and liftoff_t < LIFTOFF_TIMEOUT
    assert transition_t is not None and transition_t < TRANSITION_TIMEOUT
    assert node.rest_length >= MIN_TETHER_LENGTH_M, "Less than 50 m of tether paid out"
    assert steady_t is not None, "Steady-flight orientation was not established"
