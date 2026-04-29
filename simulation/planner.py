"""
trajectory.py — Pluggable trajectory controllers for the RAWES pumping cycle.

Models the offboard MAVLink trajectory planner that runs on a ground station
or companion computer and sends commands to the custom ArduPilot Mode_RAWES.

Protocol
--------
Two packet types cross the MAVLink boundary:

    STATE packet  (ground station assembles each cycle — MAVLink streams + local sensors):
        "pos_ned"        np.ndarray [3]  — hub position NED [m]        (LOCAL_POSITION_NED)
        "vel_ned"        np.ndarray [3]  — hub velocity NED [m/s]      (LOCAL_POSITION_NED)
        "omega_spin"     float           — rotor spin rate [rad/s]      (ESC_STATUS rpm × gear ratio)
        "body_z"         np.ndarray [3]  — rotor axis NED unit vector   (ATTITUDE_QUATERNION col 2)
        "tension_n"      float           — tether tension [N]           (winch load cell — local)
        "tether_length_m" float          — tether rest length [m]       (winch encoder — local)

    Planner-internal quantities (never on wire):
        t_free          float  — elapsed free-flight [s]         — planner internal clock

    COMMAND packet  (planner → Pixhawk, ~10 Hz):
        "attitude_q"      np.ndarray [4]  (w,x,y,z)                (SET_ATTITUDE_TARGET quaternion)
            Desired disk orientation in NED.
            [1,0,0,0] (identity) = tether-aligned natural orbit — Mode_RAWES tracks
            the tether direction at 400 Hz without any planner correction.
            Non-identity = desired body_z is quat_apply(attitude_q, [0,0,-1]) in NED.
            Mode_RAWES rate-limits the slew internally (body_z_slew_rate_rad_s).
        "thrust"          float [0..1]                              (SET_ATTITUDE_TARGET thrust)
            Normalised collective, direct output of the tension PI:
              collective_rad = kP * err + kI * integral
              thrust = clamp((collective_rad - col_min) / (col_max - col_min), 0, 1)
            Mode_RAWES calls set_throttle_out(thrust) — direct passthrough, no conversion.
            Simulation mediator denormalises: collective_rad = col_min + thrust*(col_max-col_min).
        "phase"           str — telemetry label ("hold"|"reel-out"|"reel-in"). Not on wire.

    Winch command  (planner → WinchController, ground-local link):
        "winch_speed_ms"  float [m/s]                               (MAV_CMD_DO_WINCH RATE_CONTROL)
            +ve = pay out, −ve = reel in, 0 = hold.
            Passed to WinchController.step() — the Pixhawk is not involved.

Mode_RAWES responsibilities (NOT in this file):
    - AltitudeHoldController: rate-limit elevation toward asin(target_alt/tlen), compute body_z_eq with gravity-compensation tilt
    - Attitude error → cyclic tilt  (compute_swashplate_from_state)
    - Collective: TensionPI running on rawes.lua in SITL; set_throttle_out(thrust) passthrough on hardware
    - Counter-torque motor (inner loop, not commanded by planner)

WinchController responsibilities (NOT in this file — see winch.py):
    - Execute winch_speed_ms commands
    - Enforce tension safety limit (stop paying out if tension too high)
    - Write tension_n and tether_length_m into the STATE packet each step

Available controllers
---------------------
    HoldPlanner()
        Returns natural equilibrium command every step.
        attitude_q = identity → Mode_RAWES stays tether-aligned through orbit tracking.
        Use for: test_closed_loop_90s.py

"""

import math
import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Quaternion utilities (module-level, importable)
# ---------------------------------------------------------------------------

#: Quaternion identity [w, x, y, z] — represents no rotation.
Q_IDENTITY: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_vectors(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """
    Quaternion [w, x, y, z] that rotates unit vector *v1* to unit vector *v2*.

    The quaternion satisfies:  quat_apply(q, v1) ≈ v2.
    Both vectors are normalised internally.
    """
    v1 = np.asarray(v1, dtype=float)
    v2 = np.asarray(v2, dtype=float)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    c = float(np.dot(v1, v2))
    if c >= 1.0 - 1e-9:
        return Q_IDENTITY.copy()
    if c <= -1.0 + 1e-9:
        # Anti-parallel: 180° rotation around any perpendicular axis
        perp = np.array([1.0, 0.0, 0.0]) if abs(v1[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(v1, perp)
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    axis = np.cross(v1, v2)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(c)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), s * axis[0], s * axis[1], s * axis[2]])


def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotate vector *v* by quaternion *q* = [w, x, y, z].

    Equivalent to:  q ⊗ [0, v] ⊗ q*
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    v = np.asarray(v, dtype=float)
    xyz = np.array([x, y, z])
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def quat_is_identity(q: np.ndarray, atol: float = 1e-6) -> bool:
    """Return True if *q* is the identity quaternion within *atol*."""
    return abs(float(q[0]) - 1.0) < atol and float(np.linalg.norm(q[1:])) < atol


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class TrajectoryPlanner:
    """Abstract base class — subclass and implement step()."""

    def step(self, state: dict, dt: float, **kwargs) -> dict:
        """
        Advance one planner step.

        Parameters
        ----------
        state : dict — STATE packet assembled by the ground station each cycle:
            pos_ned          np.ndarray [3] — hub position NED [m]        (LOCAL_POSITION_NED)
            vel_ned          np.ndarray [3] — hub velocity NED [m/s]      (LOCAL_POSITION_NED)
            omega_spin       float          — rotor spin rate [rad/s]      (ESC_STATUS)
            body_z           np.ndarray [3] — rotor axis NED unit vector   (ATTITUDE_QUATERNION)
            tension_n        float          — tether tension [N]           (winch load cell)
            tether_length_m  float          — tether rest length [m]       (winch encoder)
        dt    : float — timestep [s]

        Returns
        -------
        COMMAND packet dict with keys:
            attitude_q, thrust, winch_speed_ms, phase
        """
        raise NotImplementedError(
            f"{type(self).__name__} must implement step()")


# ---------------------------------------------------------------------------
# HoldPlanner
# ---------------------------------------------------------------------------

class HoldPlanner(TrajectoryPlanner):
    """
    Natural equilibrium — no correction, no winch.

    Sends identity attitude_q and zero thrust every step.  Mode_RAWES stays
    tether-aligned through its own orbit-tracking (no planner involvement).

    Use for: closed-loop stability tests where the pumping cycle is inactive.
    """

    def step(self, state: dict, dt: float, **kwargs) -> dict:
        return {
            "attitude_q":     Q_IDENTITY.copy(),
            "thrust":         0.0,
            "winch_speed_ms": 0.0,
            "phase":          "hold",
        }

