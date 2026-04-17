#!/usr/bin/env python3
"""
mediator_torque.py — Counter-torque motor stack-test mediator

Bridges ArduPilot SITL ↔ hub yaw dynamics for the counter-torque motor test
using SITLInterface for all binary servo I/O and JSON state serialisation.

Physical scenario
-----------------
  • Hub sits stationary at NED = [0, 0, 0], roll = 0, pitch = 0.
  • The rotor hub spins at ``--omega-rotor`` rad/s (simulated autorotation).
  • The motor counter-rotates via the 80:44 gear to maintain inner assembly heading.
  • Bearing and swashplate friction is the load the motor works against.
  • ArduPilot sees the yaw rate in the gyro and commands the tail-rotor
    channel (Ch4) to hold the counter-rotation speed via the GB4008 motor.

Sensor data sent to ArduPilot (via SITLInterface.send_state)
---------------------------------------------------------------------------
  position    [0, 0, 0]  NED [m]  — hub is stationary
  velocity    [0, 0, 0]  NED [m/s]
  attitude    [roll, pitch, −ψ]   — hub attitude (NED convention)
  gyro_body   [p, q, r]           — body-frame angular velocity [rad/s]
  accel_body  [ax, ay, az]        — gravity only [m/s²] projected into body frame
  rpm         ω_axle × gear → RPM — rotor RPM for ArduPilot RSC (RPM1_TYPE=10)

ArduPilot → mediator (binary servo packet over UDP 9002)
---------------------------------------------------------------------------
  Ch4 (index 3) → tail/yaw → motor throttle → hub yaw dynamics

  PWM → throttle mapping (H_TAIL_TYPE = 2, DDFP unidirectional):
      throttle = (pwm_us − 1000) / 1000   ∈ [0, 1]
      1000 µs → 0%, 2000 µs → 100%

Usage (Docker / WSL)
--------------------
    python3 mediator_torque.py [--omega-rotor FLOAT] [--log-level LEVEL]

Options
-------
    --omega-rotor  FLOAT   Rotor hub spin rate [rad/s] (default: 28.0)
    --log-level    LEVEL   Logging level (default: INFO)
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path

import numpy as np

# simulation/ — add so SITLInterface, torque_model, mediator_events are importable.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import torque_model as _m
from sitl_interface import SITLInterface
from mediator_events import MediatorEventLog

# ---------------------------------------------------------------------------
# Timing / port constants
# ---------------------------------------------------------------------------

_RECV_PORT = 9002        # must match ArduPilot SITL JSON backend default
DT         = 1.0 / 400.0  # 400 Hz loop target [s]

# Default ArduPilot channel for yaw/tail-rotor (0-based index → Ch4).
# When --tail-channel 9 is passed (Lua scripting mode), reads Ch9 (index 8).
_CH_YAW_DEFAULT = 3

# Log interval [s]
_LOG_INTERVAL = 1.0

# Gear ratio: motor axle speed → RPM sent to ArduPilot RSC
_GEAR_RATIO = 80.0 / 44.0


def _pwm_to_throttle(pwm_us: float, trim: float = 0.0) -> float:
    """
    Convert PWM microseconds to throttle [0, 1].

    Biased mapping so that neutral PWM (1500 µs) corresponds to the
    equilibrium throttle ``trim`` rather than to 50%:

        pwm = 1000 µs  →  0.0
        pwm = 1500 µs  →  trim      (neutral = equilibrium)
        pwm = 2000 µs  →  1.0

    This lets ArduPilot's PID operate symmetrically around zero output
    (neutral stick = equilibrium = no yaw rate error), so no I-term
    wind-up is required to overcome the motor's back-EMF dead zone.

    If trim = 0 (default), the mapping collapses to the linear
    (pwm − 1000) / 1000 form used by standard DDFP.
    """
    trim = max(0.0, min(1.0, trim))
    if pwm_us >= 1500.0:
        t = trim + (1.0 - trim) * (pwm_us - 1500.0) / 500.0
    else:
        t = trim * (pwm_us - 1000.0) / 500.0
    return max(0.0, min(1.0, t))


def _body_vectors(
    roll: float, pitch: float,
    psi_dot: float, roll_dot: float, pitch_dot: float,
) -> tuple[list[float], list[float]]:
    """
    Convert Euler angles + rates to body-frame gyro and specific-force vectors.

    Returns (gyro_body [rad/s], accel_body [m/s²]).

    NED convention throughout: psi_dot is NED yaw rate [rad/s], positive = CW from above.

    For a tilted hub (roll φ, pitch θ) gravity projects into body frame:
      g_body_x =  9.81 * sin(θ)
      g_body_y =  9.81 * cos(θ) * sin(φ)
      g_body_z = -9.81 * cos(θ) * cos(φ)
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    g = 9.81

    # ZYX Euler rates → body rates (NED, all rates positive = CW / nose-up / roll-right)
    #   p =  φ̇ − ψ̇ sin(θ)
    #   q =  θ̇ cos(φ) + ψ̇ cos(θ) sin(φ)
    #   r = −θ̇ sin(φ) + ψ̇ cos(θ) cos(φ)
    gyro = [
        float(roll_dot  - psi_dot * sp),
        float(pitch_dot * cr + psi_dot * sr * cp),
        float(-pitch_dot * sr + psi_dot * cr * cp),
    ]
    accel = [
        float(g * sp),
        float(g * cp * sr),
        float(-g * cp * cr),
    ]
    return gyro, accel


# ---------------------------------------------------------------------------
# Axle speed profiles  —  omega_fn(dynamics_t, omega_nom) -> float [rad/s]
# Tilt profiles        —  tilt_fn(dynamics_t)              -> (roll, pitch) [rad]
# ---------------------------------------------------------------------------

def _omega_constant(dt: float, nom: float) -> float:
    return nom

def _omega_slow_vary(dt: float, nom: float) -> float:
    """±5 rad/s sinusoidal at 0.05 Hz (20 s period)."""
    return nom + 5.0 * math.sin(2 * math.pi * 0.05 * dt)

def _omega_fast_vary(dt: float, nom: float) -> float:
    """±5 rad/s sinusoidal at 0.25 Hz (4 s period)."""
    return nom + 5.0 * math.sin(2 * math.pi * 0.25 * dt)

def _omega_gust(dt: float, nom: float) -> float:
    """Steady until dynamics_t=10 s; then a 5-second 20% overspeed gust.
    150% was too large for motor authority (eq_throttle > 1.0 at 42 rad/s).
    120% keeps eq_throttle ≈ 0.90 — within motor range with adaptive trim."""
    if 10.0 <= dt <= 15.0:
        return nom * 1.20
    return nom

def _omega_startup(dt: float, nom: float) -> float:
    """Linear spin-up from 0 to nominal over 30 s, then hold.
    Simulates the rotor accelerating from rest to autorotation speed.
    The adaptive trim tracks the changing counter-rotation speed at every RPM point."""
    T_RAMP = 30.0
    return nom * min(1.0, dt / T_RAMP)

def _tilt_flat(dt: float) -> tuple[float, float]:
    return 0.0, 0.0

def _tilt_pitch_roll(dt: float) -> tuple[float, float]:
    """Roll ±5° at 0.08 Hz, pitch ±3° at 0.05 Hz.
    Kept small so horizontal gravity component (g·sin θ ≈ 0.5 m/s²) stays
    below the EKF accel noise threshold and avoids GPS Glitch false positives."""
    roll  = math.radians(5.0) * math.sin(2 * math.pi * 0.08 * dt)
    pitch = math.radians(3.0) * math.sin(2 * math.pi * 0.05 * dt)
    return roll, pitch

def _tilt_wobble(dt: float) -> tuple[float, float]:
    """High-tilt wobble: ±20° roll / ±15° pitch at orbital frequency ~0.1 Hz.
    Simulates a RAWES hub under high cyclic swashplate tilt (e.g. steep orbit,
    strong crosswind correction, or large commanded heading change).
    Two frequency components create a less regular, more realistic wobble.
    GPS position/velocity fusion must be disabled in the test fixture to
    prevent GPS Glitch from the large horizontal gravity projection (~3.4 m/s²)."""
    roll  = math.radians(20.0) * math.sin(2 * math.pi * 0.10 * dt)
    pitch = math.radians(15.0) * math.cos(2 * math.pi * 0.10 * dt)
    roll  += math.radians(5.0) * math.sin(2 * math.pi * 0.23 * dt + 1.0)
    pitch += math.radians(4.0) * math.sin(2 * math.pi * 0.17 * dt + 0.5)
    return roll, pitch

PROFILES: dict[str, tuple] = {
    "startup":    (_omega_startup,   _tilt_flat),
    "constant":   (_omega_constant, _tilt_flat),
    "slow_vary":  (_omega_slow_vary, _tilt_flat),
    "fast_vary":  (_omega_fast_vary, _tilt_flat),
    "gust":       (_omega_gust,      _tilt_flat),
    "pitch_roll": (_omega_constant,  _tilt_pitch_roll),
    "wobble":     (_omega_constant,  _tilt_wobble),
}


def run(
    omega_rotor: float,
    startup_hold_s: float = 5.0,
    trim_throttle: float | None = None,
    profile: str = "constant",
    tail_channel: int = _CH_YAW_DEFAULT,
    lua_mode: bool = False,
    log_level: str = "INFO",
    events_log_path: "str | None" = None,
) -> None:
    """
    Main mediator loop.

    Startup hold phase (0 … startup_hold_s)
    ----------------------------------------
    Hub is locked stationary at ψ=0.  A slow constant yaw drift of 5°/s is
    sent to SITL to give ArduPilot's EKF enough gyro and compass data to
    initialise.  Bearing drag and motor dynamics are NOT active.

    Dynamic phase (startup_hold_s … ∞)
    ------------------------------------
    Real hub yaw dynamics: the motor counter-rotates via the 80:44 gear to
    maintain hub heading; ArduPilot's tail-rotor PID (Ch4) controls the
    GB4008 motor speed.

    Parameters
    ----------
    omega_rotor    : rotor hub spin rate [rad/s] (nominal autorotation speed)
    startup_hold_s : duration of EKF-initialisation spin phase [s]
    log_level      : Python logging level string
    """
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    log = logging.getLogger("mediator_torque")
    ev  = MediatorEventLog(events_log_path)
    ev.open()

    params  = _m.HubParams()
    state   = _m.HubState()
    t       = 0.0

    _STARTUP_YAW_RATE = math.radians(5.0)

    if trim_throttle is None:
        trim_throttle = _m.equilibrium_throttle(omega_rotor, params)

    eq_throttle = _m.equilibrium_throttle(omega_rotor, params)
    omega_fn, tilt_fn = PROFILES.get(profile, PROFILES["constant"])
    ch_yaw = tail_channel

    ev.write("startup", t_sim=0.0,
             omega_rotor_rad_s=round(omega_rotor, 2),
             omega_rotor_rpm=round(omega_rotor * 60.0 / (2.0 * math.pi)),
             k_bearing=round(params.k_bearing, 4),
             gear_ratio=round(params.gear_ratio, 3),
             tau_stall_nm=round(params.tau_stall, 2),
             eq_throttle=round(eq_throttle, 4),
             trim_throttle=round(trim_throttle, 4),
             startup_hold_s=round(startup_hold_s, 1),
             profile=profile,
             tail_channel=ch_yaw,
             lua_mode=lua_mode)

    throttle      = 0.0
    n_sent        = 0
    last_log      = 0.0
    prev_roll     = 0.0
    prev_pitch    = 0.0
    trim_fixed    = float(trim_throttle)
    last_pwm_ch4  = 1500.0   # neutral default
    throttle      = _pwm_to_throttle(last_pwm_ch4, trim_fixed)

    iface = SITLInterface(recv_port=_RECV_PORT)
    iface.bind()
    log.info("Bound to UDP port %d", _RECV_PORT)

    try:
        while True:
            # ── Receive servo packet from SITL ─────────────────────────────
            # recv_servos() returns None on timeout; servos are normalised [-1, 1].
            # Convert the relevant channel back to PWM µs for _pwm_to_throttle.
            servos = iface.recv_servos()
            if servos is not None:
                last_pwm_ch4 = 1500.0 + servos[ch_yaw] * 500.0

            in_startup = (t < startup_hold_s)
            dynamics_t = max(0.0, t - startup_hold_s)

            if in_startup:
                psi_send      = (_STARTUP_YAW_RATE * t) % (2.0 * math.pi)
                psi_dot_send  = _STARTUP_YAW_RATE
                roll_send     = 0.0
                pitch_send    = 0.0
                roll_dot      = 0.0
                pitch_dot     = 0.0
                current_omega = omega_rotor
                throttle      = _pwm_to_throttle(last_pwm_ch4, trim_fixed)
            else:
                current_omega = max(1.0, omega_fn(dynamics_t, omega_rotor))

                if lua_mode:
                    # PWM range 800 (off) to 2000 (full): matches SERVO9_MIN=800, SERVO9_MAX=2000
                    throttle = max(0.0, min(1.0, (last_pwm_ch4 - 800.0) / 1200.0))
                else:
                    current_trim = _m.equilibrium_throttle(current_omega, params)
                    throttle     = _pwm_to_throttle(last_pwm_ch4, current_trim)

                roll_send, pitch_send = tilt_fn(dynamics_t)
                roll_dot  = (roll_send  - prev_roll)  / DT
                pitch_dot = (pitch_send - prev_pitch) / DT

                if state.psi == 0.0 and state.psi_dot == 0.0:
                    state.psi = (_STARTUP_YAW_RATE * startup_hold_s) % (2.0 * math.pi)
                state = _m.step(state, current_omega, throttle, params, DT)
                psi_send     = math.atan2(math.sin(state.psi), math.cos(state.psi))
                psi_dot_send = state.psi_dot

            # Safety clamp: cap yaw rate to prevent ArduPilot SIGFPE
            _MAX_PSI_DOT = math.radians(500.0)
            psi_dot_send = max(-_MAX_PSI_DOT, min(_MAX_PSI_DOT, psi_dot_send))

            prev_roll, prev_pitch = roll_send, pitch_send
            t += DT

            # ── Send state back to SITL via SITLInterface ──────────────────
            gyro_body, accel_body = _body_vectors(
                roll_send, pitch_send, psi_dot_send, roll_dot, pitch_dot,
            )
            iface.send_state(
                pos_ned         = np.array([0.0, 0.0, 0.0]),
                vel_ned         = np.array([0.0, 0.0, 0.0]),
                rpy_rad         = np.array([roll_send, pitch_send, psi_send]),
                accel_body      = np.asarray(accel_body),
                gyro_body       = np.asarray(gyro_body),
                rpm_rad_s       = current_omega * _GEAR_RATIO,
                # Override SITL battery simulation with the nominal voltage used
                # by the Python physics model so that rawes.lua's compute_trim()
                # sees the same voltage and produces the same equilibrium throttle.
                battery_voltage = _m.BATTERY_V,
            )
            n_sent += 1

            # ── Periodic log ───────────────────────────────────────────────
            if t - last_log >= _LOG_INTERVAL:
                last_log = t
                phase = "STARTUP" if in_startup else "DYNAMIC"
                ev.write("heartbeat", t_sim=round(t, 1),
                         phase=phase,
                         psi_deg=round(math.degrees(psi_send), 2),
                         psi_dot_deg_s=round(math.degrees(psi_dot_send), 3),
                         throttle=round(throttle, 3),
                         omega_rad_s=round(current_omega if not in_startup else omega_rotor, 1),
                         roll_deg=round(math.degrees(roll_send), 1),
                         pitch_deg=round(math.degrees(pitch_send), 1))
                if not in_startup and t - startup_hold_s < 2.0:
                    ev.write("dynamics_start", t_sim=round(t, 1))

    except KeyboardInterrupt:
        log.info("Interrupted — shutting down")
    finally:
        ev.write("shutdown", t_sim=round(t, 1), n_sent=n_sent)
        ev.close()
        iface.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="RAWES counter-torque motor mediator (stationary hub, yaw-only)"
    )
    parser.add_argument(
        "--omega-rotor", type=float, default=_m.OMEGA_ROTOR_NOMINAL,
        help=f"Rotor hub spin rate [rad/s] (default: {_m.OMEGA_ROTOR_NOMINAL:.1f})",
    )
    parser.add_argument(
        "--startup-hold", type=float, default=5.0,
        help="Duration of EKF-initialisation spin phase [s] (default: 5.0)",
    )
    parser.add_argument(
        "--trim-throttle", type=float, default=None,
        help="Neutral-PWM throttle bias (default: computed from equilibrium)",
    )
    parser.add_argument(
        "--profile", default="constant",
        choices=list(PROFILES.keys()),
        help="Axle speed / hub tilt profile (default: constant)",
    )
    parser.add_argument(
        "--tail-channel", type=int, default=_CH_YAW_DEFAULT,
        help="0-based servo channel index for tail motor (default: 3=Ch4, lua: 8=Ch9)",
    )
    parser.add_argument(
        "--lua-mode", action="store_true",
        help="Disable adaptive trim — Lua script provides feedforward instead",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        help="Logging level (default: INFO)",
    )
    parser.add_argument(
        "--events-log", default=None,
        help="Path for structured JSONL event log",
    )
    args = parser.parse_args()
    run(args.omega_rotor, args.startup_hold, args.trim_throttle,
        args.profile, args.tail_channel, args.lua_mode, args.log_level,
        events_log_path=args.events_log)
