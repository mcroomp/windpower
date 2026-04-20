#!/usr/bin/env python3
"""
mediator_torque.py — Counter-torque motor stack-test mediator

Bridges ArduPilot SITL ↔ hub yaw dynamics for the counter-torque motor test
using SITLInterface for all binary servo I/O and JSON state serialisation.

Physical scenario
-----------------
  • Hub sits stationary at NED = [0, 0, 0], roll = 0, pitch = 0.
  • STARTUP phase: rotor and motor are both at rest (omega = 0).  Hub does not
    rotate.  ArduPilot arms and EKF initialises with no yaw disturbance.
  • DYNAMIC phase: rotor spins up per the selected profile.  ArduPilot commands
    Ch4 PWM to drive the GB4008 motor via the 80:44 gear and counteract yaw.

Sensor data sent to ArduPilot (via SITLInterface.send_state)
---------------------------------------------------------------------------
  position    [0, 0, 0]  NED [m]  — hub is stationary
  velocity    [0, 0, 0]  NED [m/s]
  attitude    [roll, pitch, −ψ]   — hub attitude (NED convention)
  gyro_body   [p, q, r]           — body-frame angular velocity [rad/s]
  accel_body  [ax, ay, az]        — gravity only [m/s²] projected into body frame
  (no RPM field — PWM-only mode, no feedback channel from ESC to ArduPilot)

ArduPilot → mediator (binary servo packet over UDP 9002)
---------------------------------------------------------------------------
  Ch4 (index 3) → tail/yaw → motor throttle → hub yaw dynamics

  PWM → throttle mapping (SERVO4_MIN=800, SERVO4_MAX=2000):
      throttle = (pwm_us − 800) / 1200   ∈ [0, 1]
       800 µs → 0%, 2000 µs → 100%

  Motor speed follows commanded throttle with first-order lag (MOTOR_TAU=0.02 s):
      d(omega_motor)/dt = (throttle × RPM_SCALE − omega_motor) / MOTOR_TAU

Usage (Docker / WSL)
--------------------
    python3 mediator_torque.py [OPTIONS]

Options
-------
    --omega-rotor       FLOAT   Rotor hub spin rate [rad/s] (default: 28.0)
    --startup-hold      FLOAT   EKF-init spin phase duration [s] (default: 5.0)
    --profile           STR     Axle speed / tilt profile name (default: constant)
    --tail-channel      INT     0-based servo channel for motor (default: 3=Ch4, Lua: 8=Ch9)
    --log-level         LEVEL   Logging level (default: INFO)
    --events-log        PATH    Path for structured JSONL event log (default: none)
    --startup-yaw-rate  FLOAT   Yaw rate [deg/s] during startup hold (default: 5.0; 0=stationary)
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
from mediator_base import install_sigterm_handler, run_lockstep, setup_logging
from mediator_events import MediatorEventLog
from sitl_interface import SITLInterface

# ---------------------------------------------------------------------------
# Timing / port constants
# ---------------------------------------------------------------------------

_RECV_PORT   = 9002      # must match ArduPilot SITL JSON backend default
_BATTERY_V   = 15.2     # V — nominal 4S LiPo; sent to SITL to override battery simulation
DT         = 1.0 / 400.0  # 400 Hz loop target [s]

# Default ArduPilot channel for yaw/tail-rotor (0-based index → Ch4).
# When --tail-channel 9 is passed (Lua scripting mode), reads Ch9 (index 8).
_CH_YAW_DEFAULT = 3

# GB4008 motor PWM range — must match SERVO9_MIN/SERVO9_MAX in ArduPilot params
# and rawes.lua _set_throttle_pct (800 + pct * 12).
MOTOR_PWM_MIN = 800.0   # µs — motor off
MOTOR_PWM_MAX = 2000.0  # µs — full throttle


def _pwm_to_throttle(pwm_us: float) -> float:
    """Convert motor PWM microseconds to throttle [0, 1].

    Linear mapping over the GB4008 ESC range:
      800 µs → 0.0  (motor off)
     2000 µs → 1.0  (full throttle)

    Matches SERVO4_MIN=800 / SERVO4_MAX=2000 set in ArduPilot params for all tests.
    """
    return max(0.0, min(1.0, (pwm_us - MOTOR_PWM_MIN) / (MOTOR_PWM_MAX - MOTOR_PWM_MIN)))


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
    120% nominal keeps eq_throttle ≈ 0.58 — comfortable headroom for adaptive trim."""
    if 10.0 <= dt <= 15.0:
        return nom * 1.20
    return nom

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

# ---------------------------------------------------------------------------
# Prescribed-yaw profiles  —  psi_dot_fn(dynamics_t) -> float [rad/s]
# When a profile tuple has a 3rd element, psi_dot is directly prescribed
# (ODE bypassed). omega is still sent as RPM for the RSC.
# ---------------------------------------------------------------------------

def _yaw_zero(dt: float) -> float:
    """No yaw at all — tests that the motor stays at 800 µs."""
    return 0.0

def _yaw_slow_ramp(dt: float) -> float:
    """Ramp 0 → 10 deg/s over 30 s — tests that the DDFP PI winds up to cancel it."""
    return math.radians(10.0 * min(1.0, dt / 30.0))


PROFILES: dict[str, tuple] = {
    "startup":    (_omega_constant,  _tilt_flat),   # shared 10 s spinup ramp in DYNAMIC loop
    "constant":   (_omega_constant, _tilt_flat),
    "slow_vary":  (_omega_slow_vary, _tilt_flat),
    "fast_vary":  (_omega_fast_vary, _tilt_flat),
    "gust":       (_omega_gust,      _tilt_flat),
    "pitch_roll": (_omega_constant,  _tilt_pitch_roll),
    "wobble":     (_omega_constant,  _tilt_wobble),
    # Prescribed-yaw profiles (3rd element = psi_dot_fn, ODE bypassed)
    "yaw_zero":      (_omega_constant, _tilt_flat, _yaw_zero),
    "yaw_slow_ramp": (_omega_constant, _tilt_flat, _yaw_slow_ramp),
}


def run(
    omega_rotor: float,
    startup_hold_s: float = 5.0,
    profile: str = "constant",
    tail_channel: int = _CH_YAW_DEFAULT,
    log_level: str = "INFO",
    events_log_path: "str | None" = None,
    startup_yaw_rate: float = 0.0,
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
    maintain hub heading; ArduPilot's tail-rotor PID controls the GB4008 motor
    speed via the channel selected by ``tail_channel``.

    Parameters
    ----------
    omega_rotor      : rotor hub spin rate [rad/s] (nominal autorotation speed)
    startup_hold_s   : duration of EKF-initialisation spin phase [s]
    profile          : axle speed / tilt profile name (key in PROFILES dict)
    tail_channel     : 0-based servo channel index for the motor (3=Ch4, 8=Ch9)
    log_level        : Python logging level string
    events_log_path  : path for structured JSONL event log (None = disabled)
    startup_yaw_rate : yaw rate [rad/s] sent to SITL during startup hold
    """
    setup_logging(log_level)
    log = logging.getLogger("mediator_torque")
    is_stopped = install_sigterm_handler()
    ev  = MediatorEventLog(events_log_path)
    ev.open()

    params = _m.HubParams()
    _STARTUP_YAW_RATE = startup_yaw_rate

    _profile_entry = PROFILES.get(profile, PROFILES["constant"])
    omega_fn, tilt_fn = _profile_entry[0], _profile_entry[1]
    psi_dot_fn = _profile_entry[2] if len(_profile_entry) > 2 else None
    ch_yaw = tail_channel

    ev.write("startup", t_sim=0.0,
             omega_rotor_rad_s=round(omega_rotor, 2),
             omega_rotor_rpm=round(omega_rotor * 60.0 / (2.0 * math.pi)),
             rpm_scale=round(params.rpm_scale, 1),
             gear_ratio=round(params.gear_ratio, 3),
             motor_tau_ms=round(params.motor_tau * 1000, 1),
             startup_hold_s=round(startup_hold_s, 1),
             profile=profile,
             tail_channel=ch_yaw)

    # -- Mutable step state (closed over by step_fn) --------------------------
    hub_state  = _m.HubState()
    prev_roll  = 0.0
    prev_pitch = 0.0
    _dynamics_started = False
    _hb_state  = {"phase": "STARTUP", "psi_dot_deg_s": 0.0, "throttle": 0.0}

    iface = SITLInterface(recv_port=_RECV_PORT)
    iface.bind()
    log.info("Bound to UDP port %d", _RECV_PORT)

    def step_fn(_servos, t):
        nonlocal hub_state, prev_roll, prev_pitch, _dynamics_started

        # Motor PWM: read raw µs from interface to avoid -1.0 clipping at 800 µs idle.
        # This is physics input — throttle drives the counter-torque motor model.
        throttle = _pwm_to_throttle(iface.last_pwm_raw[ch_yaw])

        in_startup = t < startup_hold_s
        dynamics_t = max(0.0, t - startup_hold_s)

        if in_startup:
            psi_send     = (_STARTUP_YAW_RATE * t) % (2.0 * math.pi)
            psi_dot_send = _STARTUP_YAW_RATE
            roll_send = pitch_send = roll_dot = pitch_dot = 0.0
            current_omega = 0.0
        else:
            # Universal spin-up ramp: prevents discontinuous omega jump at DYNAMIC start.
            _SPINUP_S    = 10.0
            spinup_scale  = min(1.0, dynamics_t / _SPINUP_S)
            current_omega = max(1.0, omega_fn(dynamics_t, omega_rotor) * spinup_scale)

            roll_send, pitch_send = tilt_fn(dynamics_t)
            roll_dot  = (roll_send  - prev_roll)  / DT
            pitch_dot = (pitch_send - prev_pitch) / DT

            if psi_dot_fn is not None:
                # Prescribed yaw: bypass ODE, drive psi_dot directly from profile.
                psi_dot_send   = psi_dot_fn(dynamics_t)
                hub_state.psi  = (hub_state.psi + psi_dot_send * DT) % (2.0 * math.pi)
                hub_state.psi_dot = psi_dot_send
                psi_send       = hub_state.psi
            else:
                if hub_state.psi == 0.0 and hub_state.psi_dot == 0.0:
                    hub_state.psi = (_STARTUP_YAW_RATE * startup_hold_s) % (2.0 * math.pi)
                hub_state    = _m.step(hub_state, current_omega, throttle, params, DT)
                psi_send     = math.atan2(math.sin(hub_state.psi), math.cos(hub_state.psi))
                psi_dot_send = hub_state.psi_dot

        # Safety clamp: cap yaw rate to prevent ArduPilot SIGFPE
        _MAX_PSI_DOT = math.radians(500.0)
        psi_dot_send = max(-_MAX_PSI_DOT, min(_MAX_PSI_DOT, psi_dot_send))

        prev_roll, prev_pitch = roll_send, pitch_send

        # One-shot: record when DYNAMIC phase begins
        if not in_startup and not _dynamics_started:
            _dynamics_started = True
            log.info("DYNAMIC phase started at t=%.1f s  throttle=%.3f  omega=%.1f rad/s",
                     t, throttle, current_omega)
            ev.write("dynamics_start", t_sim=round(t, 1))

        _hb_state["phase"]          = "STARTUP" if in_startup else "DYNAMIC"
        _hb_state["psi_dot_deg_s"]  = round(math.degrees(psi_dot_send), 3)
        _hb_state["throttle"]       = round(throttle, 4)

        gyro_body, accel_body = _body_vectors(
            roll_send, pitch_send, psi_dot_send, roll_dot, pitch_dot)

        return dict(
            pos_ned         = np.array([0.0, 0.0, 0.0]),
            vel_ned         = np.array([0.0, 0.0, 0.0]),
            rpy_rad         = np.array([roll_send, pitch_send, psi_send]),
            accel_body      = np.asarray(accel_body),
            gyro_body       = np.asarray(gyro_body),
            battery_voltage = _BATTERY_V,
        )

    n = 0
    try:
        n = run_lockstep(iface, step_fn, is_stopped, log=log, ev=ev,
                         heartbeat_fields=lambda: dict(_hb_state))
    except KeyboardInterrupt:
        log.info("Interrupted -- shutting down")
    finally:
        ev.write("shutdown", t_sim=round(iface.sim_now(), 1), n_sent=n)
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
        "--profile", default="constant",
        choices=list(PROFILES.keys()),
        help="Axle speed / hub tilt profile (default: constant)",
    )
    parser.add_argument(
        "--tail-channel", type=int, default=_CH_YAW_DEFAULT,
        help="0-based servo channel index for tail motor (default: 3=Ch4, lua: 8=Ch9)",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        help="Logging level (default: INFO)",
    )
    parser.add_argument(
        "--events-log", default=None,
        help="Path for structured JSONL event log",
    )
    parser.add_argument(
        "--startup-yaw-rate", type=float, default=0.0,
        help="Yaw rate [deg/s] sent to SITL during startup hold (default: 0 = stationary)",
    )
    args = parser.parse_args()
    run(args.omega_rotor, args.startup_hold,
        args.profile, args.tail_channel, args.log_level,
        events_log_path=args.events_log,
        startup_yaw_rate=math.radians(args.startup_yaw_rate))
