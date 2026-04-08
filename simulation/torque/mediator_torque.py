#!/usr/bin/env python3
"""
torque/mediator_torque.py — Counter-torque motor stack-test mediator

Bridges ArduPilot SITL (UDP 9002/9003) ↔ hub yaw dynamics for the
counter-torque motor test.

Physical scenario
-----------------
  • Hub sits stationary at NED = [0, 0, 0], roll = 0, pitch = 0.
  • The axle is driven at ``--omega-axle`` rad/s (simulated autorotation).
  • The motor counter-rotates via the 80:44 gear to maintain hub heading.
  • Bearing and swashplate friction is the load the motor works against.
  • ArduPilot sees the yaw rate in the gyro and commands the tail-rotor
    channel (Ch4) to hold the counter-rotation speed via the GB4008 motor.

Sensor data sent to ArduPilot (JSON over UDP 9003 → SITL)
---------------------------------------------------------------------------
  position    [0, 0, 0]  NED [m]  — hub is stationary
  velocity    [0, 0, 0]  NED [m/s]
  attitude    [0, 0, ψ]           — hub yaw angle [rad]
  gyro_body   [0, 0, ψ_dot]      — yaw rate only [rad/s], in body frame
  accel_body  [0, 0, −9.81]      — gravity only [m/s²]; hub is flat
  rpm         [ω_axle → RPM, 0]  — rotor RPM for ArduPilot RSC

ArduPilot → mediator (binary servo packet over UDP 9002)
---------------------------------------------------------------------------
  Ch4 (index 3) → tail/yaw → motor throttle → hub yaw dynamics

  PWM → throttle mapping (H_TAIL_TYPE = 2, DDFP unidirectional):
      throttle = (pwm_us − 1000) / 1000   ∈ [0, 1]
      1000 µs → 0%, 2000 µs → 100%

Usage (Docker / WSL)
--------------------
    python3 mediator_torque.py [--omega-axle FLOAT] [--log-level LEVEL]

Options
-------
    --omega-axle  FLOAT   Axle spin rate [rad/s] (default: 28.0)
    --log-level   LEVEL   Logging level (default: INFO)
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import socket
import struct
import sys
import time
from pathlib import Path

# Model lives in the same directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
import model as _m

# ---------------------------------------------------------------------------
# Protocol constants (must match ArduPilot SITL JSON backend)
# ---------------------------------------------------------------------------

RECV_PORT       = 9002        # UDP — we bind here; SITL sends servo packets to us
DT              = 1.0 / 400.0 # 400 Hz loop target

_SERVO_FMT_16   = "<HHI16H"
_SERVO_SIZE_16  = struct.calcsize(_SERVO_FMT_16)
_SERVO_MAGIC_16 = 18458
_SERVO_FMT_32   = "<HHI32H"
_SERVO_SIZE_32  = struct.calcsize(_SERVO_FMT_32)
_SERVO_MAGIC_32 = 29569

# Default ArduPilot channel for yaw/tail-rotor (0-based index → Ch4).
# When --tail-channel 9 is passed (Lua scripting mode), the mediator reads
# from Ch9 (index 8) which the Lua script writes exclusively.
_CH_YAW_DEFAULT = 3

# Log interval [s]
_LOG_INTERVAL = 1.0


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
        # neutral → trim,  2000 → 1.0
        t = trim + (1.0 - trim) * (pwm_us - 1500.0) / 500.0
    else:
        # 1000 → 0.0,  neutral → trim
        t = trim * (pwm_us - 1000.0) / 500.0
    return max(0.0, min(1.0, t))


def _parse_servos(data: bytes) -> list[float] | None:
    """
    Parse a binary SITL servo packet.

    Returns a list of 16 PWM values in microseconds, or None if the packet
    format is unrecognised.
    """
    if len(data) == _SERVO_SIZE_16:
        fields = struct.unpack(_SERVO_FMT_16, data)
        if fields[0] != _SERVO_MAGIC_16:
            return None
        return list(fields[3:])    # 16 PWM values
    elif len(data) == _SERVO_SIZE_32:
        fields = struct.unpack(_SERVO_FMT_32, data)
        if fields[0] != _SERVO_MAGIC_32:
            return None
        return list(fields[3:19])  # first 16 channels only
    return None


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
    # Primary orbital component (0.10 Hz, 10 s period)
    roll  = math.radians(20.0) * math.sin(2 * math.pi * 0.10 * dt)
    pitch = math.radians(15.0) * math.cos(2 * math.pi * 0.10 * dt)
    # Secondary harmonic (0.23 Hz) adds realistic non-uniform wobble
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


def _make_state_json(
    t: float,
    roll: float,
    pitch: float,
    psi: float,
    psi_dot: float,
    roll_dot: float,
    pitch_dot: float,
    omega_axle: float,
) -> bytes:
    """
    Build the JSON state packet sent back to ArduPilot SITL.

    Sign conventions — ENU model → ArduPilot NED body frame
    --------------------------------------------------------
    Gyro Z positive  = yaw CW from above (NED Z down).
    Yaw angle positive = CW from above.
    Our psi/psi_dot are ENU (positive = CCW), so we negate for NED.

    For a tilted hub (roll φ, pitch θ) gravity projects into body frame:
      g_body_x =  9.81 * sin(θ)
      g_body_y =  9.81 * cos(θ) * sin(φ)
      g_body_z = -9.81 * cos(θ) * cos(φ)
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    g = 9.81

    # ── Correct Euler-rate → body-rate transformation ──────────────────────
    # For ZYX Euler angles (ψ, θ, φ) the body angular velocities are:
    #   p = φ̇  + ψ̇_ENU × sin(θ)
    #   q = θ̇ × cos(φ) − ψ̇_ENU × sin(φ) × cos(θ)
    #   r = −θ̇ × sin(φ) − ψ̇_ENU × cos(φ) × cos(θ)
    # (substituting ψ̇_NED = −ψ̇_ENU into the standard ZYX formula)
    #
    # At 20° roll, pitch_dot=±9°/s: r ≈ ±3.1°/s even with zero yaw rate.
    # This cross-coupling is real — without it the motor ignores the wobble.
    gyro_x = roll_dot + psi_dot * sp
    gyro_y = pitch_dot * cr - psi_dot * sr * cp
    gyro_z = -pitch_dot * sr - psi_dot * cr * cp

    msg = {
        "timestamp": float(t),
        "imu": {
            "gyro":       [float(gyro_x), float(gyro_y), float(gyro_z)],
            "accel_body": [float(g * sp),
                           float(g * cp * sr),
                           float(-g * cp * cr)],
        },
        "position":   [0.0, 0.0, 0.0],
        "attitude":   [float(roll), float(pitch), float(-psi)],
        "velocity":   [0.0, 0.0, 0.0],
        # Send motor RPM via battery.voltage field — the rpm field approach
        # failed because state.rpm[] gets overwritten by SITL motor simulation.
        # Battery voltage is reliable: battery:voltage(0) in Lua reads this.
        # The Lua script interprets voltage > 50 as motor RPM (not a real voltage).
        # In non-lua-mode tests, voltage=12.6 < 50 so Lua returns early harmlessly.
        "battery":    {"voltage": float(omega_axle * (80.0/44.0) * 60.0 / (2.0 * math.pi))},
    }
    return (json.dumps(msg) + "\n").encode("utf-8")


def run(
    omega_axle: float,
    startup_hold_s: float = 5.0,
    trim_throttle: float | None = None,
    profile: str = "constant",
    tail_channel: int = _CH_YAW_DEFAULT,
    lua_mode: bool = False,
    log_level: str = "INFO",
) -> None:
    """
    Main mediator loop.

    Startup hold phase (0 … startup_hold_s)
    ----------------------------------------
    Hub is locked stationary at ψ=0.  A slow constant yaw drift of 5°/s is
    sent to SITL to give ArduPilot's EKF enough gyro and compass data to
    initialise.  Bearing drag and motor dynamics are NOT active.

    This mirrors the kinematic startup phase used by the full RAWES stack
    tests: send physically consistent (but low-amplitude) motion so the EKF
    can align its tilt, yaw, and gyro bias estimates before real dynamics
    start.

    Dynamic phase (startup_hold_s … ∞)
    ------------------------------------
    Real hub yaw dynamics: the motor counter-rotates via the 80:44 gear to
    maintain hub heading; ArduPilot's tail-rotor PID (Ch4) controls the
    GB4008 motor speed.

    Parameters
    ----------
    omega_axle     : axle spin rate [rad/s] (nominal autorotation speed)
    startup_hold_s : duration of EKF-initialisation spin phase [s]
    log_level      : Python logging level string
    """
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    log = logging.getLogger("mediator_torque")

    params  = _m.HubParams()
    state   = _m.HubState()
    t       = 0.0

    # Slow yaw spin rate during startup hold [rad/s].
    # 5°/s drives compass + gyro updates for EKF alignment without confusing it.
    _STARTUP_YAW_RATE = math.radians(5.0)

    # Trim throttle: bias so neutral PWM (1500 µs) = equilibrium torque.
    # Defaults to the model's equilibrium throttle for the given axle speed.
    if trim_throttle is None:
        trim_throttle = _m.equilibrium_throttle(omega_axle, params)

    eq_throttle = _m.equilibrium_throttle(omega_axle, params)
    log.info(
        "Counter-torque mediator starting  "
        "omega_axle=%.2f rad/s (%.0f RPM)  "
        "k_bearing=%.4f Nm*s/rad  "
        "gear=%.3f  tau_stall=%.2f Nm  eq_throttle=%.4f  "
        "trim_throttle=%.4f  startup_hold=%.0f s",
        omega_axle,
        omega_axle * 60.0 / (2.0 * math.pi),
        params.k_bearing,
        params.gear_ratio,
        params.tau_stall,
        eq_throttle,
        trim_throttle,
        startup_hold_s,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", RECV_PORT))
    sock.settimeout(DT)

    log.info("Bound to UDP port %d", RECV_PORT)

    omega_fn, tilt_fn = PROFILES.get(profile, PROFILES["constant"])
    ch_yaw = tail_channel   # which servo channel carries the motor command
    log.info("Profile: %s  tail_channel: %d  lua_mode: %s", profile, ch_yaw, lua_mode)

    sitl_addr  = None
    throttle   = 0.0
    n_sent     = 0
    last_log   = 0.0
    prev_roll  = 0.0
    prev_pitch = 0.0

    # Clamp trim_throttle type for _pwm_to_throttle
    trim_fixed = float(trim_throttle) if trim_throttle is not None else float(eq_throttle)

    # Store last raw Ch4 PWM so we can re-apply the correct (possibly updated)
    # trim after computing current_omega for adaptive profiles.
    last_pwm_ch4 = 1500.0   # neutral default
    throttle     = _pwm_to_throttle(last_pwm_ch4, trim_fixed)

    try:
        while True:
            # ── Receive servo packet from SITL ─────────────────────────────
            try:
                data, addr = sock.recvfrom(4096)
                sitl_addr  = addr
                pwm_list   = _parse_servos(data)
                if pwm_list is not None:
                    last_pwm_ch4 = pwm_list[ch_yaw]
            except socket.timeout:
                pass
            except OSError as exc:
                log.warning("Recv error: %s", exc)

            in_startup   = (t < startup_hold_s)
            dynamics_t   = max(0.0, t - startup_hold_s)

            if in_startup:
                psi_send     = (_STARTUP_YAW_RATE * t) % (2.0 * math.pi)
                psi_dot_send = _STARTUP_YAW_RATE
                roll_send    = 0.0
                pitch_send   = 0.0
                roll_dot     = 0.0
                pitch_dot    = 0.0
                current_omega = omega_axle
                # Fixed trim during startup
                throttle = _pwm_to_throttle(last_pwm_ch4, trim_fixed)
            else:
                # Profile-driven axle speed (clamped positive)
                current_omega = max(1.0, omega_fn(dynamics_t, omega_axle))

                if lua_mode:
                    # Lua script is the intended controller via the tail channel.
                    # Safety fallback: if Ch9 is at default/zero (Lua hasn't written
                    # yet or hasn't started), use adaptive trim to keep the hub stable.
                    # This prevents extreme gyro values (hub spinning freely) from
                    # triggering ArduPilot's SIGFPE when scripting is enabled.
                    raw_throttle = max(0.0, min(1.0, (last_pwm_ch4 - 1000.0) / 1000.0))
                    if raw_throttle < 0.05:
                        # Ch9 at default → Lua not yet active, keep hub stable
                        throttle = _m.equilibrium_throttle(current_omega, params)
                    else:
                        # Lua is writing — use its output directly
                        throttle = raw_throttle
                else:
                    # Adaptive trim: mediator computes equilibrium feedforward.
                    current_trim = _m.equilibrium_throttle(current_omega, params)
                    throttle     = _pwm_to_throttle(last_pwm_ch4, current_trim)

                # Profile-driven hub tilt + rates
                roll_send, pitch_send = tilt_fn(dynamics_t)
                roll_dot  = (roll_send  - prev_roll)  / DT
                pitch_dot = (pitch_send - prev_pitch) / DT

                # Initialise psi continuity on first dynamic step
                if state.psi == 0.0 and state.psi_dot == 0.0:
                    state.psi = (_STARTUP_YAW_RATE * startup_hold_s) % (2.0 * math.pi)
                state = _m.step(state, current_omega, throttle, params, DT)
                psi_send     = math.atan2(math.sin(state.psi), math.cos(state.psi))
                psi_dot_send = state.psi_dot

            prev_roll, prev_pitch = roll_send, pitch_send
            t += DT

            # ── Send state back to SITL ────────────────────────────────────
            if sitl_addr is not None:
                payload = _make_state_json(
                    t, roll_send, pitch_send, psi_send, psi_dot_send,
                    roll_dot, pitch_dot, current_omega,
                )
                try:
                    sock.sendto(payload, sitl_addr)
                    n_sent += 1
                except OSError as exc:
                    log.warning("Send error: %s", exc)

            # ── Periodic log ───────────────────────────────────────────────
            if t - last_log >= _LOG_INTERVAL:
                last_log = t
                phase = "STARTUP" if in_startup else "DYNAMIC"
                log.info(
                    "t=%6.1f s [%s]  psi=%+7.2f deg  psi_dot=%+7.3f deg/s  "
                    "throttle=%.3f  omega=%.1f rad/s  roll=%+.1f deg  pitch=%+.1f deg  n=%d",
                    t, phase,
                    math.degrees(psi_send), math.degrees(psi_dot_send),
                    throttle, current_omega if not in_startup else omega_axle,
                    math.degrees(roll_send), math.degrees(pitch_send),
                    n_sent,
                )
                if not in_startup and t - startup_hold_s < 2.0:
                    log.info("*** STARTUP HOLD COMPLETE — hub dynamics now active ***")

    except KeyboardInterrupt:
        log.info("Interrupted — shutting down  t=%.1f s  n_sent=%d", t, n_sent)
    finally:
        sock.close()
        log.info("Socket closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="RAWES counter-torque motor mediator (stationary hub, yaw-only)"
    )
    parser.add_argument(
        "--omega-axle", type=float, default=_m.OMEGA_AXLE_NOMINAL,
        help=f"Axle spin rate [rad/s] (default: {_m.OMEGA_AXLE_NOMINAL:.1f})",
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
    args = parser.parse_args()
    run(args.omega_axle, args.startup_hold, args.trim_throttle,
        args.profile, args.tail_channel, args.lua_mode, args.log_level)
