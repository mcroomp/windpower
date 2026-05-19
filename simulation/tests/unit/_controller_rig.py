"""Controller-tuning rig.

Tools for system identification and gain calibration of the cyclic
attitude loop.  Not a regression test — these are diagnostic helpers
used to tune controller gains and the swashplate phase angle for a
given rotor operating point.

What it provides
----------------
* ``probe_open_loop_plant``  — sweep a sinusoidal cyclic input through
  one channel (tilt_lat or tilt_lon) and measure the body-rate
  response on BOTH body axes.  Returns a Bode dataset:
  amplitude + phase per axis per frequency.  Run twice (once per
  channel) to characterise the full 2x2 cyclic→body-rate plant.

* ``identify_decoupling_phase`` — from a single-channel Bode dataset at
  one frequency, compute the angle that rotates the cyclic input so the
  observed body-rate vector aligns with the corresponding body axis.
  This is the swashplate_phase_deg value at that operating point.

* ``probe_step_response`` — apply a step rate setpoint through
  ``HeliCyclicController``, return overshoot, settling time, steady-state
  error.  Used to score candidate gain values.

* ``stability_margins`` — from a frequency sweep of the closed loop,
  compute approximate gain and phase margins (in dB and degrees).

Typical usage
-------------
::

    from tests.unit._controller_rig import (
        probe_open_loop_plant, identify_decoupling_phase,
        probe_step_response,
    )

    # Plant identification at the design operating point
    bode_lat = probe_open_loop_plant(
        rotor, R_hub, omega_spin=28.0,
        input_channel="tilt_lat",
        frequencies_hz=[0.5, 1.0, 2.0, 5.0],
        amplitude=0.05,
    )
    # Decoupling phase at 1 Hz
    phase_lat = identify_decoupling_phase(bode_lat, frequency_hz=1.0)
    # Step response with candidate gains
    metrics = probe_step_response(rotor, R_hub, omega_spin=28.0,
                                  kp_inner=0.1, kd_inner=0.05,
                                  setpoint=1.0)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from aero import RotorInputs, OyeBEMModel
from controller import HeliCyclicController
from dynamics import RigidBodyDynamics


_G_M_S2 = 9.81


# ---------------------------------------------------------------------------
# Dataclasses for results
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class BodeSample:
    """Single (frequency, gain, phase) triple per output axis."""
    frequency_hz:    float
    gain_x:          float       # |ω_x / u| (rad/s per unit cyclic)
    phase_x_deg:     float       # phase lag of ω_x relative to input
    gain_y:          float       # |ω_y / u|
    phase_y_deg:     float
    cross_ratio:     float       # gain_off / gain_on; near 0 = clean axes


@dataclass(frozen=True)
class StepMetrics:
    """Time-response metrics from a step setpoint."""
    overshoot_pct:   float       # max(ω) / setpoint - 1, in %
    settling_time_s: float       # time to enter ±5% band of setpoint
    rise_time_s:     float       # time from 10% to 90% of setpoint
    steady_state_error: float    # setpoint - mean(ω) over last 25% of run
    diverged:        bool        # True if |ω| exceeded 50 × setpoint anywhere
    history: dict                # full {t, omega_x, omega_y, tilt_lat, tilt_lon}


# ---------------------------------------------------------------------------
# Plant identification (open-loop frequency response)
# ---------------------------------------------------------------------------

def _settle_inflow(aero, omega_spin: float, R_hub: np.ndarray,
                   n_steps: int = 200, dt: float = 0.02) -> object:
    """Pre-settle the dynamic-inflow states so the Bode probe starts from
    a quasi-static operating point and the first cycles aren't transient."""
    state = aero.initial_rotor_state()
    state.omega_rad_s = float(omega_spin)
    inputs = RotorInputs(
        collective_rad=-0.05, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R_hub, v_hub_world=np.zeros(3), wind_world=np.zeros(3),
        t=10.0,
    )
    for _ in range(n_steps):
        _, deriv = aero.compute_forces(inputs, state)
        state = state.from_array(state.to_array() + dt * deriv.to_array())
        state.omega_rad_s = float(omega_spin)
    return state


def probe_open_loop_plant(
    rotor,
    R_hub:          np.ndarray,
    *,
    omega_spin:     float,
    input_channel:  str,                              # "tilt_lat" | "tilt_lon"
    frequencies_hz: list[float],
    amplitude:      float = 0.05,
    cycles:         int   = 5,
    dt:             float = 0.0025,
    collective_rad: float = -0.05,
    mass_kg:        float = 5.0,
    I_body:         tuple = (5.0, 5.0, 10.0),
    I_spin:         float = 4.0,
) -> list[BodeSample]:
    """Drive a single cyclic channel with a sinusoid and measure body-rate
    response on both axes.

    For each frequency, the input is ``amplitude · sin(2π·f·t)``.  The
    body integrates from rest under cancelled gravity (so we observe
    pure attitude dynamics).  We run for ``cycles`` periods of the input
    so the response reaches steady state, then fit a sinusoid to the
    last 2 cycles to extract gain and phase.

    Returns a list of :class:`BodeSample` (one per frequency).
    """
    if input_channel not in ("tilt_lat", "tilt_lon"):
        raise ValueError(f"input_channel must be 'tilt_lat' or 'tilt_lon', got {input_channel!r}")

    aero = OyeBEMModel(defn=rotor)
    F_grav_cancel = np.array([0.0, 0.0, -mass_kg * _G_M_S2])
    samples: list[BodeSample] = []

    for freq in frequencies_hz:
        period_s   = 1.0 / freq
        n_steps    = max(int(round(cycles * period_s / dt)), 100)
        omega_w_t  = 2.0 * math.pi * freq

        state = _settle_inflow(aero, omega_spin, R_hub)
        dyn = RigidBodyDynamics(
            mass=mass_kg, I_body=list(I_body), I_spin=I_spin,
            pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
            R0=R_hub.copy(), omega0=[0.0, 0.0, 0.0],
        )

        u_log     = np.empty(n_steps)
        omega_x_b = np.empty(n_steps)
        omega_y_b = np.empty(n_steps)
        for i in range(n_steps):
            t = i * dt
            u = amplitude * math.sin(omega_w_t * t)
            tlon, tlat = (u, 0.0) if input_channel == "tilt_lon" else (0.0, u)

            s        = dyn.state
            R        = s["R"]
            omega_b  = R.T @ s["omega"]
            omega_x_b[i] = omega_b[0]
            omega_y_b[i] = omega_b[1]
            u_log[i]     = u

            inputs = RotorInputs(
                collective_rad=collective_rad, tilt_lon=tlon, tilt_lat=tlat,
                R_hub=R, v_hub_world=s["vel"], wind_world=np.zeros(3),
                t=10.0,
            )
            result, deriv = aero.compute_forces(inputs, state)
            state = state.from_array(state.to_array() + dt * deriv.to_array())
            state.omega_rad_s = float(omega_spin)
            dyn.step(F_grav_cancel + result.F_world, result.M_orbital, dt,
                     omega_spin=omega_spin)

        # Fit a sinusoid to the last 2 cycles: ω(t) ≈ A·sin(ω·t) + B·cos(ω·t).
        # Then gain = sqrt(A² + B²) / amplitude, phase = atan2(-B, A).
        i_start = max(0, n_steps - int(round(2.0 * period_s / dt)))
        t_window = np.arange(i_start, n_steps) * dt
        s_basis  = np.sin(omega_w_t * t_window)
        c_basis  = np.cos(omega_w_t * t_window)
        norm     = float(np.sum(s_basis * s_basis))   # = sum(c²) for full cycles
        Ax = float(np.sum(omega_x_b[i_start:] * s_basis)) / norm
        Bx = float(np.sum(omega_x_b[i_start:] * c_basis)) / norm
        Ay = float(np.sum(omega_y_b[i_start:] * s_basis)) / norm
        By = float(np.sum(omega_y_b[i_start:] * c_basis)) / norm
        gain_x  = math.hypot(Ax, Bx) / amplitude
        gain_y  = math.hypot(Ay, By) / amplitude
        # Phase lag of ω relative to input u = amp·sin(ω·t): positive lag
        # means the response peaks AFTER the input.  atan2(-Bx, Ax)
        # converts from (A·sin + B·cos) form to amplitude·sin(ω·t − φ).
        phase_x = math.degrees(math.atan2(-Bx, Ax)) if (abs(Ax) + abs(Bx)) > 1e-12 else 0.0
        phase_y = math.degrees(math.atan2(-By, Ay)) if (abs(Ay) + abs(By)) > 1e-12 else 0.0

        on_axis_gain  = gain_x if input_channel == "tilt_lat" else gain_y
        off_axis_gain = gain_y if input_channel == "tilt_lat" else gain_x
        cross_ratio   = (off_axis_gain / on_axis_gain) if on_axis_gain > 1e-9 else float("inf")

        samples.append(BodeSample(
            frequency_hz=freq,
            gain_x=gain_x, phase_x_deg=phase_x,
            gain_y=gain_y, phase_y_deg=phase_y,
            cross_ratio=cross_ratio,
        ))

    return samples


def identify_decoupling_phase(
    samples:      list[BodeSample],
    *,
    frequency_hz: float,
    input_channel: str,            # which channel the sweep used
    tol_hz:       float = 0.01,
) -> float:
    """Find the swashplate_phase_deg that aligns the response with the
    intended body axis.

    From a single-channel sweep at ``frequency_hz``, computes the angle
    in the body-XY plane of the observed body-rate response, and returns
    the phase rotation needed to bring that angle to:

      * 0°    (along +x_b) for ``input_channel="tilt_lat"`` (roll)
      * -90°  (along -y_b) for ``input_channel="tilt_lon"`` (pitch)

    The returned value is the swashplate_phase_deg that should be set in
    the rotor definition.  Run the sweep on BOTH channels and check the
    two estimates agree; if not, there's cross-coupling beyond a pure
    rotation and a decoupling matrix (not just a phase) is needed.
    """
    target_deg = 0.0 if input_channel == "tilt_lat" else -90.0
    for s in samples:
        if abs(s.frequency_hz - frequency_hz) <= tol_hz:
            response_angle = math.degrees(math.atan2(s.gain_y, s.gain_x))
            # Note: gain_x and gain_y are *magnitudes* — we want the SIGNED
            # response.  Recover sign from the phase lag at the input
            # frequency.  Cleaner: use the fitted (A, B) coefficients
            # directly.  Here we approximate by treating in-phase response
            # as positive (phase ≈ 0) and 180° out-of-phase as negative.
            sign_x = -1.0 if abs(s.phase_x_deg) > 90.0 else 1.0
            sign_y = -1.0 if abs(s.phase_y_deg) > 90.0 else 1.0
            response_angle = math.degrees(math.atan2(
                sign_y * s.gain_y, sign_x * s.gain_x,
            ))
            return float((target_deg - response_angle) % 360.0)
    raise ValueError(f"No sample at frequency {frequency_hz} Hz (tol {tol_hz})")


# ---------------------------------------------------------------------------
# Closed-loop step response
# ---------------------------------------------------------------------------

def probe_step_response(
    rotor,
    R_hub:          np.ndarray,
    *,
    omega_spin:     float,
    kp_inner:       float = 2.0/3.0,
    ki_inner:       float = 0.0,
    kd_inner:       float = 0.0,
    imax_inner:     float = 0.0,
    setpoint:       float = 1.0,
    channel:        str   = "roll",                  # "roll" | "pitch"
    duration_s:     float = 2.0,
    dt:             float = 0.0025,
    collective_rad: float = -0.05,
    mass_kg:        float = 5.0,
    I_body:         tuple = (5.0, 5.0, 10.0),
    I_spin:         float = 4.0,
) -> StepMetrics:
    """Apply a step rate setpoint through ``HeliCyclicController`` and
    return time-response metrics.  Use as the cost function for an
    automated gain search."""
    aero  = OyeBEMModel(defn=rotor)
    state = _settle_inflow(aero, omega_spin, R_hub)
    acro  = HeliCyclicController(
        rotor, col_min_rad=-0.28, col_max_rad=0.10,
        P=kp_inner, I=ki_inner, D=kd_inner, IMAX=imax_inner,
    )
    dyn = RigidBodyDynamics(
        mass=mass_kg, I_body=list(I_body), I_spin=I_spin,
        pos0=[0.0, 0.0, -50.0], vel0=[0.0, 0.0, 0.0],
        R0=R_hub.copy(), omega0=[0.0, 0.0, 0.0],
    )
    F_grav_cancel = np.array([0.0, 0.0, -mass_kg * _G_M_S2])

    n_steps = int(round(duration_s / dt))
    t_log       = np.arange(n_steps, dtype=float) * dt
    omega_x_log = np.empty(n_steps)
    omega_y_log = np.empty(n_steps)
    tlon_log    = np.empty(n_steps)
    tlat_log    = np.empty(n_steps)

    diverged = False
    for i in range(n_steps):
        s        = dyn.state
        R        = s["R"]
        omega_b  = R.T @ s["omega"]
        if abs(omega_b[0]) > 50.0 * abs(setpoint) + 5.0:
            diverged = True
            omega_x_log[i:] = omega_b[0]
            omega_y_log[i:] = omega_b[1]
            tlon_log[i:] = 0.0
            tlat_log[i:] = 0.0
            break
        omega_x_log[i] = omega_b[0]
        omega_y_log[i] = omega_b[1]
        rate_roll  = setpoint if channel == "roll"  else 0.0
        rate_pitch = setpoint if channel == "pitch" else 0.0
        tlon, tlat, _ = acro.step(
            collective_cmd=collective_rad,
            rate_roll_sp=rate_roll, rate_pitch_sp=rate_pitch,
            omega_body=omega_b, dt=dt,
        )
        tlon_log[i] = tlon
        tlat_log[i] = tlat
        inputs = RotorInputs(
            collective_rad=collective_rad, tilt_lon=tlon, tilt_lat=tlat,
            R_hub=R, v_hub_world=s["vel"], wind_world=np.zeros(3), t=10.0,
        )
        result, deriv = aero.compute_forces(inputs, state)
        state = state.from_array(state.to_array() + dt * deriv.to_array())
        state.omega_rad_s = float(omega_spin)
        dyn.step(F_grav_cancel + result.F_world, result.M_orbital, dt,
                 omega_spin=omega_spin)

    # On-axis response: x for roll, y for pitch
    response = omega_x_log if channel == "roll" else omega_y_log
    sp = float(setpoint)

    if diverged or not np.all(np.isfinite(response)):
        return StepMetrics(
            overshoot_pct=float("inf"), settling_time_s=duration_s,
            rise_time_s=duration_s, steady_state_error=float("nan"),
            diverged=True,
            history=dict(t=t_log, omega_x=omega_x_log, omega_y=omega_y_log,
                         tilt_lat=tlat_log, tilt_lon=tlon_log),
        )

    peak = float(np.max(np.abs(response)))
    overshoot = (peak / abs(sp) - 1.0) * 100.0 if abs(sp) > 1e-9 else 0.0
    # Settling time: last time the response was outside ±5% of setpoint
    tol = 0.05 * abs(sp)
    outside = np.where(np.abs(response - sp) > tol)[0]
    settling_time = (outside[-1] + 1) * dt if len(outside) else 0.0
    # Rise time: 10% → 90% of setpoint (one-sided, assumes positive sp)
    target_lo = 0.10 * sp
    target_hi = 0.90 * sp
    try:
        i_lo = int(np.argmax(response >= target_lo)) if sp > 0 else int(np.argmax(response <= target_lo))
        i_hi = int(np.argmax(response >= target_hi)) if sp > 0 else int(np.argmax(response <= target_hi))
        rise_time = (i_hi - i_lo) * dt if i_hi > i_lo else duration_s
    except Exception:
        rise_time = duration_s
    # Steady-state error over last 25% of the run
    i_ss = int(0.75 * n_steps)
    ss_error = sp - float(np.mean(response[i_ss:]))

    return StepMetrics(
        overshoot_pct=overshoot, settling_time_s=settling_time,
        rise_time_s=rise_time, steady_state_error=ss_error,
        diverged=False,
        history=dict(t=t_log, omega_x=omega_x_log, omega_y=omega_y_log,
                     tilt_lat=tlat_log, tilt_lon=tlon_log),
    )


# ---------------------------------------------------------------------------
# Stability margins (approximate, from a closed-loop frequency sweep)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class StabilityMargins:
    """Approximate gain margin and phase margin extracted from a Bode sweep
    of the closed-loop system (input = setpoint, output = response)."""
    gain_margin_db:    float    # ∞ if -180° phase crossing never reached
    phase_margin_deg:  float    # 180 + phase at unity-gain (0 dB) frequency
    crossover_hz:      float    # frequency at which |loop gain| = 1
    phase_180_hz:      float    # frequency at which phase = -180°


def stability_margins(loop_bode: list[BodeSample]) -> StabilityMargins:
    """Estimate gain/phase margins from a closed-loop or open-loop Bode sweep.

    Pass the loop-transfer-function Bode (gain_x is treated as |L(jω)|,
    phase_x_deg as ∠L(jω)).  Linear interpolation in log-frequency / dB.
    """
    if not loop_bode:
        return StabilityMargins(float("inf"), float("inf"), float("nan"), float("nan"))

    freqs    = np.array([s.frequency_hz   for s in loop_bode])
    gains_db = 20.0 * np.log10(np.array([max(s.gain_x, 1e-12) for s in loop_bode]))
    phases   = np.array([s.phase_x_deg    for s in loop_bode])

    # Crossover frequency (gain = 0 dB)
    crossover = float("nan")
    for i in range(len(freqs) - 1):
        if (gains_db[i] - 0.0) * (gains_db[i + 1] - 0.0) < 0:
            crossover = float(np.interp(0.0, gains_db[i:i+2], freqs[i:i+2]))
            break
    phase_margin = (180.0 + float(np.interp(crossover, freqs, phases))) if np.isfinite(crossover) else float("inf")

    # Phase = -180° frequency
    phase_180_freq = float("nan")
    for i in range(len(freqs) - 1):
        if (phases[i] + 180.0) * (phases[i + 1] + 180.0) < 0:
            phase_180_freq = float(np.interp(-180.0, phases[i:i+2], freqs[i:i+2]))
            break
    gain_margin_db = (-float(np.interp(phase_180_freq, freqs, gains_db))) if np.isfinite(phase_180_freq) else float("inf")

    return StabilityMargins(
        gain_margin_db=gain_margin_db,
        phase_margin_deg=phase_margin,
        crossover_hz=crossover,
        phase_180_hz=phase_180_freq,
    )
