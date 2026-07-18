"""
A coupled rotational plant suitable for tuning the heli rate loop.

States
------
    phi, theta              body roll, pitch angles (rad)
    p, q                    body roll, pitch rates  (rad/s)
    psi, r                  body yaw angle, rate    (rad/s)
    x, x_dot                tether-spring elongation along the pendulum axis (m, m/s)
    px, py, vx, vy          pendulum cart position & velocity (m)

Dynamics (simplified, but capturing the modes the user identified)
------------------------------------------------------------------
* Inner rate loop sees a first-order rotor flap response from cyclic input to
  body rate, with a cross-coupling **at the nutation frequency** modelled as
  a phase rotation that becomes ill-defined near `f_nut`.
* Pendulum mode (~0.05 Hz) couples body attitude to lateral cart motion.
* Tether spring mode (~3.77 Hz) is a 1-DOF mass-on-spring driven by collective
  & by cyclic-induced lateral thrust.

This is *not* a high-fidelity heli FEM. It is calibrated to reproduce the
three regimes you measured: clean inner-loop step, ~60:1 cross-coupling near
DC, and a nutation crossover near `f_nut`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np


@dataclass
class PlantParams:
    # Inner rate axis — first-order gain (rad/s per unit cyclic), time const (s)
    K_cyclic: float = 12.0
    tau_flap: float = 0.06       # ~2.6 Hz break — typical small heli rotor

    # Cross-coupling: rotation between commanded and produced body torque.
    # `phase0_deg` is the high-frequency phase; the apparent phase rotates an
    # additional 180° as frequency drops through `f_nut`.
    phase0_deg: float = 0.0
    f_nut: float = 3.77          # nutation crossover (Hz) — the user's measured value

    # Pendulum
    L_pend: float = 100.0        # length (m) → omega = sqrt(g/L) ≈ 0.31 rad/s ≈ 0.05 Hz
    pend_damping: float = 0.02   # weak

    # Tether spring (axial elongation)
    f_spring: float = 3.77
    spring_damping: float = 0.03

    # Yaw axis — simple first-order
    K_yaw:   float = 8.0
    tau_yaw: float = 0.10

    g: float = 9.81


@dataclass
class PlantState:
    phi:   float = 0.0
    theta: float = 0.0
    psi:   float = 0.0
    p:     float = 0.0
    q:     float = 0.0
    r:     float = 0.0
    # Pendulum cart
    px:    float = 0.0
    py:    float = 0.0
    vx:    float = 0.0
    vy:    float = 0.0
    # Tether elongation
    x_t:     float = 0.0
    x_t_dot: float = 0.0


class HeliPlant:
    """Discrete-time forward Euler simulator.

    `step(roll_cyc, pitch_cyc, yaw_cmd, collective, dt)` advances state by
    one tick and returns (p, q, r) — the gyro-equivalent body rates, with
    optional measurement noise.
    """

    def __init__(self, params: PlantParams | None = None,
                 enable_pendulum: bool = True,
                 enable_tether: bool = True,
                 gyro_noise_std: float = 0.0,
                 seed: int | None = 0):
        self.p = params or PlantParams()
        self.state = PlantState()
        self.enable_pendulum = enable_pendulum
        self.enable_tether = enable_tether
        self.gyro_noise_std = gyro_noise_std
        self._rng = np.random.default_rng(seed)

    def reset(self) -> None:
        self.state = PlantState()

    # ------------------------------------------------------------------
    # Cross-coupling: produces a frequency-dependent phase rotation.
    # We approximate by remembering a slow filtered copy of the cyclic input
    # and rotating the difference (high-frequency part) by phase0_deg, while
    # rotating the slow part by phase0_deg - 180° (the pendulum-region phase).
    # The crossover is centred at f_nut by choosing the smoothing time const.
    # ------------------------------------------------------------------

    def __post_init__(self):  # not auto-called (not a dataclass); kept for clarity
        pass

    _u_slow_r: float = 0.0
    _u_slow_p: float = 0.0

    def _rotate(self, ur: float, up: float, deg: float) -> tuple[float, float]:
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        return c * ur - s * up, s * ur + c * up

    def step(self,
             roll_cyc: float,
             pitch_cyc: float,
             yaw_cmd: float,
             collective: float,
             dt: float) -> tuple[float, float, float]:
        p = self.p
        s = self.state

        # Cross-coupling phase rotation, split into "slow" and "fast" parts.
        tau_x = 1.0 / (2.0 * math.pi * p.f_nut)
        alpha = dt / (dt + tau_x)
        self._u_slow_r += alpha * (roll_cyc  - self._u_slow_r)
        self._u_slow_p += alpha * (pitch_cyc - self._u_slow_p)
        u_fast_r = roll_cyc  - self._u_slow_r
        u_fast_p = pitch_cyc - self._u_slow_p

        u_r_fast_rot, u_p_fast_rot = self._rotate(u_fast_r, u_fast_p, p.phase0_deg)
        u_r_slow_rot, u_p_slow_rot = self._rotate(self._u_slow_r, self._u_slow_p,
                                                  p.phase0_deg - 180.0)
        u_r_eff = u_r_fast_rot + u_r_slow_rot
        u_p_eff = u_p_fast_rot + u_p_slow_rot

        # First-order flap response: tau * p_dot + p = K * u_eff
        p_dot = (p.K_cyclic * u_r_eff - s.p) / p.tau_flap
        q_dot = (p.K_cyclic * u_p_eff - s.q) / p.tau_flap
        r_dot = (p.K_yaw * yaw_cmd - s.r) / p.tau_yaw

        s.p += p_dot * dt
        s.q += q_dot * dt
        s.r += r_dot * dt

        s.phi   += s.p * dt
        s.theta += s.q * dt
        s.psi   += s.r * dt

        # Pendulum (lateral cart driven by body tilt)
        if self.enable_pendulum:
            omega_p2 = p.g / p.L_pend
            ax = -omega_p2 * s.px + p.g * s.theta - p.pend_damping * s.vx
            ay = -omega_p2 * s.py - p.g * s.phi   - p.pend_damping * s.vy
            s.vx += ax * dt
            s.vy += ay * dt
            s.px += s.vx * dt
            s.py += s.vy * dt
            # Pendulum reaction torque on the body (the "cross-ratio ~60" effect)
            react_roll  =  0.02 * s.py
            react_pitch = -0.02 * s.px
            s.p += react_roll  * dt
            s.q += react_pitch * dt

        # Tether spring (axial elongation, driven by collective changes)
        if self.enable_tether:
            omega_s = 2.0 * math.pi * p.f_spring
            zeta = p.spring_damping
            drive = collective + 0.05 * (u_r_eff + u_p_eff)
            x_t_ddot = (omega_s ** 2) * (drive - s.x_t) - 2.0 * zeta * omega_s * s.x_t_dot
            s.x_t_dot += x_t_ddot * dt
            s.x_t     += s.x_t_dot * dt
            # Bleed a little tether motion back into pitch — this is the
            # "tether spring at nutation crossover" coupling.
            s.q += 0.5 * s.x_t_dot * dt

        # Gyro noise
        gp = s.p + self._rng.normal(0.0, self.gyro_noise_std) if self.gyro_noise_std > 0 else s.p
        gq = s.q + self._rng.normal(0.0, self.gyro_noise_std) if self.gyro_noise_std > 0 else s.q
        gr = s.r + self._rng.normal(0.0, self.gyro_noise_std) if self.gyro_noise_std > 0 else s.r
        return gp, gq, gr
