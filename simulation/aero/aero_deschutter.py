"""
aero_deschutter.py — Per-blade strip-theory BEM aerodynamic model (DeSchutterAero)

NumPy-vectorized: all (azimuth × blade × radial) strips evaluated as batched
array ops — no Python loops over strips.  NumPy ufuncs (cross, einsum, etc.)
use SIMD (AVX2/AVX-512) automatically via OpenBLAS/MKL.

Per-blade model:
  - N_BLADES blades sampled at N_AZ equally-spaced azimuths (revolution averaging)
  - N_RADIAL radial strips per blade from R_ROOT to R_TIP
  - Each strip sees its own rotational velocity omega × r_i
  - AoA per strip: α = −ua_normal / ua_chord  (Kutta-Joukowski sign convention)
  - Lift direction: Kutta-Joukowski cross product (ua × e_span)
  - Cyclic moments emerge from per-blade force asymmetry — no K_cyc needed

Precomputed in __init__ (layout-invariant quantities):
  _phi_offsets  (N_AB,)    — azimuth + blade phase angles, without spin_angle
  _r_stations   (N_RADIAL,) — strip centre radii
  N_AB = N_AZ × N_BLADES

Runtime array dimensions: (N_AB, N_RADIAL) or (N_AB, N_RADIAL, 3).

Spin dynamics:
  last_Q_spin uses an empirical model:  Q = k_drive × v_inplane − k_drag × ω²
  Equilibrium: ω_eq = sqrt(k_drive × v_inplane / k_drag)
"""

import math
import logging
import sys as _sys
import os as _os
import numpy as np

_SIM_DIR = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _SIM_DIR not in _sys.path:
    _sys.path.insert(0, _SIM_DIR)

from aero import AeroResult

log = logging.getLogger(__name__)


class DeSchutterAero:
    """
    Per-blade strip-theory aerodynamic model — De Schutter et al. 2018.
    NumPy-vectorized: all strip evaluations run as batched array ops.

    Spin: last_Q_spin = k_drive_spin * v_inplane - k_drag_spin * omega²
    Use:  omega_spin += aero.last_Q_spin / I_spin * dt
    """

    N_AZ          = 8
    CP_FRAC       = 2.0 / 3.0
    PITCH_MAX_DEG = 15.0

    def __init__(self, rotor, ramp_time: float = 5.0, n_radial: int = 8, **overrides):
        p = {**rotor.aero_kwargs(), **overrides}
        self.N_BLADES   = int(p["n_blades"])
        self.R_ROOT     = float(p["r_root"])
        self.R_TIP      = float(p["r_tip"])
        self.CHORD      = float(p["chord"])
        self.RHO        = float(p["rho"])
        self.OSWALD     = float(p["oswald_eff"])
        self.CD0        = float(p["CD0"])
        self.CL0        = float(p["CL0"])
        self.CL_ALPHA   = float(p["CL_alpha"])
        self.AOA_LIMIT  = float(p["aoa_limit"])
        self.ramp_time  = float(ramp_time)
        self.k_drive_spin = float(p["k_drive_spin"])
        self.k_drag_spin  = float(p["k_drag_spin"])

        span            = self.R_TIP - self.R_ROOT
        self.S_blade    = span * self.CHORD
        self.AR         = float(p.get("aspect_ratio") or span ** 2 / self.S_blade)
        self.R_CP       = self.R_ROOT + self.CP_FRAC * span
        self.pitch_gain = float(p.get("pitch_gain_rad", math.radians(self.PITCH_MAX_DEG)))
        self.disk_area  = math.pi * (self.R_TIP ** 2 - self.R_ROOT ** 2)

        self.N_RADIAL    = int(n_radial)
        self._dr         = span / self.N_RADIAL
        # (N_RADIAL,) — strip centre radii; C-contiguous for SIMD friendliness
        self._r_stations = np.ascontiguousarray(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )
        self._S_elem = self.CHORD * self._dr

        # ── Precompute phase offsets ──────────────────────────────────────────
        # All (az, blade) phase combinations, shape (N_AB,)
        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = np.ascontiguousarray(
            (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        )  # (N_AB,)
        self.N_AB = self.N_AZ * self.N_BLADES

        # Diagnostics (readable after each compute_forces call)
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_collective_rad = 0.0
        self.last_tilt_lon       = 0.0
        self.last_tilt_lat       = 0.0
        self.last_Q_spin         = 0.0
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_Q_drive        = 0.0
        self.last_Q_drag         = 0.0
        self.last_H_force        = 0.0

    @classmethod
    def from_definition(cls, defn, **overrides) -> "DeSchutterAero":
        """Alias for DeSchutterAero(rotor, **overrides)."""
        return cls(defn, **overrides)

    def _ramp_factor(self, t: float) -> float:
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _induced_velocity(self, T_guess: float, v_axial: float = 0.0) -> float:
        T_abs = max(abs(T_guess), 0.01)
        v_ax  = abs(v_axial)
        disc  = v_ax ** 2 + 2.0 * T_abs / (self.RHO * self.disk_area)
        return max(0.0, (-v_ax + math.sqrt(disc)) / 2.0)

    def compute_forces(
        self,
        collective_rad: float,
        tilt_lon:       float,
        tilt_lat:       float,
        R_hub:          np.ndarray,
        v_hub_world:    np.ndarray,
        omega_rotor:    float,
        wind_world:     np.ndarray,
        t:              float,
        spin_angle:     float = 0.0,
    ) -> AeroResult:
        """
        Compute aerodynamic wrench [Fx,Fy,Fz,Mx,My,Mz] in world NED [N, N·m].

        Vectorized over all (N_AB × N_RADIAL) strips — no Python strip loop.

        After the call:
          last_Q_spin   — empirical spin torque; use for spin ODE
          last_M_spin   — spin-axis moment vector
          last_v_inplane — in-plane wind speed [m/s]
        """
        ramp = self._ramp_factor(t)

        disk_normal = R_hub[:, 2]                                    # (3,)
        spin_sign   = float(np.sign(omega_rotor)) if omega_rotor != 0.0 else 1.0
        omega_abs   = abs(float(omega_rotor))

        tilt_lon_rad = tilt_lon * self.pitch_gain
        tilt_lat_rad = tilt_lat * self.pitch_gain

        v_rel_world   = wind_world - v_hub_world                     # (3,)
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # ── Bootstrap induction (3 passes at R_CP) ───────────────────────────
        v_i = 0.0
        for _ in range(3):
            v_tan = omega_abs * self.R_CP
            v_loc = math.sqrt(v_tan ** 2 + (v_axial + v_i) ** 2)
            if v_loc > 0.5:
                inflow = math.atan2(v_axial + v_i, v_tan)
                aoa0   = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, inflow + collective_rad))
                CL0_   = self.CL0 + self.CL_ALPHA * aoa0
                CD0_   = self.CD0 + CL0_ ** 2 / (math.pi * self.AR * self.OSWALD)
                q0     = 0.5 * self.RHO * v_loc ** 2
                T_est  = max(0.0, self.N_BLADES * q0 * self.S_blade
                             * (CL0_ * math.cos(inflow) - CD0_ * math.sin(inflow)))
                v_i    = self._induced_velocity(T_est, v_axial)

        v_i_vec = v_i * disk_normal                                  # (3,)

        # ── Azimuth + blade phase  ────────────────────────────────────────────
        # phi: (N_AB,)  —  spin_angle added at runtime; offsets precomputed
        phi = spin_angle + self._phi_offsets
        ca  = np.cos(phi)                                            # (N_AB,)
        sa  = np.sin(phi)                                            # (N_AB,)

        # ── Hub-frame unit vectors → world frame ──────────────────────────────
        # e_span_body: (N_AB, 3)  [ca, sa, 0] for each (az, blade)
        e_span_body = np.stack([ca, sa, np.zeros(self.N_AB)], axis=1)

        # Blade pitch: collective + swashplate cyclic tilt: (N_AB,)
        p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
        cp_k = np.cos(p_k)
        sp_k = np.sin(p_k)

        # Chord and normal unit vectors in hub body frame: (N_AB, 3)
        e_chord_body  = np.stack([-sa * cp_k,  ca * cp_k,  sp_k], axis=1)
        e_normal_body = np.stack([ sa * sp_k, -ca * sp_k,  cp_k], axis=1)

        # Rotate all to world frame in one batched matmul: (N_AB, 3)
        # x_world = R_hub @ x_body  ↔  x_world = x_body @ R_hub.T
        e_span_world   = e_span_body   @ R_hub.T
        e_chord_world  = e_chord_body  @ R_hub.T
        e_normal_world = e_normal_body @ R_hub.T

        # Tangential direction for torque decomposition: (N_AB, 3)
        e_tang = np.cross(disk_normal[None, :], e_span_world)

        # ── Strip positions in world frame: (N_AB, N_RADIAL, 3) ──────────────
        # r_cp_world[i,j] = r_j · e_span_world[i]  (linearity of rotation)
        r_cp_world = e_span_world[:, None, :] * self._r_stations[None, :, None]

        # ── Apparent wind at each strip: (N_AB, N_RADIAL, 3) ─────────────────
        # v_rot = ω × r_cp = ω · (disk_normal × r_cp) = ω · r · e_tang
        v_rot  = omega_rotor * e_tang[:, None, :] * self._r_stations[None, :, None]
        v_base = wind_world - v_hub_world - v_i_vec                  # (3,)
        ua     = v_base[None, None, :] - v_rot                       # (N_AB, N_RADIAL, 3)

        # ── Velocity decomposition ────────────────────────────────────────────
        ua_norm   = np.sqrt(np.einsum('ijk,ijk->ij', ua, ua))        # (N_AB, N_RADIAL)
        # Project onto chord / normal: einsum contracts last axis (dim 3)
        ua_chord  = np.einsum('ijk,ik->ij', ua, e_chord_world)       # (N_AB, N_RADIAL)
        ua_normal = np.einsum('ijk,ik->ij', ua, e_normal_world)      # (N_AB, N_RADIAL)

        # Valid strips: sufficient speed and non-degenerate chord component
        valid = (ua_norm >= 0.5) & (np.abs(ua_chord) >= 1e-6)        # (N_AB, N_RADIAL) bool

        if not np.any(valid):
            self.last_M_spin = np.zeros(3)
            self.last_M_cyc  = np.zeros(3)
            self.last_Q_spin = 0.0
            return AeroResult(
                F_world=np.zeros(3), M_orbital=np.zeros(3), Q_spin=0.0, M_spin=np.zeros(3)
            )

        # ── AoA with stall clamp; zero on invalid strips ──────────────────────
        safe_chord = np.where(np.abs(ua_chord) >= 1e-6, ua_chord, 1.0)
        alpha = np.where(valid,
                         np.clip(-ua_normal / safe_chord, -self.AOA_LIMIT, self.AOA_LIMIT),
                         0.0)                                         # (N_AB, N_RADIAL)

        # ── Aerodynamic coefficients ──────────────────────────────────────────
        CL = self.CL0 + self.CL_ALPHA * alpha                        # (N_AB, N_RADIAL)
        CD = self.CD0 + CL ** 2 / (math.pi * self.AR * self.OSWALD) # (N_AB, N_RADIAL)
        # Dynamic pressure × strip area; zero on invalid strips
        q_dyn = np.where(valid, 0.5 * self.RHO * ua_norm ** 2 * self._S_elem, 0.0)

        # ── Lift direction (Kutta-Joukowski: ua × e_span) ─────────────────────
        # ua: (N_AB, N_RADIAL, 3), e_span_world: (N_AB, 3) → broadcast over radial
        lift_raw  = np.cross(ua, e_span_world[:, None, :])           # (N_AB, N_RADIAL, 3)
        lift_norm = np.linalg.norm(lift_raw, axis=-1, keepdims=True) # (N_AB, N_RADIAL, 1)
        e_lift    = np.where(lift_norm > 1e-9,
                             lift_raw / np.maximum(lift_norm, 1e-9),
                             0.0)                                     # (N_AB, N_RADIAL, 3)

        # ── Per-strip force and moment ────────────────────────────────────────
        ua_unit = ua / np.maximum(ua_norm[..., None], 1e-9)          # (N_AB, N_RADIAL, 3)
        F_strip = q_dyn[..., None] * (CL[..., None] * e_lift
                                      + CD[..., None] * ua_unit)     # (N_AB, N_RADIAL, 3)
        M_strip = np.cross(r_cp_world, F_strip)                      # (N_AB, N_RADIAL, 3)

        # Torque component along spin axis per strip: (N_AB, N_RADIAL)
        Q_strip = (np.einsum('ijk,ik->ij', F_strip, e_tang)
                   * self._r_stations[None, :] * spin_sign)

        # ── Accumulate ───────────────────────────────────────────────────────
        F_acc       = F_strip.sum(axis=(0, 1))                        # (3,)
        M_acc       = M_strip.sum(axis=(0, 1))                        # (3,)
        Q_drive_acc = float(Q_strip[Q_strip >= 0].sum())
        Q_drag_acc  = float(Q_strip[Q_strip  < 0].sum())

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale
        Q_drive = Q_drive_acc * scale
        Q_drag  = Q_drag_acc  * scale

        # Decompose moment into spin-axis and orbital components
        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # ── Diagnostics ───────────────────────────────────────────────────────
        self.last_T              = float(np.dot(F_total, disk_normal))
        self.last_v_axial        = v_axial
        self.last_v_i            = v_i
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_collective_rad = collective_rad
        self.last_tilt_lon       = tilt_lon
        self.last_tilt_lat       = tilt_lat
        self.last_Q_spin         = float(self.k_drive_spin * v_inplane
                                         - self.k_drag_spin * omega_abs ** 2)
        self.last_Q_drive        = Q_drive
        self.last_Q_drag         = Q_drag
        self.last_H_force        = float(np.linalg.norm(
            F_total - self.last_T * disk_normal))
        self.last_M_spin         = M_spin_world.copy()
        self.last_M_cyc          = M_cyc_world.copy()

        log.debug("t=%.3f T=%.2fN H=%.2fN v_axial=%.2f v_i=%.2f ramp=%.2f",
                  t, self.last_T, self.last_H_force, v_axial, v_i, ramp)

        return AeroResult(
            F_world   = F_total.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs**2),
            M_spin    = M_spin_world.copy(),
        )
