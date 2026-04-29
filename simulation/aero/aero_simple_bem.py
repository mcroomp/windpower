"""
aero_simple_bem.py — Minimal textbook BEM for sanity cross-checking.

Simplifications versus the production models:
  - No Prandtl tip / hub loss            (F = 1 everywhere)
  - No tangential induction              (a' = 0)
  - No Glauert / Buhl correction         (axial induction clamped at 0.5)
  - Axial induction by fixed-point iteration (10 steps, damped)
  - N_AZ azimuthal stations, N_RADIAL uniform radial strips
  - Quasi-static — no ODE states

Alpha convention (Leishman helicopter, NOT OpenFAST wind-turbine):
  phi   = atan2(Vx*(1-a), Vy)   where Vx = axial inflow (positive = upward through disk)
  alpha = pitch - phi            more collective -> higher AoA -> more thrust

At Vx=0 with a=0: phi=0, alpha = pitch.
Zero CL -> zero thrust when pitch = -CL0/CL_alpha (nose-down to cancel camber).

Same compute_forces() interface as PetersHeBEM and OpenFASTBEM.
"""

import math

import numpy as np

from . import AeroResult, AeroBase
from .rotor_definition import RotorDefinition


class SimpleBEM(AeroBase):
    """Minimal blade-element / momentum model — sanity reference only."""

    N_AZ     = 8
    N_RADIAL = 10
    A_MAX    = 0.5   # axial induction clamp (avoids turbulent-windmill divergence)

    def __init__(self, rotor: RotorDefinition, ramp_time: float = 5.0,
                 state_dict: dict | None = None):
        self.N_BLADES       = rotor.n_blades
        self.R_ROOT         = rotor.root_cutout_m
        self.R_TIP          = rotor.radius_m
        self.CHORD          = rotor.chord_m
        self.RHO            = rotor.rho_kg_m3
        self.CL0            = rotor.CL0
        self.CL_ALPHA       = rotor.CL_alpha_per_rad
        self.CD0            = rotor.CD0
        self.AR             = rotor.aspect_ratio
        self.OSWALD         = rotor.oswald_efficiency
        self.AOA_LIMIT      = math.radians(rotor.alpha_stall_deg)
        self.ramp_time      = float(ramp_time)
        self.pitch_gain_rad = rotor.swashplate_pitch_gain_rad

        span = self.R_TIP - self.R_ROOT
        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )

        # Diagnostics — same attribute names as production models
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_skew_angle_deg = 0.0
        self.last_M_spin         = np.zeros(3)

    def cl_cd(self, alpha_rad: float) -> tuple[float, float]:
        alpha_c = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, alpha_rad))
        CL = self.CL0 + self.CL_ALPHA * alpha_c
        CD = self.CD0 + CL ** 2 / (math.pi * self.AR * self.OSWALD)
        return CL, CD

    def to_dict(self) -> dict:
        return {"type": "SimpleBEM"}

    def is_valid(self) -> bool:
        return math.isfinite(self.last_T)

    # ── BEM core ─────────────────────────────────────────────────────────────

    def _axial_induction(self, Cn: float, sigma_p: float, phi: float) -> float:
        """Momentum-theory axial induction with no tip-loss (F=1)."""
        sin_phi = math.sin(phi)
        if abs(sin_phi) < 1e-6:
            return 0.0
        k = sigma_p * Cn / (4.0 * sin_phi * sin_phi)
        a = k / (1.0 + k) if k > -1.0 else 0.0
        return min(self.A_MAX, max(-0.5, a))

    def _solve_strip(self, Vx: float, Vy: float,
                     sigma_p: float, pitch: float) -> tuple:
        """Ten fixed-point iterations for one (azimuth, radius) element."""
        a = 0.0
        CL = Ct = Cn = phi = 0.0
        for _ in range(10):
            phi = math.atan2(Vx * (1.0 - a), max(abs(Vy), 1e-6))
            if Vy < 0.0:
                phi = -phi
            alpha = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, pitch - phi))
            CL   = self.CL0 + self.CL_ALPHA * alpha
            CD   = self.CD0 + CL * CL / (math.pi * self.AR * self.OSWALD)
            Cn   = CL * math.cos(phi) + CD * math.sin(phi)
            Ct   = CL * math.sin(phi) - CD * math.cos(phi)
            a_new = self._axial_induction(Cn, sigma_p, phi)
            a = 0.5 * (a + a_new)          # damped update for convergence
        return phi, a, Cn, Ct

    # ── Public interface ──────────────────────────────────────────────────────

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
        ramp = 1.0 if self.ramp_time <= 0.0 else min(1.0, t / self.ramp_time)

        disk_normal  = R_hub[:, 2]
        omega_abs    = abs(float(omega_rotor))
        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        F_acc = np.zeros(3)
        M_acc = np.zeros(3)

        for i in range(self.N_AZ):
            phi_az = spin_angle + (2.0 * math.pi * i) / self.N_AZ
            ca, sa = math.cos(phi_az), math.sin(phi_az)

            pitch = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca

            e_span_body  = np.array([ca, sa, 0.0])
            e_span_world = R_hub @ e_span_body
            e_tang       = np.cross(disk_normal, e_span_world)

            # In-plane wind resolved along the blade's tangential direction
            # (advancing side adds velocity, retreating side subtracts)
            v_tang = float(np.dot(v_inplane_vec, e_tang))

            for j in range(self.N_RADIAL):
                r = self._r_stations[j]
                Vy = omega_abs * r + v_tang
                if abs(v_axial) < 1e-4 and abs(Vy) < 1e-4:
                    continue

                sigma_p = self.N_BLADES * self.CHORD / (2.0 * math.pi * r)
                phi, a, Cn, Ct = self._solve_strip(v_axial, Vy, sigma_p, pitch)

                Vx_eff  = v_axial * (1.0 - a)
                Vrel_sq = Vx_eff * Vx_eff + Vy * Vy
                q       = 0.5 * self.RHO * Vrel_sq * self.CHORD * self._dr

                dF = q * Cn * disk_normal + q * Ct * e_tang
                F_acc += dF
                M_acc += np.cross(r * e_span_world, dF)

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        T_disk       = float(np.dot(F_total, disk_normal))
        Q_spin_sc    = float(np.dot(M_total, disk_normal))
        M_spin_world = Q_spin_sc * disk_normal
        M_cyc_world  = M_total - M_spin_world

        V_T = math.sqrt(v_inplane ** 2 + v_axial ** 2 + 1e-6)
        v_i = T_disk / (2.0 * self.RHO * math.pi * self.R_TIP ** 2 * max(V_T, 0.1))
        chi = math.atan2(v_inplane, max(abs(v_axial + v_i), 0.01))

        self.last_T              = T_disk
        self.last_v_axial        = v_axial
        self.last_v_i            = v_i
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_skew_angle_deg = math.degrees(chi)
        self.last_M_spin         = np.array(M_spin_world)

        return AeroResult(
            F_world   = np.array(F_total),
            M_orbital = np.array(M_cyc_world),
            Q_spin    = Q_spin_sc,
            M_spin    = np.array(M_spin_world),
        )
