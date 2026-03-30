"""
aero_strip.py — Independent strip-theory thrust estimator (no BEM actuator disk).

Third independent estimate — different principles from RotorAero (lumped BEM) and
DeSchutterAero (per-blade strip with BEM induction):

    RotorAero      — lumped single-point BEM at r_cp, actuator-disk induction
    DeSchutterAero — per-blade multi-strip with BEM induction
    StripThrust    — per-strip radial integration, NO induction (kinematic only)
                     + Glauert oblique-flow correction for near-edgewise regime

Glauert oblique-flow momentum theory (1926):
    T = 2 ρ A v_i √( v_inplane² + (v_axial + v_i)² )
    Reduces to standard actuator disk at v_inplane=0 (purely axial flow).
    Does NOT drop to zero at ξ=90° unlike the pure-axial De Schutter Eq. 17.

This module is a standalone analysis tool — not part of the simulation runtime.
"""

import math
import numpy as np
from typing import Optional


class StripThrust:
    """
    Independent strip-theory thrust estimate with Glauert oblique-flow induction.

    Radial integration over N strips from r_root to r_tip.
    Each strip sees:
      - Rotational velocity Ω × r  (tangential)
      - Axial effective wind:       v_axial + v_i   (normal to disk)
      - Induced velocity v_i from Glauert's oblique-flow momentum equation

    NO lumped r_cp approximation.  Properly captures the r³ dependence of dT/dr.

    Parameters
    ----------
    n_blades, r_root, r_tip, chord, rho, CL0, CL_alpha, CD0, aoa_limit_deg : floats
    n_strips : int   radial integration strips (default 20)
    """

    def __init__(self, n_blades: int, r_root: float, r_tip: float,
                 chord: float, rho: float, CL0: float, CL_alpha: float,
                 CD0: float, aoa_limit_deg: float = 12.0, n_strips: int = 20):
        self.n_blades   = n_blades
        self.r_root     = r_root
        self.r_tip      = r_tip
        self.chord      = chord
        self.rho        = rho
        self.CL0        = CL0
        self.CL_alpha   = CL_alpha
        self.CD0        = CD0
        self.aoa_limit  = math.radians(aoa_limit_deg)
        self.n_strips   = n_strips

        self.span       = r_tip - r_root
        self.disk_area  = math.pi * (r_tip**2 - r_root**2)

        # Strip radii and areas (midpoint rule)
        dr       = self.span / n_strips
        self.r_i = np.array([r_root + (i + 0.5) * dr for i in range(n_strips)])
        self.S_i = n_blades * chord * dr   # blade area per strip (all blades)
        self.dr  = dr

    def _glauert_vi(self, T_guess: float, v_axial: float, v_inplane: float) -> float:
        """
        Glauert oblique-flow induced velocity (iterative).

        T = 2 ρ A v_i √( v_inplane² + (v_axial + v_i)² )
        → v_i² + 2 v_axial v_i + v_axial² + v_inplane² = (T/(2ρA v_i))²

        Solved by fixed-point iteration from the axial solution as starting point.
        Returns v_i ≥ 0.
        """
        T_abs = max(abs(T_guess), 0.01)
        A     = self.disk_area
        rhoA2 = 2.0 * self.rho * A
        v_ax  = abs(v_axial)
        v_ip  = abs(v_inplane)

        # Axial starting point (pure actuator disk)
        disc = v_ax**2 + 2.0 * T_abs / (self.rho * A)
        vi   = max(0.0, (-v_ax + math.sqrt(disc)) / 2.0)

        # Glauert fixed-point: vi = T / (rhoA2 × sqrt(v_ip² + (v_ax + vi)²))
        for _ in range(8):
            denom = math.sqrt(v_ip**2 + (v_ax + vi)**2)
            if denom < 1e-6:
                break
            vi_new = T_abs / (rhoA2 * denom)
            vi = max(0.0, 0.5 * vi + 0.5 * vi_new)   # damped iteration

        return vi

    def thrust(self, v_axial: float, v_inplane: float,
               omega_rotor: float, collective_rad: float,
               n_iter: int = 4) -> dict:
        """
        Compute total thrust and torque by strip integration.

        Parameters
        ----------
        v_axial      : float — wind component normal to disk [m/s]  (positive = into disk)
        v_inplane    : float — wind component in disk plane  [m/s]
        omega_rotor  : float — rotor spin [rad/s]
        collective_rad : float — blade pitch [rad]
        n_iter       : int   — induction iterations

        Returns
        -------
        dict with keys: T_N, Q_Nm, CL_mean, v_i, breakdown (per-strip T array)
        """
        # Integrate strip by strip with Glauert-corrected induction
        v_ax  = abs(v_axial)
        v_ip  = abs(v_inplane)
        omega = abs(omega_rotor)

        # Initial total thrust estimate (no induction) for Glauert seed
        T_strips = np.zeros(self.n_strips)
        for idx, r in enumerate(self.r_i):
            v_tan   = omega * r
            v_loc   = math.sqrt(v_tan**2 + v_ax**2)
            if v_loc < 0.5:
                continue
            phi     = math.atan2(v_ax, v_tan)
            aoa     = max(-self.aoa_limit, min(self.aoa_limit, phi + collective_rad))
            CL      = self.CL0 + self.CL_alpha * aoa
            CD      = self.CD0 + CL**2 / (math.pi * (self.span / self.chord) * 0.8)
            q       = 0.5 * self.rho * v_loc**2
            dT      = max(0.0, q * self.S_i * (CL * math.cos(phi) - CD * math.sin(phi)))
            T_strips[idx] = dT

        T_total = float(np.sum(T_strips))

        # Glauert induced velocity on whole disk
        v_i = 0.0
        for _ in range(n_iter):
            v_i = self._glauert_vi(T_total, v_ax, v_ip)
            v_eff = v_ax + v_i

            T_strips = np.zeros(self.n_strips)
            Q_strips = np.zeros(self.n_strips)
            CL_vals  = []
            for idx, r in enumerate(self.r_i):
                v_tan   = omega * r
                v_loc   = math.sqrt(v_tan**2 + v_eff**2)
                if v_loc < 0.5:
                    continue
                phi     = math.atan2(v_eff, v_tan)
                aoa     = max(-self.aoa_limit, min(self.aoa_limit, phi + collective_rad))
                CL      = self.CL0 + self.CL_alpha * aoa
                CD      = self.CD0 + CL**2 / (math.pi * (self.span / self.chord) * 0.8)
                q       = 0.5 * self.rho * v_loc**2
                dT      = max(0.0, q * self.S_i * (CL * math.cos(phi) - CD * math.sin(phi)))
                dQ      = q * self.S_i * r * (CL * math.sin(phi) - CD * math.cos(phi))
                T_strips[idx] = dT
                Q_strips[idx] = dQ
                CL_vals.append(CL)

            T_total = float(np.sum(T_strips))

        return {
            "T_N":       T_total,
            "Q_Nm":      float(np.sum(Q_strips)),
            "v_i":       v_i,
            "CL_mean":   float(np.mean(CL_vals)) if CL_vals else 0.0,
            "T_strips":  T_strips,
            "betz_frac": T_total / max(
                0.5 * self.rho * (v_ax**2 + v_ip**2) * self.disk_area * 0.89, 0.01),
        }


def from_rotor(rotor, n_strips: int = 20) -> "StripThrust":
    """Build a StripThrust from a RotorDefinition."""
    p = rotor.aero_kwargs()
    return StripThrust(
        n_blades      = int(p["n_blades"]),
        r_root        = float(p["r_root"]),
        r_tip         = float(p["r_tip"]),
        chord         = float(p["chord"]),
        rho           = float(p["rho"]),
        CL0           = float(p["CL0"]),
        CL_alpha      = float(p["CL_alpha"]),
        CD0           = float(p["CD0"]),
        aoa_limit_deg = math.degrees(float(p["aoa_limit"])),
        n_strips      = n_strips,
    )
