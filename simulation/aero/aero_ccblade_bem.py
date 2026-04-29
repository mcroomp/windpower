"""
aero_ccblade_bem.py — BEM with tabulated polars; no external CCBlade dependency.

Replaces the linear CL_alpha airfoil model used by all other models in this
project with a tabulated XFOIL/AeroDyn polar (CL and CD as functions of alpha
and Re).  The BEM phi-solve uses Ning (2013) guaranteed-convergence formulation,
translated directly from CCBlade's Fortran inductionFactors subroutine.

Key improvements over OpenFASTBEM / PetersHeBEM:
  - Re-dependent CL and CD from a real polar (e.g. sg6040_re500k.csv)
  - Drag is correctly interpolated near stall, not modelled as CL²/(pi·AR·e)
  - Accurate for any pitch angle that stays within the polar's alpha range
  - Prandtl tip + hub loss, Glauert/Buhl high-thrust correction

Limitations vs PetersHeBEM:
  - No dynamic inflow ODE (fully quasi-static — same as OpenFASTBEM)
  - Requires v_axial != 0: returns near-zero loads at pure hover with zero wind

Setup
-----
Add ``polar_csv: sg6040_re500k.csv`` to the rotor YAML under ``airfoil:``.
If omitted, a synthetic linear polar is built from CL0/CL_alpha/CD0/AR/e.

Alpha / pitch convention (helicopter, consistent with all other models):
  more collective  →  higher AoA  →  more thrust
  alpha = collective - phi_he   (helicopter phi = inflow angle, positive upward)

Internally the BEM residual is solved in the wind-turbine convention
  alpha_wt = phi_wt - pitch_wt  where  pitch_wt = -collective_rad
so that Ning's (2013) convergence regions are correct.
"""

import math
import logging

import numpy as np
from scipy.interpolate import RectBivariateSpline
from scipy.optimize import brentq

from . import AeroResult, AeroBase
from .rotor_definition import RotorDefinition

log = logging.getLogger(__name__)

_EPS_PHI = 1e-6
_MU_AIR  = 1.81206e-5   # dynamic viscosity of air [Pa·s]


# ── Private polar class ───────────────────────────────────────────────────────

class _Polar:
    """
    Bivariate spline over (alpha_rad, Re) for CL and CD.
    Mirrors CCAirfoil from S. Andrew Ning's CCBlade (Apache-2.0).
    """

    def __init__(self, alpha_deg, Re, cl, cd):
        """
        alpha_deg : array-like (deg), shape (n_alpha,)
        Re        : array-like, shape (n_Re,) — duplicate if only one value
        cl, cd    : array-like, shape (n_alpha, n_Re)
        """
        alpha_rad = np.deg2rad(np.asarray(alpha_deg, dtype=float))
        Re  = np.asarray(Re,  dtype=float).ravel()
        cl  = np.asarray(cl,  dtype=float)
        cd  = np.asarray(cd,  dtype=float)

        if len(Re) < 2:
            Re = np.array([1e1, 1e15])
            cl = np.c_[cl, cl]
            cd = np.c_[cd, cd]

        kx = min(len(alpha_rad) - 1, 3)
        ky = min(len(Re) - 1, 3)

        self._cl_sp = RectBivariateSpline(alpha_rad, Re, cl, kx=kx, ky=ky, s=0.1)    # type: ignore[arg-type]
        self._cd_sp = RectBivariateSpline(alpha_rad, Re, cd, kx=kx, ky=ky, s=0.001)  # type: ignore[arg-type]

    def evaluate(self, alpha_rad: float, Re: float) -> tuple[float, float]:
        return float(self._cl_sp.ev(alpha_rad, Re)), float(self._cd_sp.ev(alpha_rad, Re))


# ── Ning (2013) BEM kernel ────────────────────────────────────────────────────

def _induction_factors(r, chord, Rhub, Rtip, phi, cl, cd, B, Vx, Vy):
    """
    Translated from CCBlade's Fortran inductionFactors subroutine (Ning 2013).
    Assumes useCd=hubLoss=tipLoss=wakerotation=True.
    Returns (fzero, a, ap).
    """
    pi = math.pi
    sigma_p = B * chord / (2.0 * pi * r)
    sphi = math.sin(phi)
    cphi = math.cos(phi)

    cn = cl * cphi + cd * sphi
    ct = cl * sphi - cd * cphi

    factortip = B / 2.0 * (Rtip - r) / (r * sphi)
    Ftip = (2.0 / pi) * math.acos(min(1.0, math.exp(-factortip)))

    factorhub = B / 2.0 * (r - Rhub) / (Rhub * sphi)
    Fhub = (2.0 / pi) * math.acos(min(1.0, math.exp(-factorhub)))

    F = max(Ftip * Fhub, 1e-6)

    k  = sigma_p * cn / (4.0 * F * sphi * sphi)
    kp = sigma_p * ct / (4.0 * F * sphi * cphi)

    if phi > 0.0:
        if k <= 2.0 / 3.0:
            a = k / (1.0 + k)
        else:
            g1 = 2.0 * F * k - (10.0 / 9.0 - F)
            g2 = 2.0 * F * k - (4.0 / 3.0 - F) * F
            g3 = 2.0 * F * k - (25.0 / 9.0 - 2.0 * F)
            if abs(g3) < 1e-6:
                a = 1.0 - 1.0 / (2.0 * math.sqrt(max(g2, 1e-12)))
            else:
                a = (g1 - math.sqrt(max(g2, 0.0))) / g3
    else:
        a = k / (k - 1.0) if k > 1.0 else 0.0

    ap = kp / (1.0 - kp)

    lambda_r = Vy / Vx
    if phi > 0.0:
        fzero = sphi / (1.0 - a) - cphi / lambda_r * (1.0 - kp)
    else:
        fzero = sphi * (1.0 - k) - cphi / lambda_r * (1.0 - kp)

    return fzero, a, ap


# ── Helpers ───────────────────────────────────────────────────────────────────

def _prandtl_F(B: int, r: float, R_tip: float, R_root: float, phi: float) -> float:
    """Prandtl tip + hub loss factor."""
    sin_phi = max(abs(math.sin(phi)), 1e-4)
    f_tip = (B / 2.0) * (R_tip - r)             / (r             * sin_phi)
    f_hub = (B / 2.0) * (r    - R_root)          / (max(R_root, 0.01) * sin_phi)
    F_tip = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_tip))))
    F_hub = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_hub))))
    return max(1e-4, F_tip * F_hub)


def _build_synthetic_polar(CL0, CL_alpha, CD0, AR, oswald, aoa_limit_rad):
    """Build a _Polar from a linear lift + polar drag model."""
    alpha_deg = np.linspace(-180, 180, 361)
    alpha_rad = np.deg2rad(alpha_deg)
    cl = np.clip(CL0 + CL_alpha * alpha_rad,
                 CL0 + CL_alpha * (-aoa_limit_rad),
                 CL0 + CL_alpha * aoa_limit_rad)
    cd = np.maximum(CD0 + cl ** 2 / (math.pi * AR * oswald), CD0)
    return _Polar(alpha_deg, [1e6], cl.reshape(-1, 1), cd.reshape(-1, 1))


# ── CCBladeBEM ────────────────────────────────────────────────────────────────

class CCBladeBEM(AeroBase):
    """
    Quasi-static BEM with tabulated polars.

    Drop-in replacement for PetersHeBEM / OpenFASTBEM.  Same constructor,
    same compute_forces() signature.  No ODE inflow states.

    Polar priority:
      1. Tabulated polar from ``polar_csv`` field in the rotor YAML
      2. Synthetic linear polar built from CL0 / CL_alpha / CD0 / AR / e
    """

    N_AZ     = 12
    N_RADIAL = 20

    def __init__(self, rotor: RotorDefinition, ramp_time: float = 5.0,
                 state_dict: dict | None = None):
        self.N_BLADES       = rotor.n_blades
        self.R_ROOT         = rotor.root_cutout_m
        self.R_TIP          = rotor.radius_m
        self.CHORD          = rotor.chord_m
        self.RHO            = rotor.rho_kg_m3
        self.AOA_LIMIT      = math.radians(rotor.alpha_stall_deg)
        self.ramp_time      = float(ramp_time)
        self.pitch_gain_rad = rotor.swashplate_pitch_gain_rad

        self.AR        = rotor.aspect_ratio
        self.disk_area = math.pi * self.R_TIP ** 2

        span = self.R_TIP - self.R_ROOT
        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )

        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        # ── Airfoil polar ────────────────────────────────────────────────────
        _polar = rotor.polar_arrays()
        if _polar is not None:
            alpha_rad, cl_arr, cd_arr = (np.asarray(a, dtype=float) for a in _polar)
            self._Re_ref = float(rotor.Re_design)
            self._af = _Polar(
                np.degrees(alpha_rad),
                [self._Re_ref],
                cl_arr.reshape(-1, 1),
                cd_arr.reshape(-1, 1),
            )
            self._polar_source = "tabulated"
            log.info("CCBladeBEM: loaded tabulated polar (%d points, Re=%.0f)",
                     len(alpha_rad), self._Re_ref)
        else:
            self._Re_ref = float(rotor.Re_design)
            self._af = _build_synthetic_polar(
                rotor.CL0, rotor.CL_alpha_per_rad,
                rotor.CD0, self.AR,
                rotor.oswald_efficiency, self.AOA_LIMIT,
            )
            self._polar_source = "linear"
            log.info("CCBladeBEM: using synthetic linear polar (no polar_csv in YAML)")

        # ── Diagnostics (same names as PetersHeBEM / OpenFASTBEM) ────────────
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_H_force        = 0.0
        self.last_F_prandtl      = 1.0
        self.last_v1c            = 0.0
        self.last_v1s            = 0.0
        self.last_tau0           = 0.0
        self.last_skew_angle_deg = 0.0

    def cl_cd(self, alpha_rad: float) -> tuple[float, float]:
        return self._af.evaluate(alpha_rad, self._Re_ref)

    def to_dict(self) -> dict:
        return {"type": "CCBladeBEM", "polar_source": self._polar_source}

    def is_valid(self) -> bool:
        return math.isfinite(self.last_T)

    def _ramp_factor(self, t: float) -> float:
        if self.ramp_time <= 0.0 or t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    # ── BEM strip solver ─────────────────────────────────────────────────────

    def _solve_strip(
        self,
        Vx: float,
        Vy: float,
        r: float,
        pitch_wt: float,
    ) -> tuple[float, float, float, float, float]:
        """
        Solve BEM residual for one (azimuth, radius) element.
        Uses Ning (2013) _induction_factors with _Polar lookup at each phi.

        Parameters
        ----------
        Vx       : axial inflow (positive = wind into rotor face)
        Vy       : tangential velocity (positive = in direction of rotation)
        r        : radial station [m]
        pitch_wt : collective in wind-turbine convention [rad]

        Returns (phi, a, ap, cl, cd) at the converged solution.
        Falls back to geometric phi with a=ap=0 on solver failure.
        """
        if abs(Vx) < 1e-4 and abs(Vy) < 1e-4:
            return 0.0, 0.0, 0.0, 0.0, 0.0

        W_geom = math.sqrt(Vx * Vx + Vy * Vy)
        Re     = max(self.RHO * W_geom * self.CHORD / _MU_AIR, 1e4)

        def _cl_cd(phi: float) -> tuple[float, float]:
            alpha = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, phi - pitch_wt))
            return self._af.evaluate(alpha, Re)

        def residual(phi: float) -> float:
            cl, cd = _cl_cd(phi)
            fz, _, _ = _induction_factors(
                r, self.CHORD, self.R_ROOT, self.R_TIP,
                phi, cl, cd, self.N_BLADES, Vx, Vy,
            )
            return float(fz)

        eps      = _EPS_PHI
        half_pi  = math.pi * 0.5
        phi_sol  = None

        for lo, hi in [(eps, half_pi - eps), (half_pi + eps, math.pi - eps)]:
            try:
                if residual(lo) * residual(hi) < 0.0:
                    phi_sol = float(brentq(residual, lo, hi, xtol=1e-5, maxiter=50))  # type: ignore[arg-type]
                    break
            except Exception:
                pass

        if phi_sol is None:
            phi_sol = math.atan2(abs(Vx), max(abs(Vy), 1e-6))
            if Vx < 0.0:
                phi_sol = -phi_sol
            cl, cd = _cl_cd(phi_sol)
            return phi_sol, 0.0, 0.0, cl, cd

        cl, cd = _cl_cd(phi_sol)
        _, a, ap = _induction_factors(
            r, self.CHORD, self.R_ROOT, self.R_TIP,
            phi_sol, cl, cd, self.N_BLADES, Vx, Vy,
        )

        a  = float(max(-1.0, min(1.5, a)))
        ap = float(max(-1.0, min(1.0, ap)))
        W_sol  = math.sqrt((Vx * (1.0 - a)) ** 2 + (Vy * (1.0 + ap)) ** 2)
        Re_sol = max(self.RHO * W_sol * self.CHORD / _MU_AIR, 1e4)
        alpha_sol = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, phi_sol - pitch_wt))
        cl, cd = self._af.evaluate(alpha_sol, Re_sol)

        return phi_sol, a, ap, float(cl), float(cd)

    # ── 3D strip integration ─────────────────────────────────────────────────

    def _strip_forces(
        self,
        v_rel_world:    np.ndarray,
        disk_normal:    np.ndarray,
        R_hub:          np.ndarray,
        omega_rotor:    float,
        collective_rad: float,
        tilt_lon_rad:   float,
        tilt_lat_rad:   float,
        spin_angle:     float,
    ) -> tuple[np.ndarray, np.ndarray]:
        omega_abs = abs(float(omega_rotor))
        F_acc     = np.zeros(3)
        M_acc     = np.zeros(3)
        F_prandtl_sum = 0.0

        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal

        for i in range(self.N_AB):
            phi_az = spin_angle + self._phi_offsets[i]
            ca = math.cos(phi_az)
            sa = math.sin(phi_az)

            pitch = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca

            e_span_body  = np.array([ca, sa, 0.0])
            e_span_world = R_hub @ e_span_body
            e_tang       = np.cross(disk_normal, e_span_world)

            v_tang = float(np.dot(v_inplane_vec, e_tang))

            for j in range(self.N_RADIAL):
                r  = self._r_stations[j]
                Vy = omega_abs * r + v_tang
                Vx = v_axial

                if abs(Vx) < 1e-4 and abs(Vy) < 1e-4:
                    continue

                phi_sol, a, ap, cl, cd = self._solve_strip(Vx, Vy, r, pitch)

                if i == 0:
                    F_prandtl_sum += _prandtl_F(
                        self.N_BLADES, r, self.R_TIP, self.R_ROOT, phi_sol
                    )

                a  = max(-1.0, min(1.5, a))
                ap = max(-1.0, min(1.0, ap))

                Vx_eff  = Vx * (1.0 - a)
                Vy_eff  = Vy * (1.0 + ap)
                Vrel_sq = Vx_eff * Vx_eff + Vy_eff * Vy_eff
                if Vrel_sq < 0.25:
                    continue

                q_elem = 0.5 * self.RHO * Vrel_sq * self.CHORD * self._dr

                Cn = cl * math.cos(phi_sol) + cd * math.sin(phi_sol)
                Ct = cl * math.sin(phi_sol) - cd * math.cos(phi_sol)

                F_strip = q_elem * Cn * disk_normal + q_elem * Ct * e_tang
                M_acc  += np.cross(r * e_span_world, F_strip)
                F_acc  += F_strip

        if self.N_RADIAL > 0:
            self.last_F_prandtl = F_prandtl_sum / self.N_RADIAL

        return F_acc, M_acc

    # ── Public interface ─────────────────────────────────────────────────────

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
        ramp = self._ramp_factor(t)

        disk_normal   = R_hub[:, 2]
        tilt_lon_rad  = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad  = tilt_lat * self.pitch_gain_rad

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # Negate: helicopter convention → wind-turbine convention
        F_acc, M_acc = self._strip_forces(
            v_rel_world, disk_normal, R_hub, omega_rotor,
            -collective_rad, -tilt_lon_rad, -tilt_lat_rad, spin_angle,
        )

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        T_disk        = float(np.dot(F_total, disk_normal))
        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        V_T = math.sqrt(v_inplane ** 2 + v_axial ** 2 + 1e-6)
        v_i = T_disk / (2.0 * self.RHO * self.disk_area * max(V_T, 0.1))
        chi = math.atan2(v_inplane, max(abs(v_axial + v_i), 0.01))

        self.last_T              = T_disk
        self.last_v_axial        = v_axial
        self.last_v_i            = v_i
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_M_spin         = np.array(M_spin_world)
        self.last_M_cyc          = np.array(M_cyc_world)
        self.last_H_force        = float(np.linalg.norm(F_total - T_disk * disk_normal))
        self.last_skew_angle_deg = math.degrees(chi)

        log.debug(
            "t=%.3f T=%.2fN v_i=%.3f v_ax=%.2f v_ip=%.2f polar=%s",
            t, T_disk, v_i, v_axial, v_inplane, self._polar_source,
        )

        return AeroResult(
            F_world   = np.array(F_total),
            M_orbital = np.array(M_cyc_world),
            Q_spin    = Q_spin_scalar,
            M_spin    = np.array(M_spin_world),
        )
