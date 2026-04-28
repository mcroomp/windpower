"""
aero_openfast_bem.py — Quasi-static BEM using the OpenFAST BEMTUncoupled algorithm.

Core algorithm ported from:
  OpenFAST/modules/aerodyn/src/BEMTUncoupled.f90  (Apache 2.0)

Key features relative to PetersHeBEM:
  • Phi solved at each blade element via Brent's method (self-consistent BEM)
  • Glauert/Buhl high-thrust correction for a > 0.4 (where momentum theory breaks down)
  • Prandtl tip + hub loss computed at the actual phi solution, not a geometric estimate
  • Tangential induction a' included
  • No ODE states — fully quasi-static

Drop-in replacement for PetersHeBEM.  Same constructor, same compute_forces()
signature, same diagnostic attributes.

References
──────────
OpenFAST AeroDyn, BEMTUncoupled.f90, NREL (Apache 2.0)
Buhl M.L. (2005) "A New Empirical Relationship between Thrust Coefficient and
  Induction Factor for the Turbulent Windmill State", NREL/TP-500-36834.
"""

import math
import logging
import sys
import os
import numpy as np
from scipy.optimize import brentq

_SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

from aero import AeroResult

log = logging.getLogger(__name__)

_EPS_PHI = 1e-6   # avoid phi=0 and phi=pi/2 singularities


# ── Low-level BEM functions ──────────────────────────────────────────────────

def _prandtl_F(B: int, r: float, R_tip: float, R_root: float,
               phi: float) -> float:
    """Prandtl tip + hub loss factor (OpenFAST getHubTipLossCorrection, BEMMod_2D)."""
    sin_phi = abs(math.sin(phi))
    if sin_phi < 1e-4:
        sin_phi = 1e-4

    # tip:  factortip = (B/2)·(R_tip - r) / (r·|sinφ|)
    f_tip = (B / 2.0) * (R_tip - r) / (r * sin_phi)
    F_tip = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_tip))))

    # hub:  factorhub = (B/2)·(r - R_root) / (R_root·|sinφ|)
    r_hub = max(R_root, 0.01)
    f_hub = (B / 2.0) * (r - R_root) / (r_hub * sin_phi)
    F_hub = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_hub))))

    return max(0.0001, F_tip * F_hub)


def _axial_induction_momentum(k: float, F: float) -> float:
    """
    Axial induction from k = σ·Cn/(4·F·sin²φ).

    k ≤ 2/3  → momentum state:   a = k/(1+k)
    k > 2/3  → Glauert/Buhl correction (OpenFAST inductionFactors0 lines 578-606).
    """
    if k <= 2.0 / 3.0:
        if abs(k + 1.0) < 1e-12:
            return 1e6
        return k / (1.0 + k)

    # Glauert/Buhl empirical correction
    temp = 2.0 * F * k
    g1 = temp - (10.0 / 9.0 - F)
    g2 = temp - (4.0  / 3.0 - F) * F
    g3 = temp - (25.0 / 9.0 - 2.0 * F)

    if abs(g3) < 1e-6:
        return 1.0 - 0.5 / math.sqrt(max(g2, 1e-10))
    return (g1 - math.sqrt(max(g2, 0.0))) / g3


def _induction_and_residual(
    phi: float,
    Vx: float, Vy: float,
    sigma_p: float,
    Cn: float, Ct: float,
    F: float,
) -> tuple[float, float, float]:
    """
    BEM induction factors and residual for a given phi.

    Matches OpenFAST inductionFactors0 (2D, no cant/toe angle).

    Returns (fzero, a, ap).
    """
    sphi = math.sin(phi)
    cphi = math.cos(phi)

    if abs(sphi) < 1e-10:
        return 0.0, 0.0, 0.0

    # Determine region
    momentum = (phi > 0.0 and Vx >= 0.0) or (phi < 0.0 and Vx < 0.0)

    k  = sigma_p * Cn / (4.0 * F * sphi * sphi)
    kp = 0.0
    a  = 0.0
    ap = 0.0

    if momentum:
        a = _axial_induction_momentum(k, F)
        if k < -1.0:   # invalid in momentum region
            a = 1e6

        if abs(cphi) > 1e-10:
            kp = sigma_p * Ct / (4.0 * F * sphi * cphi)
            if Vx < 0.0:
                kp = -kp
            ap = kp / (1.0 - kp) if abs(kp - 1.0) > 1e-10 else 1e6

        lambda_r = Vy / Vx if abs(Vx) > 1e-8 else 1e8
        if abs(a - 1.0) < 1e-10:
            fzero = -cphi / lambda_r * (1.0 - kp)
        else:
            fzero = sphi / (1.0 - a) - cphi / lambda_r * (1.0 - kp)
    else:
        # propeller-brake region
        a = k / (k - 1.0) if abs(k - 1.0) > 1e-10 else 1e6
        if k <= 1.0:   # invalid in propeller-brake region
            a = 1e6

        if abs(cphi) > 1e-10:
            kp = sigma_p * Ct / (4.0 * F * sphi * cphi)
            if Vx < 0.0:
                kp = -kp
            ap = kp / (1.0 - kp) if abs(kp - 1.0) > 1e-10 else 1e6

        lambda_r = Vy / Vx if abs(Vx) > 1e-8 else 1e8
        fzero = sphi * (1.0 - k) - cphi / lambda_r * (1.0 - kp)

    return fzero, a, ap


def _solve_phi_brent(
    Vx: float, Vy: float,
    sigma_p: float,
    pitch: float,
    B: int, r: float, R_tip: float, R_root: float,
    CL0: float, CL_alpha: float, CD0: float, AR: float,
    OSWALD: float, AOA_LIMIT: float,
) -> tuple[float, float, float]:
    """
    Find the inflow angle phi where the BEM residual = 0.

    Prandtl loss F is re-evaluated at each phi candidate (consistent with
    OpenFAST which calls getHubTipLossCorrection inside BEMTU_InductionWithResidual).

    Returns (phi, a, ap).  Falls back to geometric phi with a=ap=0 if solve fails.
    """
    def airfoil(phi: float) -> tuple[float, float]:
        alpha = max(-AOA_LIMIT, min(AOA_LIMIT, phi - pitch))
        CL = CL0 + CL_alpha * alpha
        CD = CD0 + CL * CL / (math.pi * AR * OSWALD)
        Cn = CL * math.cos(phi) + CD * math.sin(phi)
        Ct = CL * math.sin(phi) - CD * math.cos(phi)
        return Cn, Ct

    def residual(phi: float) -> float:
        F  = _prandtl_F(B, r, R_tip, R_root, phi)
        Cn, Ct = airfoil(phi)
        fz, _, _ = _induction_and_residual(phi, Vx, Vy, sigma_p, Cn, Ct, F)
        return fz

    eps = _EPS_PHI
    half_pi = math.pi / 2.0

    # Try momentum region: phi in (eps, π/2 - eps)
    try:
        fa = residual(eps)
        fb = residual(half_pi - eps)
        if fa * fb < 0.0:
            phi = brentq(residual, eps, half_pi - eps, xtol=1e-5, maxiter=50)
            F   = _prandtl_F(B, r, R_tip, R_root, phi)
            Cn, Ct = airfoil(phi)
            _, a, ap = _induction_and_residual(phi, Vx, Vy, sigma_p, Cn, Ct, F)
            return phi, a, ap
    except Exception:
        pass

    # Try propeller-brake region: phi in (π/2 + eps, π - eps)
    try:
        fa = residual(half_pi + eps)
        fb = residual(math.pi - eps)
        if fa * fb < 0.0:
            phi = brentq(residual, half_pi + eps, math.pi - eps, xtol=1e-5, maxiter=50)
            F   = _prandtl_F(B, r, R_tip, R_root, phi)
            Cn, Ct = airfoil(phi)
            _, a, ap = _induction_and_residual(phi, Vx, Vy, sigma_p, Cn, Ct, F)
            return phi, a, ap
    except Exception:
        pass

    # Fallback: geometric inflow angle, no induction
    phi_geom = math.atan2(abs(Vx), max(abs(Vy), 1e-6)) * (1.0 if Vx >= 0.0 else -1.0)
    return phi_geom, 0.0, 0.0


# ── Main class ───────────────────────────────────────────────────────────────

class OpenFASTBEM:
    """
    Quasi-static BEM with OpenFAST inductionFactors0 algorithm.

    Drop-in replacement for PetersHeBEM.  Same constructor, same
    compute_forces() signature.  No ODE states — fully quasi-static.
    """

    N_AZ     = 12
    N_RADIAL = 10

    def __init__(self, rotor, ramp_time: float = 5.0,
                 state_dict: dict | None = None, **overrides):
        p = {**rotor.aero_kwargs(), **overrides}
        self.N_BLADES       = int(p["n_blades"])
        self.R_ROOT         = float(p["r_root"])
        self.R_TIP          = float(p["r_tip"])
        self.CHORD          = float(p["chord"])
        self.RHO            = float(p["rho"])
        self.OSWALD         = float(p["oswald_eff"])
        self.CD0            = float(p["CD0"])
        self.CL0            = float(p["CL0"])
        self.CL_ALPHA       = float(p["CL_alpha"])
        self.AOA_LIMIT      = float(p["aoa_limit"])
        self.ramp_time      = float(ramp_time)
        self.k_drive_spin   = float(p["k_drive_spin"])
        self.k_drag_spin    = float(p["k_drag_spin"])
        self.pitch_gain_rad = float(p["pitch_gain_rad"])

        span = self.R_TIP - self.R_ROOT
        self.AR        = float(p["aspect_ratio"])
        self.R_CP      = self.R_ROOT + (2.0 / 3.0) * span
        self.disk_area = math.pi * self.R_TIP ** 2

        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )

        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        self._last_t = float(state_dict["last_t"]) if state_dict else -1.0

        # Diagnostics — same names as PetersHeBEM / GlauertBEM
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_Q_drive        = 0.0
        self.last_Q_drag         = 0.0
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_H_force        = 0.0
        self.last_F_prandtl      = 1.0
        self.last_v1c            = 0.0   # no harmonic states; zero always
        self.last_v1s            = 0.0
        self.last_tau0           = 0.0   # no ODE time constant
        self.last_skew_angle_deg = 0.0

    def to_dict(self) -> dict:
        return {"type": "OpenFASTBEM", "last_t": self._last_t}

    @classmethod
    def from_definition(cls, defn, state_dict: dict | None = None,
                        **overrides) -> "OpenFASTBEM":
        return cls(defn, state_dict=state_dict, **overrides)

    def is_valid(self) -> bool:
        return math.isfinite(self.last_T)

    def _ramp_factor(self, t: float) -> float:
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _strip_forces(
        self,
        v_rel_world: np.ndarray,
        disk_normal: np.ndarray,
        R_hub: np.ndarray,
        omega_rotor: float,
        collective_rad: float,
        tilt_lon_rad: float,
        tilt_lat_rad: float,
        spin_angle: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Strip integration: BEM phi-solve at each (azimuth, radial) element.

        At each element:
          Vx = free-stream axial velocity (BEM solves for a, so no subtraction here)
          Vy = omega·r  (rotational velocity at this radius)
          phi solved → a, a'
          dFn = ½ρ·Vrel²·Cn·chord·dr·F  (normal/thrust direction)
          dFt = ½ρ·Vrel²·Ct·chord·dr·F  (tangential/torque direction)
        """
        omega_abs = abs(float(omega_rotor))
        F_acc = np.zeros(3)
        M_acc = np.zeros(3)
        F_prandtl_sum = 0.0

        # Vx is identical for all azimuths (uniform inflow assumption in BEM)
        v_axial = float(np.dot(v_rel_world, disk_normal))

        for i in range(self.N_AB):
            phi_az = spin_angle + self._phi_offsets[i]
            ca  = math.cos(phi_az)
            sa  = math.sin(phi_az)

            pitch = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
            cp_k  = math.cos(pitch)
            sp_k  = math.sin(pitch)

            e_span_body   = np.array([ ca,        sa,        0.0])
            e_chord_body  = np.array([-sa * cp_k, ca * cp_k, sp_k])
            e_normal_body = np.array([ sa * sp_k, -ca * sp_k, cp_k])  # noqa: F841

            e_span_world = R_hub @ e_span_body
            e_tang       = np.cross(disk_normal, e_span_world)  # direction of blade motion

            for j in range(self.N_RADIAL):
                r = self._r_stations[j]

                Vy_elem = omega_abs * r
                Vx_elem = v_axial

                if abs(Vx_elem) < 0.001 and Vy_elem < 0.001:
                    continue

                sigma_p = (self.N_BLADES * self.CHORD) / (2.0 * math.pi * r)

                phi_sol, a, ap = _solve_phi_brent(
                    Vx_elem, Vy_elem, sigma_p, pitch,
                    self.N_BLADES, r, self.R_TIP, self.R_ROOT,
                    self.CL0, self.CL_ALPHA, self.CD0, self.AR,
                    self.OSWALD, self.AOA_LIMIT,
                )

                # Accumulate representative Prandtl factor (first azimuth only)
                if i == 0:
                    F_prandtl_sum += _prandtl_F(
                        self.N_BLADES, r, self.R_TIP, self.R_ROOT, phi_sol
                    )

                a  = max(-1.0, min(1.5, a))
                ap = max(-1.0, min(1.0, ap))

                ua_axial  = Vx_elem * (1.0 - a)
                ua_tang   = Vy_elem * (1.0 + ap)
                Vrel_sq   = ua_axial * ua_axial + ua_tang * ua_tang
                if Vrel_sq < 0.25:
                    continue

                # Airfoil coefficients at the solved phi
                alpha = max(-self.AOA_LIMIT,
                            min(self.AOA_LIMIT, phi_sol - pitch))
                CL = self.CL0 + self.CL_ALPHA * alpha
                CD = self.CD0 + CL * CL / (math.pi * self.AR * self.OSWALD)

                F_loss = _prandtl_F(self.N_BLADES, r, self.R_TIP, self.R_ROOT, phi_sol)
                q_elem = 0.5 * self.RHO * Vrel_sq * self.CHORD * self._dr * F_loss

                # Normal (thrust) and tangential (torque) force coefficients
                Cn_sol = CL * math.cos(phi_sol) + CD * math.sin(phi_sol)
                Ct_sol = CL * math.sin(phi_sol) - CD * math.cos(phi_sol)

                dFn = q_elem * Cn_sol   # along disk_normal
                dFt = q_elem * Ct_sol   # along e_tang (blade-motion direction)

                F_strip = dFn * disk_normal + dFt * e_tang
                r_cp    = r * e_span_world

                F_acc += F_strip
                M_acc += np.cross(r_cp, F_strip)

        if self.N_RADIAL > 0:
            self.last_F_prandtl = F_prandtl_sum / self.N_RADIAL

        return F_acc, M_acc

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

        disk_normal  = R_hub[:, 2]
        omega_abs    = abs(float(omega_rotor))
        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        F_acc, M_acc = self._strip_forces(
            v_rel_world, disk_normal, R_hub, omega_rotor,
            collective_rad, tilt_lon_rad, tilt_lat_rad, spin_angle,
        )

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        T_disk = float(np.dot(F_total, disk_normal))

        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # Disk-average induced velocity from actuator-disk momentum theory (diagnostic)
        V_T = math.sqrt(v_inplane ** 2 + v_axial ** 2 + 1e-6)
        v_i = T_disk / (2.0 * self.RHO * self.disk_area * max(V_T, 0.1))

        chi = math.atan2(v_inplane, max(abs(v_axial + v_i), 0.01))

        self.last_T              = T_disk
        self.last_v_axial        = v_axial
        self.last_v_i            = v_i
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_Q_drive        = float(self.k_drive_spin * v_inplane)
        self.last_Q_drag         = float(self.k_drag_spin * omega_abs ** 2)
        self.last_M_spin         = np.array(M_spin_world)
        self.last_M_cyc          = np.array(M_cyc_world)
        self.last_H_force        = float(np.linalg.norm(F_total - T_disk * disk_normal))
        self.last_skew_angle_deg = math.degrees(chi)
        self._last_t             = t

        log.debug(
            "t=%.3f T=%.2fN v_i=%.3f v_ax=%.2f v_ip=%.2f a_disk=%.3f",
            t, T_disk, v_i, v_axial, v_inplane,
            v_i / max(V_T, 0.1),
        )

        return AeroResult(
            F_world   = np.array(F_total),
            M_orbital = np.array(M_cyc_world),
            Q_spin    = Q_spin_scalar,
            M_spin    = np.array(M_spin_world),
        )
