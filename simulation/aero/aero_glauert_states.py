"""
aero_glauert_states.py — BEM with Glauert inflow-state corrections.

Standard BEM momentum theory breaks down in several inflow regimes:
  - Vortex ring state  (v_axial < 0, descent into own wake)
  - Turbulent wake     (v_axial > 0, high induction a > 0.4)
  - Autorotation       (v_axial > 0, near Betz limit a ≈ 1/3)
  - Windmill-brake     (v_axial > 0, a > 1/2, over-induction)

For RAWES the primary regime is the **windmill / autorotation** state (a ∈ [0, 0.5]):
wind drives the rotor, BEM momentum theory applies with the Glauert empirical
correction in the turbulent wake region (a > 0.4).

State detection per strip
-------------------------
  a  = v_i / |v_axial|   (axial induction factor)
  CT = 4·a·(1−a)         — valid for a ≤ 0.4 (windmill, Betz: a=1/3)
  CT = 4·a·(1−¼·(5−3a)·a) — Leishman turbulent-wake fit (0.4 < a < 1.0)
  CT ≥ 1.6 → vortex ring  — empirical ceiling

  For autorotation (RAWES reel-out): a = 1/3 → CT_max = 8/9 ≈ 0.889 (Betz)
  A strip at a < 0 is being powered (helicopter driven rotor) — clamped to 0.

Simplified strip implementation (per-strip scalar loop, no azimuth integration)
so the cyclic moment is computed via the K_cyc empirical approach (same as RotorAero).
The H-force uses the advance-ratio empirical formula (same as RotorAero).

This makes the model *deliberately simpler* in structure than the per-blade azimuth
models, isolating the inflow-state correction as the distinguishing feature.

References
----------
Leishman J.G. (2006) *Principles of Helicopter Aerodynamics*, 2nd ed., §2.13–2.15.
Glauert H. (1926) *A General Theory of the Autogyro*, ARCR&M No. 1111, Appendix.
Johnson W. (1994) *Helicopter Theory*, Dover, §2.6.
Buhl M.L. (2005) *A New Empirical Relationship between Thrust Coefficient and
    Induction Factor for the Turbulent Windmill State*, NREL/TP-500-36834.
"""

import math
import logging
import sys
import os
import numpy as np

# Ensure simulation/ is on sys.path so we can import from aero.py
_SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

from aero import AeroResult

log = logging.getLogger(__name__)

# Induction factor threshold for state transitions
_A_TURBULENT   = 0.4   # above this: turbulent wake correction (Leishman)
_A_VORTEX_RING = 0.9   # above this: vortex ring ceiling (extreme over-induction)


class GlauertStateBEM:
    """
    Strip BEM with Glauert inflow-state corrections for all flight regimes.

    Uses a per-strip scalar loop (not azimuth-integrated) to allow per-strip
    state detection and correction.  Cyclic moment uses the K_cyc phenomenological
    model (same as RotorAero) and H-force uses advance-ratio formula.

    Diagnostic attributes (set after each compute_forces call)
    ----------------------------------------------------------
    last_Q_spin       — empirical spin ODE torque [N·m]
    last_a_mean       — mean axial induction factor across strips
    last_state        — dominant inflow state ('windmill','turbulent','vortex_ring','climb')
    last_CT_bem       — mean BEM thrust coefficient CT = T/(ρ·A·v_eff²)
    last_F_glauert    — mean Glauert correction factor applied
    """

    N_STRIPS = 20

    def __init__(self, rotor, ramp_time: float = 5.0, **overrides):
        p = {**rotor.aero_kwargs(), **overrides}
        self.n_blades     = int(p["n_blades"])
        self.r_root       = float(p["r_root"])
        self.r_tip        = float(p["r_tip"])
        self.chord        = float(p["chord"])
        self.rho          = float(p["rho"])
        self.oswald_eff   = float(p["oswald_eff"])
        self.CD0          = float(p["CD0"])
        self.CL0          = float(p["CL0"])
        self.CL_alpha     = float(p["CL_alpha"])
        self.K_cyc        = float(p["K_cyc"])
        self.aoa_limit    = float(p["aoa_limit"])
        self.ramp_time    = float(ramp_time)
        self.k_drive_spin   = float(p["k_drive_spin"])
        self.k_drag_spin    = float(p["k_drag_spin"])
        self.pitch_gain_rad = float(p["pitch_gain_rad"])

        span             = self.r_tip - self.r_root
        self.aspect_ratio = float(p["aspect_ratio"])
        self.r_cp        = self.r_root + (2.0 / 3.0) * span
        self.disk_area   = math.pi * (self.r_tip ** 2 - self.r_root ** 2)
        self.S_w         = self.n_blades * self.chord * span

        _n  = self.N_STRIPS
        _dr = span / _n
        self._strip_r  = np.array([self.r_root + (i + 0.5) * _dr for i in range(_n)])
        self._strip_dA = self.n_blades * self.chord * _dr

        # Diagnostics
        self.last_T          = 0.0
        self.last_v_axial    = 0.0
        self.last_v_i        = 0.0
        self.last_v_inplane  = 0.0
        self.last_ramp       = 0.0
        self.last_Q_spin     = 0.0
        self.last_M_spin     = np.zeros(3)
        self.last_M_cyc      = np.zeros(3)
        self.last_H_force    = 0.0
        self.last_a_mean     = 0.0
        self.last_state      = "unknown"
        self.last_CT_bem     = 0.0
        self.last_F_glauert  = 1.0

    @classmethod
    def from_definition(cls, defn, **overrides) -> "GlauertStateBEM":
        return cls(defn, **overrides)

    def _ramp_factor(self, t: float) -> float:
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _ct_from_a_glauert(self, a: float) -> float:
        """
        Glauert-corrected CT–a relationship.

        Windmill/autorotation state (a ≤ 0.4):
            CT = 4·a·(1−a)   (standard momentum theory)

        Turbulent wake correction (0.4 < a < 1.0) — Leishman (2006) §2.14:
            CT = 0.889 − (1.6·a − 0.8·a²)/(0.6−0.61·a)  [simplified Glauert]
          → Buhl (2005) NREL fit for 0.4 ≤ a ≤ 1.0:
            CT = (8/9) + (4a − 40/9)·a + (50/9 − 4a)·a²
        """
        if a <= _A_TURBULENT:
            return 4.0 * a * (1.0 - a)
        elif a <= _A_VORTEX_RING:
            # Buhl (2005) NREL empirical correction for turbulent wake
            return (8.0 / 9.0) + (4.0 * a - 40.0 / 9.0) * a + (50.0 / 9.0 - 4.0 * a) * a ** 2
        else:
            # Vortex ring / extreme over-induction — cap CT
            return 2.0

    def _a_from_ct_glauert(self, CT: float) -> float:
        """
        Invert CT(a) using bisection.

        For windmill state (0 ≤ CT ≤ 8/9): direct quadratic from CT=4a(1-a)
          a = (1 − √(1 − CT)) / 2  ... for CT ≤ 8/9

        For turbulent wake (CT > 8/9): solve Buhl fit numerically.
        """
        if CT <= 0:
            return 0.0
        if CT <= 8.0 / 9.0:
            # Standard momentum: CT = 4a(1-a) → a = (1 - sqrt(1-CT))/2
            return (1.0 - math.sqrt(max(0.0, 1.0 - CT))) / 2.0
        # Turbulent wake: bisect for a in [0.4, 0.95]
        lo, hi = _A_TURBULENT, _A_VORTEX_RING
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self._ct_from_a_glauert(mid) < CT:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def _bem_strip_glauert(self, r_i: float, v_axial: float, omega_abs: float,
                           collective_rad: float) -> tuple:
        """
        Per-strip BEM with Glauert inflow-state correction.

        Returns (dT, dQ_drive, dQ_drag, a, F_glauert) where:
          dT       — strip thrust [N]
          dQ_drive — positive (driving) torque [N·m]
          dQ_drag  — negative (braking) torque [N·m]
          a        — induction factor at this strip
          F_glauert— correction factor applied
        """
        v_tan = omega_abs * r_i
        if v_tan < 0.1 and abs(v_axial) < 0.1:
            return 0.0, 0.0, 0.0, 0.0, 1.0

        # ── Initial guess: standard BEM without correction ────────────────────
        v_eff  = math.sqrt(v_tan ** 2 + v_axial ** 2)
        phi    = math.atan2(v_axial, v_tan)
        alpha  = max(-self.aoa_limit, min(self.aoa_limit, phi + collective_rad))
        CL     = self.CL0 + self.CL_alpha * alpha
        CD     = self.CD0 + CL ** 2 / (math.pi * self.aspect_ratio * self.oswald_eff)
        q      = 0.5 * self.rho * v_eff ** 2
        dT_uncorr = q * self._strip_dA * (CL * math.cos(phi) - CD * math.sin(phi))

        if abs(v_axial) < 0.1 or dT_uncorr <= 0:
            # Low axial flow — cannot define induction factor; return uncorrected
            dT = max(0.0, dT_uncorr)
            dQ = q * self._strip_dA * r_i * (CL * math.sin(phi) - CD * math.cos(phi))
            return dT, max(0.0, dQ), min(0.0, dQ), 0.0, 1.0

        # ── Glauert CT–a correction  ─────────────────────────────────────────
        # Strip CT using annulus area (2π r dr) and disk velocity v_axial
        A_annulus = 2.0 * math.pi * r_i * (self._strip_dA / (self.n_blades * self.chord))
        CT_uncorr = dT_uncorr / max(0.5 * self.rho * v_axial ** 2 * A_annulus, 1e-6)
        CT_uncorr = max(0.0, CT_uncorr)

        # Find corrected induction factor
        a = self._a_from_ct_glauert(CT_uncorr)
        CT_corrected = self._ct_from_a_glauert(a)

        # Correction factor (ratio of corrected to uncorrected CT)
        if CT_uncorr > 1e-6:
            F_glauert = CT_corrected / CT_uncorr
        else:
            F_glauert = 1.0
        F_glauert = max(0.0, min(2.0, F_glauert))

        # ── Corrected strip forces  ───────────────────────────────────────────
        # Apply induction correction: v_eff with corrected v_i
        v_i    = a * abs(v_axial)
        v_eff2 = math.sqrt(v_tan ** 2 + (v_axial + v_i) ** 2)
        phi2   = math.atan2(v_axial + v_i, v_tan)
        alpha2 = max(-self.aoa_limit, min(self.aoa_limit, phi2 + collective_rad))
        CL2    = self.CL0 + self.CL_alpha * alpha2
        CD2    = self.CD0 + CL2 ** 2 / (math.pi * self.aspect_ratio * self.oswald_eff)
        q2     = 0.5 * self.rho * v_eff2 ** 2

        dT = max(0.0, q2 * self._strip_dA * (CL2 * math.cos(phi2) - CD2 * math.sin(phi2)))
        dQ = q2 * self._strip_dA * r_i * (CL2 * math.sin(phi2) - CD2 * math.cos(phi2))

        return dT, max(0.0, dQ), min(0.0, dQ), a, F_glauert

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
    ) -> AeroResult:
        """
        Compute aerodynamic wrench [Fx,Fy,Fz,Mx,My,Mz] in world NED [N, N·m].

        Strip-based BEM with Glauert inflow-state correction at each strip.
        Cyclic moment via K_cyc empirical model (same as RotorAero).
        H-force via advance-ratio formula.
        """
        ramp = self._ramp_factor(t)

        disk_normal = R_hub[:, 2]
        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))
        omega_abs     = abs(omega_rotor)

        # ── Per-strip Glauert-corrected BEM  ─────────────────────────────────
        T_sum   = 0.0
        Q_pos   = 0.0
        Q_neg   = 0.0
        a_sum   = 0.0
        Fg_sum  = 0.0
        n_valid = 0

        for r_i in self._strip_r:
            dT, dQp, dQn, a_i, Fg_i = self._bem_strip_glauert(
                r_i, v_axial, omega_abs, collective_rad)
            T_sum  += dT
            Q_pos  += dQp
            Q_neg  += dQn
            a_sum  += a_i
            Fg_sum += Fg_i
            n_valid += 1

        if n_valid > 0:
            self.last_a_mean    = a_sum / n_valid
            self.last_F_glauert = Fg_sum / n_valid
        else:
            self.last_a_mean    = 0.0
            self.last_F_glauert = 1.0

        # ── Determine dominant inflow state  ──────────────────────────────────
        a_m = self.last_a_mean
        if v_axial < 0:
            self.last_state = "vortex_ring"
        elif a_m > _A_VORTEX_RING:
            self.last_state = "vortex_ring"
        elif a_m > _A_TURBULENT:
            self.last_state = "turbulent"
        elif a_m > 0.01:
            self.last_state = "windmill"
        else:
            self.last_state = "climb"

        T       = T_sum * ramp
        F_world = T * disk_normal

        # ── H-force (advance-ratio method, same as RotorAero)  ───────────────
        H = 0.0
        if v_inplane > 0.01 and omega_abs > 0.1:
            mu      = v_inplane / (omega_abs * self.r_tip)
            H       = 0.5 * mu * T
            F_world = F_world + H * (v_inplane_vec / v_inplane)

        # ── Spin moment  ──────────────────────────────────────────────────────
        Q_total      = Q_pos + Q_neg
        spin_sign    = float(np.sign(omega_rotor)) if omega_rotor != 0.0 else 1.0
        M_spin_world = Q_total * ramp * spin_sign * disk_normal

        # ── Cyclic moment (K_cyc empirical)  ──────────────────────────────────
        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        # Use hub body frame (R_hub[:,0], R_hub[:,1]) for the cyclic moment so that
        # tilt_lon/tilt_lat are interpreted in the same frame as the blade azimuth
        # convention used by the strip-integration models.
        M_cyc_world = R_hub @ np.array([-self.K_cyc * tilt_lon_rad * T,
                                          self.K_cyc * tilt_lat_rad * T,
                                          0.0])

        M_world = M_spin_world + M_cyc_world

        # ── CT diagnostic  ────────────────────────────────────────────────────
        v_ref = max(abs(v_axial), 0.5)
        self.last_CT_bem = T / max(0.5 * self.rho * v_ref ** 2 * self.disk_area, 1e-6)

        self.last_M_spin     = M_spin_world.copy()
        self.last_M_cyc      = M_cyc_world.copy()
        self.last_T          = T
        self.last_v_axial    = v_axial
        self.last_v_i        = self.last_a_mean * abs(v_axial)
        self.last_v_inplane  = v_inplane
        self.last_ramp       = ramp
        self.last_H_force    = float(H)
        self.last_Q_spin     = float(self.k_drive_spin * v_inplane
                                     - self.k_drag_spin * omega_abs ** 2)

        log.debug("t=%.3f T=%.2fN a=%.3f state=%s F_glauert=%.3f",
                  t, T, self.last_a_mean, self.last_state, self.last_F_glauert)

        return AeroResult(
            F_world   = F_world.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = self.last_Q_spin,
            M_spin    = M_spin_world.copy(),
        )
