"""
aero_skewed_wake_jit.py  —  Numba JIT-compiled SkewedWakeBEM.

Identical physics to SkewedWakeBEM.  The (N_AB x N_RADIAL) strip loop is
compiled to native SIMD code via numba @njit, eliminating all intermediate
array allocations and Python dispatch overhead.

Two JIT kernels, both cached to __pycache__ so only the very first import
triggers LLVM compilation (~1-2 s):

  _jit_vi0          3-iteration Newton bootstrap for uniform v_i0.
  _jit_strip_loop   Per-blade azimuth x radial strip accumulation.

Usage
-----
    from aero.aero_skewed_wake_jit import SkewedWakeBEMJit
    bem = SkewedWakeBEMJit(rotor)          # compiles once, then cached
    result = bem.compute_forces(...)       # same interface as SkewedWakeBEM
"""

import math
import logging
import sys
import os
import numpy as np
from numba import njit

_AERO_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR  = os.path.dirname(_AERO_DIR)
for _p in (_SIM_DIR, _AERO_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aero import AeroResult
from aero_skewed_wake import SkewedWakeBEM

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# JIT kernel 1 — uniform induced velocity bootstrap (3-iteration Newton)
# ---------------------------------------------------------------------------

@njit(cache=True, fastmath=True)
def _jit_vi0(
    v_axial, omega_abs, R_CP, S_blade, N_BLADES_f, RHO, disk_area,
    CL0, CL_ALPHA, CD0, AR_pi_e, AOA_LIMIT, collective_rad,
):
    v_i0     = 0.0
    v_ax_abs = abs(v_axial)
    for _ in range(3):
        v_sum    = v_axial + v_i0
        v_loc_sq = (omega_abs * R_CP) ** 2 + v_sum ** 2
        if v_loc_sq > 0.25:
            inflow = math.atan2(v_sum, omega_abs * R_CP)
            aoa0   = max(-AOA_LIMIT, min(AOA_LIMIT, inflow + collective_rad))
            CL_    = CL0 + CL_ALPHA * aoa0
            CD_    = CD0 + CL_ * CL_ / AR_pi_e
            T_est  = max(0.0, N_BLADES_f * 0.5 * RHO * v_loc_sq * S_blade
                         * (CL_ * math.cos(inflow) - CD_ * math.sin(inflow)))
            disc   = v_ax_abs * v_ax_abs + 2.0 * max(T_est, 0.01) / (RHO * disk_area)
            v_i0   = max(0.0, (-v_ax_abs + math.sqrt(disc)) / 2.0)
    return v_i0


# ---------------------------------------------------------------------------
# JIT kernel 2 — N_AB x N_RADIAL strip accumulation
# ---------------------------------------------------------------------------

@njit(cache=True, fastmath=True)
def _jit_strip_loop(
    phi_offsets,   # (N_AB,)   blade azimuth phase offsets
    r_stations,    # (N_RADIAL,) radial centres
    r_norm,        # (N_RADIAL,) r/R_tip
    F_per_strip,   # (N_RADIAL,) Prandtl F, pre-computed
    R_hub,         # (3, 3)    hub rotation matrix (body -> world)
    # scalars
    collective_rad, tilt_lon_rad, tilt_lat_rad,
    omega_rotor,    # signed [rad/s]
    spin_angle,     # current rotor spin angle [rad]
    ramp,           # start-up ramp factor [0..1]
    v_i0, K_skew, psi_skew,
    vb0, vb1, vb2,  # v_base = wind - v_hub  (world NED)
    dn0, dn1, dn2,  # disk_normal             (world NED)
    CL0, CL_ALPHA, CD0, AOA_LIMIT, AR_pi_e, RHO, S_elem,
    N_AZ,
):
    N_AB     = phi_offsets.shape[0]
    N_RADIAL = r_stations.shape[0]
    Fx = 0.0; Fy = 0.0; Fz = 0.0
    Mx = 0.0; My = 0.0; Mz = 0.0

    for i in range(N_AB):
        phi_az = spin_angle + phi_offsets[i]
        ca     = math.cos(phi_az)
        sa     = math.sin(phi_az)

        p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
        cp_k = math.cos(p_k)
        sp_k = math.sin(p_k)

        # Blade unit vectors in body frame
        esb0 =  ca;        esb1 =  sa;       esb2 = 0.0
        ecb0 = -sa * cp_k; ecb1 =  ca * cp_k; ecb2 = sp_k
        enb0 =  sa * sp_k; enb1 = -ca * sp_k; enb2 = cp_k

        # Rotate to world frame:  v_w = R_hub @ v_b
        esw0 = R_hub[0,0]*esb0 + R_hub[0,1]*esb1 + R_hub[0,2]*esb2
        esw1 = R_hub[1,0]*esb0 + R_hub[1,1]*esb1 + R_hub[1,2]*esb2
        esw2 = R_hub[2,0]*esb0 + R_hub[2,1]*esb1 + R_hub[2,2]*esb2

        ecw0 = R_hub[0,0]*ecb0 + R_hub[0,1]*ecb1 + R_hub[0,2]*ecb2
        ecw1 = R_hub[1,0]*ecb0 + R_hub[1,1]*ecb1 + R_hub[1,2]*ecb2
        ecw2 = R_hub[2,0]*ecb0 + R_hub[2,1]*ecb1 + R_hub[2,2]*ecb2

        enw0 = R_hub[0,0]*enb0 + R_hub[0,1]*enb1 + R_hub[0,2]*enb2
        enw1 = R_hub[1,0]*enb0 + R_hub[1,1]*enb1 + R_hub[1,2]*enb2
        enw2 = R_hub[2,0]*enb0 + R_hub[2,1]*enb1 + R_hub[2,2]*enb2

        # e_tang = disk_normal x e_span_world
        etw0 = dn1*esw2 - dn2*esw1
        etw1 = dn2*esw0 - dn0*esw2
        etw2 = dn0*esw1 - dn1*esw0

        # Coleman modulation for this azimuth
        cos_skew_i = math.cos(phi_az - psi_skew)

        for j in range(N_RADIAL):
            r_j  = r_stations[j]
            vi_j = v_i0 * (1.0 + K_skew * r_norm[j] * cos_skew_i)

            # Strip centre position: r_cp = r_j * e_span_world
            rc0 = r_j * esw0; rc1 = r_j * esw1; rc2 = r_j * esw2

            # Apparent wind:  ua = v_base - omega*r*e_tang - vi*disk_normal
            ua0 = vb0 - omega_rotor * r_j * etw0 - vi_j * dn0
            ua1 = vb1 - omega_rotor * r_j * etw1 - vi_j * dn1
            ua2 = vb2 - omega_rotor * r_j * etw2 - vi_j * dn2

            ua_norm_sq = ua0*ua0 + ua1*ua1 + ua2*ua2
            if ua_norm_sq < 0.25:
                continue
            ua_norm = math.sqrt(ua_norm_sq)

            ua_chord = ua0*ecw0 + ua1*ecw1 + ua2*ecw2
            if abs(ua_chord) < 1e-6:
                continue
            ua_normal = ua0*enw0 + ua1*enw1 + ua2*enw2

            alpha = max(-AOA_LIMIT, min(AOA_LIMIT, -ua_normal / ua_chord))
            CL_   = CL0 + CL_ALPHA * alpha
            CD_   = CD0 + CL_ * CL_ / AR_pi_e
            q     = 0.5 * RHO * ua_norm_sq * S_elem * F_per_strip[j]

            # Lift direction:  e_lift = (ua x e_span_world) / |...|
            lr0 = ua1*esw2 - ua2*esw1
            lr1 = ua2*esw0 - ua0*esw2
            lr2 = ua0*esw1 - ua1*esw0
            ln  = math.sqrt(lr0*lr0 + lr1*lr1 + lr2*lr2)
            if ln < 1e-9:
                continue
            inv_ln = 1.0 / ln
            inv_ua = 1.0 / ua_norm

            # F_strip = q * (CL * e_lift + CD * ua_unit)
            fs0 = q * (CL_ * lr0 * inv_ln + CD_ * ua0 * inv_ua)
            fs1 = q * (CL_ * lr1 * inv_ln + CD_ * ua1 * inv_ua)
            fs2 = q * (CL_ * lr2 * inv_ln + CD_ * ua2 * inv_ua)

            Fx += fs0; Fy += fs1; Fz += fs2

            # M_strip = r_cp x F_strip
            Mx += rc1*fs2 - rc2*fs1
            My += rc2*fs0 - rc0*fs2
            Mz += rc0*fs1 - rc1*fs0

    s = ramp / N_AZ
    F_out = np.empty(3); F_out[0] = Fx*s; F_out[1] = Fy*s; F_out[2] = Fz*s
    M_out = np.empty(3); M_out[0] = Mx*s; M_out[1] = My*s; M_out[2] = Mz*s
    return F_out, M_out


# ---------------------------------------------------------------------------
# Class — same public interface as SkewedWakeBEM
# ---------------------------------------------------------------------------

class SkewedWakeBEMJit(SkewedWakeBEM):
    """
    Drop-in replacement for SkewedWakeBEM backed by numba @njit kernels.

    Construction triggers JIT compilation (~1-2 s on first run; cached
    thereafter via numba's __pycache__ mechanism).
    """

    def __init__(self, rotor, ramp_time: float = 5.0, **overrides):
        super().__init__(rotor, ramp_time, **overrides)
        self._AR_pi_e = math.pi * self.AR * self.OSWALD
        self._warmup()

    def _warmup(self):
        """Force JIT compilation on dummy inputs so first real call is fast."""
        log.info("SkewedWakeBEMJit: compiling JIT kernels (first run only)...")
        _jit_vi0(0.0, 3.0, self.R_CP, self.S_blade, float(self.N_BLADES),
                 self.RHO, self.disk_area, self.CL0, self.CL_ALPHA, self.CD0,
                 self._AR_pi_e, self.AOA_LIMIT, 0.05)
        _jit_strip_loop(
            self._phi_offsets, self._r_stations, self._r_norm,
            np.ones(self.N_RADIAL), np.eye(3),
            0.05, 0.0, 0.0, 3.0, 0.0, 1.0,
            0.1, 0.3, 0.0,
            0.0, 5.0, 0.0,
            0.0, 0.0, 1.0,
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem), self.N_AZ,
        )
        log.info("SkewedWakeBEMJit: JIT compilation done.")

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

        disk_normal = R_hub[:, 2]
        omega_abs   = abs(float(omega_rotor))

        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # -- Uniform induced velocity (JIT) -----------------------------------
        v_i0 = float(_jit_vi0(
            v_axial, omega_abs, self.R_CP, self.S_blade, float(self.N_BLADES),
            self.RHO, self.disk_area, self.CL0, self.CL_ALPHA, self.CD0,
            self._AR_pi_e, self.AOA_LIMIT, collective_rad,
        ))

        # -- Coleman skew (cheap scalar Python) --------------------------------
        K_skew, chi = self._coleman_skew(v_inplane, v_axial + v_i0)

        if v_inplane > 0.01:
            v_ip_unit = v_inplane_vec / v_inplane
            _ep = np.array([0.0, 1.0, 0.0])   # NED East
            _ep = _ep - np.dot(_ep, disk_normal) * disk_normal
            if np.linalg.norm(_ep) < 1e-6:
                _ep = np.array([1.0, 0.0, 0.0])   # NED North fallback
                _ep = _ep - np.dot(_ep, disk_normal) * disk_normal
            _bx      = _ep / np.linalg.norm(_ep)
            _by      = np.cross(disk_normal, _bx)
            psi_skew = math.atan2(float(np.dot(v_ip_unit, _by)),
                                  float(np.dot(v_ip_unit, _bx)))
        else:
            psi_skew = 0.0

        # -- Prandtl F per radial strip (vectorized numpy, N_RADIAL=10) --------
        v_ax_eff    = abs(v_axial + v_i0)
        v_tan_j     = np.maximum(omega_abs * self._r_stations, 0.5)
        phi_j       = np.arctan2(v_ax_eff, v_tan_j)
        sin_phi_j   = np.maximum(np.abs(np.sin(phi_j)), 1e-4)
        r_ref       = max(self.R_ROOT, 0.01)
        f_tip       = (self.N_BLADES / 2.0) * (self.R_TIP  - self._r_stations) / (self._r_stations * sin_phi_j)
        f_root      = (self.N_BLADES / 2.0) * (self._r_stations - self.R_ROOT) / (r_ref * sin_phi_j)
        F_per_strip = np.maximum(0.01,
                                 (2.0 / math.pi) * np.arccos(np.minimum(1.0, np.exp(-np.maximum(0.0, f_tip))))
                                 * (2.0 / math.pi) * np.arccos(np.minimum(1.0, np.exp(-np.maximum(0.0, f_root)))))

        self.last_F_prandtl      = float(np.mean(F_per_strip))
        self.last_K_skew         = K_skew
        self.last_skew_angle_deg = math.degrees(chi)

        # -- Main strip loop (JIT) ---------------------------------------------
        dn = disk_normal
        vb = v_rel_world
        F_total, M_total = _jit_strip_loop(
            self._phi_offsets, self._r_stations, self._r_norm, F_per_strip,
            np.ascontiguousarray(R_hub, dtype=np.float64),
            float(collective_rad), float(tilt_lon_rad), float(tilt_lat_rad),
            float(omega_rotor), float(spin_angle), float(ramp),
            float(v_i0), float(K_skew), float(psi_skew),
            float(vb[0]), float(vb[1]), float(vb[2]),
            float(dn[0]), float(dn[1]), float(dn[2]),
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem), self.N_AZ,
        )

        # -- Decompose moment into spin and cyclic components ------------------
        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # -- Diagnostics -------------------------------------------------------
        self.last_T         = float(np.dot(F_total, disk_normal))
        self.last_v_axial   = v_axial
        self.last_v_i       = v_i0
        self.last_v_inplane = v_inplane
        self.last_ramp      = ramp
        self.last_Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs ** 2)
        self.last_M_spin    = M_spin_world.copy()
        self.last_M_cyc     = M_cyc_world.copy()
        self.last_H_force   = float(np.linalg.norm(F_total - self.last_T * disk_normal))

        log.debug("t=%.3f T=%.2fN chi=%.1f K=%.3f F_prandtl=%.3f",
                  t, self.last_T, self.last_skew_angle_deg, K_skew, self.last_F_prandtl)

        return AeroResult(
            F_world   = F_total.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs ** 2),
            M_spin    = M_spin_world.copy(),
        )
