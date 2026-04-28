"""
aero_skewed_wake2_jit.py — Numba JIT-compiled SkewedWakeBEM2.

Two @njit kernels:
  _jit2_strip_pass       One strip integration pass (N_AB x N_RADIAL).
                         Called inside the convergence loop.
  _jit2_inflow_and_strip Iterates v0 to quasi-static Coleman fixed point,
                         returns final (F, M, v0, F_prandtl_mean).

The convergence loop lives inside the JIT kernel so v0 updates don't
cross the Python/JIT boundary on every iteration.
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
from aero_skewed_wake2 import SkewedWakeBEM2

log = logging.getLogger(__name__)

_KX_MAX_F = math.tan(math.radians(85.0))


# ---------------------------------------------------------------------------
# JIT kernel 1 — one strip integration pass
# ---------------------------------------------------------------------------

@njit(cache=True, fastmath=True)
def _jit2_strip_pass(
    phi_offsets,   # (N_AB,)
    r_stations,    # (N_RADIAL,)
    r_norm,        # (N_RADIAL,)  r / R_tip
    R_hub,         # (3, 3)
    collective_rad, tilt_lon_rad, tilt_lat_rad,
    omega_rotor, spin_angle, psi_skew,
    v0, v1c,
    vb0, vb1, vb2,   # v_rel_world
    dn0, dn1, dn2,   # disk_normal
    v_ax_eff, omega_abs,
    CL0, CL_ALPHA, CD0, AOA_LIMIT, AR_pi_e, RHO, S_elem,
    N_BLADES_f, R_TIP, R_ROOT_ref,
):
    """One (N_AB x N_RADIAL) strip pass. Returns (Fx,Fy,Fz, Mx,My,Mz, F_prandtl_mean)."""
    N_AB     = phi_offsets.shape[0]
    N_RADIAL = r_stations.shape[0]
    Fx = 0.0; Fy = 0.0; Fz = 0.0
    Mx = 0.0; My = 0.0; Mz = 0.0

    # Pre-compute Prandtl tip/root loss per radial strip (azimuth-independent).
    F_prandtl     = np.empty(N_RADIAL)
    F_prandtl_sum = 0.0
    half_blades   = N_BLADES_f * 0.5
    for j in range(N_RADIAL):
        r_j       = r_stations[j]
        v_tan_j   = max(omega_abs * r_j, 0.5)
        phi_j     = math.atan2(v_ax_eff, v_tan_j)
        sin_phi_j = max(abs(math.sin(phi_j)), 1e-4)
        f_tip  = half_blades * (R_TIP    - r_j) / (r_j        * sin_phi_j)
        f_root = half_blades * (r_j - R_ROOT_ref) / (R_ROOT_ref * sin_phi_j)
        F_j = max(0.01,
                  (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_tip))))
                  * (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_root)))))
        F_prandtl[j]   = F_j
        F_prandtl_sum += F_j

    for i in range(N_AB):
        phi_az = spin_angle + phi_offsets[i]
        ca     = math.cos(phi_az)
        sa     = math.sin(phi_az)

        p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
        cp_k = math.cos(p_k)
        sp_k = math.sin(p_k)

        # Blade vectors in body frame
        esb0 =  ca;        esb1 =  sa;        esb2 = 0.0
        ecb0 = -sa * cp_k; ecb1 =  ca * cp_k; ecb2 = sp_k
        enb0 =  sa * sp_k; enb1 = -ca * sp_k; enb2 = cp_k

        # Rotate to world frame
        esw0 = R_hub[0,0]*esb0 + R_hub[0,1]*esb1 + R_hub[0,2]*esb2
        esw1 = R_hub[1,0]*esb0 + R_hub[1,1]*esb1 + R_hub[1,2]*esb2
        esw2 = R_hub[2,0]*esb0 + R_hub[2,1]*esb1 + R_hub[2,2]*esb2

        ecw0 = R_hub[0,0]*ecb0 + R_hub[0,1]*ecb1 + R_hub[0,2]*ecb2
        ecw1 = R_hub[1,0]*ecb0 + R_hub[1,1]*ecb1 + R_hub[1,2]*ecb2
        ecw2 = R_hub[2,0]*ecb0 + R_hub[2,1]*ecb1 + R_hub[2,2]*ecb2

        enw0 = R_hub[0,0]*enb0 + R_hub[0,1]*enb1 + R_hub[0,2]*enb2
        enw1 = R_hub[1,0]*enb0 + R_hub[1,1]*enb1 + R_hub[1,2]*enb2
        enw2 = R_hub[2,0]*enb0 + R_hub[2,1]*enb1 + R_hub[2,2]*enb2

        # e_tang = disk_normal x e_span
        etw0 = dn1*esw2 - dn2*esw1
        etw1 = dn2*esw0 - dn0*esw2
        etw2 = dn0*esw1 - dn1*esw0

        # Coleman modulation at this azimuth
        cos_skew_i = math.cos(phi_az - psi_skew)

        for j in range(N_RADIAL):
            r_j  = r_stations[j]
            vi_j = v0 + v1c * r_norm[j] * cos_skew_i

            rc0 = r_j * esw0; rc1 = r_j * esw1; rc2 = r_j * esw2

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
            q     = 0.5 * RHO * ua_norm_sq * S_elem * F_prandtl[j]

            lr0 = ua1*esw2 - ua2*esw1
            lr1 = ua2*esw0 - ua0*esw2
            lr2 = ua0*esw1 - ua1*esw0
            ln  = math.sqrt(lr0*lr0 + lr1*lr1 + lr2*lr2)
            if ln < 1e-9:
                continue
            inv_ln = 1.0 / ln
            inv_ua = 1.0 / ua_norm

            fs0 = q * (CL_ * lr0 * inv_ln + CD_ * ua0 * inv_ua)
            fs1 = q * (CL_ * lr1 * inv_ln + CD_ * ua1 * inv_ua)
            fs2 = q * (CL_ * lr2 * inv_ln + CD_ * ua2 * inv_ua)

            Fx += fs0; Fy += fs1; Fz += fs2
            Mx += rc1*fs2 - rc2*fs1
            My += rc2*fs0 - rc0*fs2
            Mz += rc0*fs1 - rc1*fs0

    return Fx, Fy, Fz, Mx, My, Mz, F_prandtl_sum / N_RADIAL


# ---------------------------------------------------------------------------
# JIT kernel 2 — convergence loop + final strip pass
# ---------------------------------------------------------------------------

@njit(cache=True, fastmath=True)
def _jit2_inflow_and_strip(
    phi_offsets, r_stations, r_norm,
    R_hub,
    collective_rad, tilt_lon_rad, tilt_lat_rad,
    omega_rotor, spin_angle, ramp,
    psi_skew,
    vb0, vb1, vb2,
    dn0, dn1, dn2,
    v_axial, v_inplane,
    v0_init, cold,        # warm-start value; cold=1 on first call
    rho_A,                # RHO * disk_area
    CL0, CL_ALPHA, CD0, AOA_LIMIT, AR_pi_e, RHO, S_elem,
    N_AZ_f, N_BLADES_f, R_TIP, R_ROOT_ref, omega_abs,
    KX_MAX_f,
):
    """Iterate v0 to Coleman quasi-static fixed point, return (F, M, v0, F_prandtl_mean)."""
    n_iter      = 60 if cold else 30
    alpha_relax = 0.10 if cold else 0.25

    v0 = v0_init

    # Bootstrap: one strip pass at v0=0 to escape the V_T=0 singularity.
    if v0 < 1e-3:
        Fx0, Fy0, Fz0, _, _, _, _ = _jit2_strip_pass(
            phi_offsets, r_stations, r_norm, R_hub,
            collective_rad, tilt_lon_rad, tilt_lat_rad,
            omega_rotor, spin_angle, psi_skew,
            0.0, 0.0,
            vb0, vb1, vb2, dn0, dn1, dn2,
            abs(v_axial), omega_abs,
            CL0, CL_ALPHA, CD0, AOA_LIMIT, AR_pi_e, RHO, S_elem,
            N_BLADES_f, R_TIP, R_ROOT_ref,
        )
        T0 = (Fx0*dn0 + Fy0*dn1 + Fz0*dn2) / N_AZ_f
        v0 = math.sqrt(max(abs(T0), 0.1) / (2.0 * rho_A))

    Fx = 0.0; Fy = 0.0; Fz = 0.0
    Mx = 0.0; My = 0.0; Mz = 0.0
    F_prandtl_mean = 1.0

    for _ in range(n_iter):
        chi = math.atan2(v_inplane, max(abs(v_axial + v0), 0.01))
        K_x = min(math.tan(chi / 2.0), KX_MAX_f)
        v1c = K_x * v0

        v_ax_eff = abs(v_axial + v0)

        Fx, Fy, Fz, Mx, My, Mz, F_prandtl_mean = _jit2_strip_pass(
            phi_offsets, r_stations, r_norm, R_hub,
            collective_rad, tilt_lon_rad, tilt_lat_rad,
            omega_rotor, spin_angle, psi_skew,
            v0, v1c,
            vb0, vb1, vb2, dn0, dn1, dn2,
            v_ax_eff, omega_abs,
            CL0, CL_ALPHA, CD0, AOA_LIMIT, AR_pi_e, RHO, S_elem,
            N_BLADES_f, R_TIP, R_ROOT_ref,
        )

        T_raw = (Fx*dn0 + Fy*dn1 + Fz*dn2) / N_AZ_f
        V_T   = math.sqrt(v_inplane*v_inplane + (v_axial + v0)**2 + 1e-6)
        v0_ss = max(0.0, T_raw) / (2.0 * rho_A * max(V_T, 0.1))

        err   = abs(v0_ss - v0)
        v0   += alpha_relax * (v0_ss - v0)
        if err < 1e-4:
            break

    s     = ramp / N_AZ_f
    F_out = np.empty(3); F_out[0] = Fx*s; F_out[1] = Fy*s; F_out[2] = Fz*s
    M_out = np.empty(3); M_out[0] = Mx*s; M_out[1] = My*s; M_out[2] = Mz*s

    return F_out, M_out, v0, F_prandtl_mean


# ---------------------------------------------------------------------------
# Class
# ---------------------------------------------------------------------------

class SkewedWakeBEM2Jit(SkewedWakeBEM2):
    """
    Drop-in replacement for SkewedWakeBEM2 backed by numba @njit kernels.

    The entire inflow convergence loop + strip integration runs inside a
    single JIT kernel so v0 state updates never cross the Python boundary.
    """

    def __init__(self, rotor, ramp_time: float = 5.0,
                 state_dict=None, **overrides):
        super().__init__(rotor, ramp_time, state_dict=state_dict, **overrides)
        self._AR_pi_e = math.pi * self.AR * self.OSWALD
        self._rho_A   = self.RHO * self.disk_area
        self._warmup()

    def _warmup(self):
        log.info("SkewedWakeBEM2Jit: compiling JIT kernels (first run only)...")
        dummy_phi = self._phi_offsets
        dummy_r   = self._r_stations
        dummy_rn  = self._r_stations / self.R_TIP
        dummy_R   = np.eye(3)
        _jit2_strip_pass(
            dummy_phi, dummy_r, dummy_rn, dummy_R,
            0.05, 0.0, 0.0, 3.0, 0.0, 0.0,
            0.1, 0.03,
            0.0, 5.0, 0.0, 0.0, 0.0, 1.0,
            1.0, 3.0,
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem),
            float(self.N_BLADES), self.R_TIP, max(self.R_ROOT, 0.01),
        )
        _jit2_inflow_and_strip(
            dummy_phi, dummy_r, dummy_rn, dummy_R,
            0.05, 0.0, 0.0, 3.0, 0.0, 1.0,
            0.0,
            0.0, 5.0, 0.0, 0.0, 0.0, 1.0,
            2.0, 4.0,
            0.1, 1,
            self._rho_A,
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem),
            float(self.N_AZ), float(self.N_BLADES),
            self.R_TIP, max(self.R_ROOT, 0.01), 3.0,
            _KX_MAX_F,
        )
        log.info("SkewedWakeBEM2Jit: JIT compilation done.")

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

        if v_inplane > 0.01:
            v_ip_unit = v_inplane_vec / v_inplane
            psi_skew  = math.atan2(float(np.dot(v_ip_unit, R_hub[:, 1])),
                                   float(np.dot(v_ip_unit, R_hub[:, 0])))
        else:
            psi_skew = 0.0

        cold   = 1 if self._last_t < 0.0 else 0
        R_hub_c = R_hub if (R_hub.dtype == np.float64 and R_hub.flags['C_CONTIGUOUS']) \
                        else np.ascontiguousarray(R_hub, dtype=np.float64)
        vb = v_rel_world
        dn = disk_normal

        F_total, M_total, v0_new, F_prandtl_mean = _jit2_inflow_and_strip(
            self._phi_offsets, self._r_stations, self._r_stations / self.R_TIP,
            R_hub_c,
            float(collective_rad), float(tilt_lon_rad), float(tilt_lat_rad),
            float(omega_rotor), float(spin_angle), float(ramp),
            float(psi_skew),
            float(vb[0]), float(vb[1]), float(vb[2]),
            float(dn[0]), float(dn[1]), float(dn[2]),
            float(v_axial), float(v_inplane),
            float(self._v0), cold,
            float(self._rho_A),
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem),
            float(self.N_AZ), float(self.N_BLADES),
            self.R_TIP, max(self.R_ROOT, 0.01), float(omega_abs),
            _KX_MAX_F,
        )

        self._v0     = float(v0_new)
        self._last_t = t
        self.last_F_prandtl = float(F_prandtl_mean)

        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        chi = math.atan2(v_inplane, max(abs(v_axial + self._v0), 0.01))

        self.last_T              = float(np.dot(F_total, disk_normal))
        self.last_v_axial        = v_axial
        self.last_v_i            = self._v0
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_M_spin         = np.array(M_spin_world)
        self.last_M_cyc          = np.array(M_cyc_world)
        self.last_H_force        = float(np.linalg.norm(F_total - self.last_T * disk_normal))
        self.last_skew_angle_deg = math.degrees(chi)

        log.debug("t=%.3f T=%.2fN v0=%.3f chi=%.1f F_prandtl=%.3f",
                  t, self.last_T, self._v0, self.last_skew_angle_deg, F_prandtl_mean)

        return AeroResult(
            F_world   = np.array(F_total),
            M_orbital = np.array(M_cyc_world),
            Q_spin    = Q_spin_scalar,
            M_spin    = np.array(M_spin_world),
        )
