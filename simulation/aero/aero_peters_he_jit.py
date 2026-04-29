"""
aero_peters_he_jit.py — Numba JIT-compiled PetersHeBEM.

Identical physics to PetersHeBEM.  The (N_AB x N_RADIAL) strip loop is
compiled to native SIMD code via numba @njit.  Cold start, ODE step, and
diagnostics remain in Python — only the hot loop is JIT-compiled.

The Peters-He induction replaces the Coleman skewed-wake term:
    Coleman:     vi_j = v_i0 * (1 + K_skew * r_norm[j] * cos(phi - psi_skew))
    Peters-He:   vi_j = v0 + v1c * r_norm[j] * cos(phi) + v1s * r_norm[j] * sin(phi)
"""

import math
import logging
import numpy as np
from numba import njit

from . import AeroResult
from .aero_peters_he import PetersHeBEM

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# JIT kernel — N_AB x N_RADIAL strip accumulation with Peters-He induction
# ---------------------------------------------------------------------------

@njit(cache=True, fastmath=True)
def _jit_peters_he_strip_loop(
    phi_offsets,    # (N_AB,)    blade azimuth phase offsets
    r_stations,     # (N_RADIAL,) radial centres
    r_norm,         # (N_RADIAL,) r/R_tip
    R_hub,          # (3, 3)     hub rotation matrix (body -> world)
    # scalars
    collective_rad, tilt_lon_rad, tilt_lat_rad,
    omega_rotor,    # signed [rad/s]
    spin_angle,     # current rotor spin angle [rad]
    ramp,           # start-up ramp factor [0..1]
    v0, v1c, v1s, v2c, v2s,   # Peters-He 5-state inflow [m/s]
    vb0, vb1, vb2,  # v_base = wind - v_hub  (world NED)
    dn0, dn1, dn2,  # disk_normal             (world NED)
    CL0, CL_ALPHA, CD0, AOA_LIMIT, AR_pi_e, RHO, S_elem,
    N_AZ,
    # Prandtl F parameters
    N_BLADES_f, R_TIP, R_ROOT_ref, v_ax_eff, omega_abs,
):
    N_AB     = phi_offsets.shape[0]
    N_RADIAL = r_stations.shape[0]
    Fx = 0.0; Fy = 0.0; Fz = 0.0
    Mx = 0.0; My = 0.0; Mz = 0.0
    M2cx = 0.0; M2sx = 0.0   # 2nd-harmonic disk moment accumulators

    # Prandtl tip/root loss per radial strip (azimuth-independent)
    F_prandtl = np.empty(N_RADIAL)
    F_prandtl_sum = 0.0
    half_blades = N_BLADES_f * 0.5
    for j in range(N_RADIAL):
        r_j       = r_stations[j]
        v_tan_j   = max(omega_abs * r_j, 0.5)
        phi_j     = math.atan2(v_ax_eff, v_tan_j)
        sin_phi_j = max(abs(math.sin(phi_j)), 1e-4)
        f_tip  = half_blades * (R_TIP    - r_j) / (r_j       * sin_phi_j)
        f_root = half_blades * (r_j - R_ROOT_ref) / (R_ROOT_ref * sin_phi_j)
        F_j = max(0.01,
                  (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_tip))))
                  * (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_root)))))
        F_prandtl[j]  = F_j
        F_prandtl_sum += F_j

    for i in range(N_AB):
        phi_az = spin_angle + phi_offsets[i]
        ca     = math.cos(phi_az)
        sa     = math.sin(phi_az)
        cos2   = 2.0 * ca * ca - 1.0   # cos(2ψ)
        sin2   = 2.0 * sa * ca          # sin(2ψ)

        p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
        cp_k = math.cos(p_k)
        sp_k = math.sin(p_k)

        # Blade unit vectors in body frame
        esb0 =  ca;        esb1 =  sa;        esb2 = 0.0
        ecb0 = -sa * cp_k; ecb1 =  ca * cp_k; ecb2 = sp_k
        enb0 =  sa * sp_k; enb1 = -ca * sp_k; enb2 = cp_k

        # Rotate to world frame: v_w = R_hub @ v_b
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

        for j in range(N_RADIAL):
            r_j      = r_stations[j]
            rn_j     = r_norm[j]
            rn_sq_j  = rn_j * rn_j

            # Peters-He 5-state non-uniform induction
            vi_j = (v0
                    + v1c * rn_j * ca  + v1s * rn_j * sa
                    + v2c * rn_sq_j * cos2 + v2s * rn_sq_j * sin2)

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

            # Lift direction: e_lift = (ua x e_span_world) / |...|
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

            # 2nd-harmonic disk-moment accumulators (axial strip force × r²)
            f_axial = fs0*dn0 + fs1*dn1 + fs2*dn2
            M2cx += f_axial * rn_sq_j * cos2
            M2sx += f_axial * rn_sq_j * sin2

    s = ramp / N_AZ
    F_out  = np.empty(3); F_out[0]  = Fx*s;   F_out[1]  = Fy*s;   F_out[2]  = Fz*s
    M_out  = np.empty(3); M_out[0]  = Mx*s;   M_out[1]  = My*s;   M_out[2]  = Mz*s
    return F_out, M_out, M2cx*s, M2sx*s, F_prandtl_sum / N_RADIAL


# ---------------------------------------------------------------------------
# Class — same public interface as PetersHeBEM
# ---------------------------------------------------------------------------

class PetersHeBEMJit(PetersHeBEM):
    """
    Drop-in replacement for PetersHeBEM backed by a numba @njit strip kernel.

    Cold start and ODE step remain in Python; only the N_AB x N_RADIAL
    strip accumulation is compiled.  First instantiation triggers LLVM
    compilation (~1-2 s); cached in __pycache__ thereafter.
    """

    def __init__(self, rotor, ramp_time: float = 5.0,
                 state_dict: dict | None = None):
        super().__init__(rotor, ramp_time, state_dict=state_dict)
        self._r_norm  = self._r_stations / self.R_TIP
        self._AR_pi_e = math.pi * self.AR * self.OSWALD
        self._warmup()

    def _warmup(self):
        """Force JIT compilation on dummy inputs so first real call is fast."""
        log.info("PetersHeBEMJit: compiling JIT kernel (first run only)...")
        _jit_peters_he_strip_loop(
            self._phi_offsets, self._r_stations, self._r_norm,
            np.eye(3),
            0.05, 0.0, 0.0, 3.0, 0.0, 1.0,
            0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 5.0, 0.0,
            0.0, 0.0, 1.0,
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem), self.N_AZ,
            float(self.N_BLADES), self.R_TIP, max(self.R_ROOT, 0.01), 1.0, 3.0,
        )
        log.info("PetersHeBEMJit: JIT compilation done.")

    def _cold_start(
        self,
        v_rel_world: np.ndarray,
        disk_normal: np.ndarray,
        R_hub:       np.ndarray,
        omega_rotor: float,
        collective_rad: float,
        tilt_lon_rad:   float,
        tilt_lat_rad:   float,
        v_axial:    float,
        v_inplane:  float,
    ) -> None:
        """JIT-accelerated cold start — replaces Python _strip_forces with JIT kernel."""
        rho_A     = self.RHO * self.disk_area
        omega_abs = abs(float(omega_rotor))
        R_hub_c   = R_hub if (R_hub.dtype == np.float64 and R_hub.flags['C_CONTIGUOUS']) \
                          else np.ascontiguousarray(R_hub, dtype=np.float64)
        vb = v_rel_world
        dn = disk_normal

        # Step 1: bootstrap v0 from zero-induction strip thrust
        F0, _, _, _, _ = _jit_peters_he_strip_loop(
            self._phi_offsets, self._r_stations, self._r_norm, R_hub_c,
            float(collective_rad), float(tilt_lon_rad), float(tilt_lat_rad),
            float(omega_rotor), 0.0, 1.0,   # spin_angle=0, ramp=1.0
            0.0, 0.0, 0.0, 0.0, 0.0,        # v0=v1c=v1s=v2c=v2s=0
            float(vb[0]), float(vb[1]), float(vb[2]),
            float(dn[0]), float(dn[1]), float(dn[2]),
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem), self.N_AZ,
            float(self.N_BLADES), self.R_TIP, max(self.R_ROOT, 0.01),
            abs(v_axial), float(omega_abs),
        )
        T0 = float(np.dot(F0, dn))   # JIT already divides by N_AZ (ramp=1)
        self._v0 = math.sqrt(max(abs(T0), 0.1) / (2.0 * rho_A))

        # Step 2: iterate all 5 states to fixed point using JIT kernel
        R2 = self.R_TIP ** 2
        for _ in range(60):
            v_ax_eff = abs(v_axial + self._v0)
            F_acc, M_acc, M2c, M2s, _ = _jit_peters_he_strip_loop(
                self._phi_offsets, self._r_stations, self._r_norm, R_hub_c,
                float(collective_rad), float(tilt_lon_rad), float(tilt_lat_rad),
                float(omega_rotor), 0.0, 1.0,
                float(self._v0), float(self._v1c), float(self._v1s),
                float(self._v2c), float(self._v2s),
                float(vb[0]), float(vb[1]), float(vb[2]),
                float(dn[0]), float(dn[1]), float(dn[2]),
                self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
                self._AR_pi_e, self.RHO, float(self._S_elem), self.N_AZ,
                float(self.N_BLADES), self.R_TIP, max(self.R_ROOT, 0.01),
                float(v_ax_eff), float(omega_abs),
            )
            T_raw    = float(np.dot(F_acc, dn))
            Mx_disk  = float(np.dot(M_acc, R_hub_c[:, 0]))
            My_disk  = float(np.dot(M_acc, R_hub_c[:, 1]))
            M2c_disk = float(M2c)
            M2s_disk = float(M2s)

            V_T = math.sqrt(v_inplane**2 + (v_axial + self._v0)**2 + 1e-6)
            V_T = max(V_T, self._v0 * 0.5 + 0.01)

            v0_ss  = T_raw    / (2.0 * rho_A * V_T)
            v1c_ss = My_disk  / (rho_A * self.R_TIP * V_T)
            v1s_ss = -Mx_disk / (rho_A * self.R_TIP * V_T)
            v2c_ss = M2c_disk / (rho_A * R2 * V_T)
            v2s_ss = -M2s_disk / (rho_A * R2 * V_T)

            self._v0  += 0.1 * (v0_ss  - self._v0)
            self._v1c += 0.1 * (v1c_ss - self._v1c)
            self._v1s += 0.1 * (v1s_ss - self._v1s)
            self._v2c += 0.1 * (v2c_ss - self._v2c)
            self._v2s += 0.1 * (v2s_ss - self._v2s)

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

        if self._last_t < 0.0:
            self._cold_start(
                v_rel_world, disk_normal, R_hub, omega_rotor,
                collective_rad, tilt_lon_rad, tilt_lat_rad,
                v_axial, v_inplane,
            )

        v_ax_eff = abs(v_axial + self._v0)

        R_hub_c = R_hub if (R_hub.dtype == np.float64 and R_hub.flags['C_CONTIGUOUS']) \
                       else np.ascontiguousarray(R_hub, dtype=np.float64)
        dn = disk_normal
        vb = v_rel_world

        F_total, M_total, M2c_raw, M2s_raw, F_prandtl_mean = _jit_peters_he_strip_loop(
            self._phi_offsets, self._r_stations, self._r_norm,
            R_hub_c,
            float(collective_rad), float(tilt_lon_rad), float(tilt_lat_rad),
            float(omega_rotor), float(spin_angle), float(ramp),
            float(self._v0), float(self._v1c), float(self._v1s),
            float(self._v2c), float(self._v2s),
            float(vb[0]), float(vb[1]), float(vb[2]),
            float(dn[0]), float(dn[1]), float(dn[2]),
            self.CL0, self.CL_ALPHA, self.CD0, self.AOA_LIMIT,
            self._AR_pi_e, self.RHO, float(self._S_elem), self.N_AZ,
            float(self.N_BLADES), self.R_TIP, max(self.R_ROOT, 0.01),
            float(v_ax_eff), float(omega_abs),
        )
        self.last_F_prandtl = float(F_prandtl_mean)

        # Extract disk-frame moments for ODE forcing
        T_disk   = float(np.dot(F_total, disk_normal))
        Mx_disk  = float(np.dot(M_total, R_hub[:, 0]))
        My_disk  = float(np.dot(M_total, R_hub[:, 1]))
        M2c_disk = float(M2c_raw)
        M2s_disk = float(M2s_raw)

        dt = max(0.0, t - self._last_t) if self._last_t >= 0.0 else 0.0
        self._ode_step(T_disk, Mx_disk, My_disk, M2c_disk, M2s_disk,
                       v_axial, v_inplane, dt)
        self._last_t = t

        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        self.last_T         = T_disk
        self.last_v_axial   = v_axial
        self.last_v_i       = self._v0
        self.last_v_inplane = v_inplane
        self.last_ramp      = ramp
        self.last_M_spin    = M_spin_world.copy()
        self.last_M_cyc     = M_cyc_world.copy()
        self.last_H_force   = float(np.linalg.norm(F_total - T_disk * disk_normal))
        self.last_v1c       = self._v1c
        self.last_v1s       = self._v1s

        log.debug(
            "t=%.3f T=%.2fN v0=%.3f v1c=%.3f v1s=%.3f v_ax=%.2f v_ip=%.2f tau0=%.3f",
            t, self.last_T, self._v0, self._v1c, self._v1s,
            v_axial, v_inplane, self.last_tau0,
        )

        return AeroResult(
            F_world   = F_total.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = Q_spin_scalar,
            M_spin    = M_spin_world.copy(),
        )
