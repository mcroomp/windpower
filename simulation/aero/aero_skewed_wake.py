"""
aero_skewed_wake.py — BEM with Coleman skewed-wake non-uniform induction.

Standard BEM assumes the wake is axisymmetric and straight behind the disk.
For a tilted rotor with significant advance ratio μ (typical RAWES operation
with disk tilt of 30–60°), the wake is skewed at angle χ from the disk normal
and the induction is non-uniform across the disk:

    v_i(r, ψ) = v_i0 · (1 + K · (r/R) · cos(ψ − ψ_skew))
    K          = tan(χ/2)
    χ          = atan2(v_inplane, |v_axial + v_i0|)   [wake skew angle]
    ψ_skew     = atan2(v_ip_y, v_ip_x)                [in-plane wind azimuth]

where ψ is the blade azimuth angle.  For large advance ratios:
    μ = 0.10 → χ ≈ 47°, K ≈ 0.47   (mild asymmetry)
    μ = 0.20 → χ ≈ 63°, K ≈ 0.70   (significant — typical RAWES reel-out)
    μ = 0.40 → χ ≈ 76°, K ≈ 1.00   (large asymmetry, advancing side dominated)

This non-uniformity:
  • Increases lift on the advancing blade (less local induction → higher AoA)
  • Decreases lift on the retreating blade (more induction → lower AoA)
  • Produces a pitching moment on the disk (longitudinal flapping moment)
  • Is the primary cause of H-force asymmetry in forward flight

The Prandtl tip-loss correction (from PrandtlBEM) is also included here,
since skewed-wake codes always combine both corrections in practice.

References
----------
Coleman R.P., Feingold A.M. (1945) *Theory of Self-Excited Mechanical
    Oscillations of Helicopter Rotors*, NACA TN-1181.
Glauert H. (1926) *A General Theory of the Autogyro*, ARCR&M No. 1111.
Leishman J.G. (2006) *Principles of Helicopter Aerodynamics*, 2nd ed., §5.5.
Drees J.M., Hendal W.P. (1951) *The Field of Flow Through a Helicopter Rotor*,
    J. Aircraft Eng., Vol. 23, pp. 107–111.
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


class SkewedWakeBEM:
    """
    BEM + Coleman skewed-wake non-uniform induction + Prandtl tip/root loss.

    Key diagnostic attributes (set after each compute_forces call)
    --------------------------------------------------------------
    last_skew_angle_deg — wake skew angle χ [°]  (0 = axial flow, 90 = edgewise)
    last_K_skew         — Coleman K = tan(χ/2) — induction asymmetry factor
    last_F_prandtl      — mean Prandtl tip/root loss factor
    last_Q_spin         — empirical spin ODE torque [N·m]
    last_M_spin         — spin-axis moment vector [N·m]
    """

    N_AZ     = 12    # more azimuth points — skew effect is azimuth-sensitive
    N_RADIAL = 10

    def __init__(self, rotor, ramp_time: float = 5.0, **overrides):
        p = {**rotor.aero_kwargs(), **overrides}
        self.N_BLADES = int(p["n_blades"])
        self.R_ROOT   = float(p["r_root"])
        self.R_TIP    = float(p["r_tip"])
        self.CHORD    = float(p["chord"])
        self.RHO      = float(p["rho"])
        self.OSWALD   = float(p["oswald_eff"])
        self.CD0      = float(p["CD0"])
        self.CL0      = float(p["CL0"])
        self.CL_ALPHA = float(p["CL_alpha"])
        self.AOA_LIMIT = float(p["aoa_limit"])
        self.ramp_time = float(ramp_time)
        self.k_drive_spin  = float(p["k_drive_spin"])
        self.k_drag_spin   = float(p["k_drag_spin"])
        self.pitch_gain_rad = float(p["pitch_gain_rad"])

        span = self.R_TIP - self.R_ROOT
        self.S_blade  = span * self.CHORD
        self.AR       = float(p["aspect_ratio"])
        self.R_CP     = self.R_ROOT + (2.0 / 3.0) * span
        self.disk_area = math.pi * (self.R_TIP ** 2 - self.R_ROOT ** 2)

        self._dr = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )
        self._r_norm = self._r_stations / self.R_TIP   # r/R, shape (N_RADIAL,)
        self._S_elem  = self.CHORD * self._dr

        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        # Diagnostics
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_Q_spin         = 0.0
        self.last_Q_drive        = 0.0
        self.last_Q_drag         = 0.0
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_H_force        = 0.0
        self.last_skew_angle_deg = 0.0
        self.last_K_skew         = 0.0
        self.last_F_prandtl      = 1.0

    @classmethod
    def from_definition(cls, defn, **overrides) -> "SkewedWakeBEM":
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

    def _prandtl_F(self, r: float, phi_rad: float) -> float:
        sin_phi = abs(math.sin(phi_rad))
        if sin_phi < 1e-4:
            sin_phi = 1e-4
        f_tip  = (self.N_BLADES / 2.0) * (self.R_TIP - r) / (r * sin_phi)
        F_tip  = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_tip))))
        r_ref  = max(self.R_ROOT, 0.01)
        f_root = (self.N_BLADES / 2.0) * (r - self.R_ROOT) / (r_ref * sin_phi)
        F_root = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_root))))
        return max(0.01, F_tip * F_root)

    def _coleman_skew(self, v_inplane: float, v_axial_eff: float) -> tuple:
        """
        Compute Coleman skewed-wake parameters.

        Returns (K_skew, skew_angle_rad) where:
          K_skew = tan(χ/2)
          χ = wake skew angle (0 = axial, π/2 = edgewise)
        """
        chi = math.atan2(v_inplane, max(abs(v_axial_eff), 0.01))
        K   = math.tan(min(chi / 2.0, math.radians(89.0)))   # clamp to prevent K→∞
        return K, chi

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

        Applies Coleman skewed-wake correction: induction is non-uniform across
        the disk, modulated as  v_i(r, ψ) = v_i0·(1 + K·(r/R)·cos(ψ − ψ_skew)).
        Also applies Prandtl tip/root loss per radial strip.
        """
        ramp = self._ramp_factor(t)

        disk_normal = R_hub[:, 2]
        spin_sign   = float(np.sign(omega_rotor)) if omega_rotor != 0.0 else 1.0
        omega_abs   = abs(float(omega_rotor))

        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # ── Uniform induction bootstrap (used as v_i0 for skew correction) ───
        v_i0 = 0.0
        for _ in range(3):
            v_tan = omega_abs * self.R_CP
            v_loc = math.sqrt(v_tan ** 2 + (v_axial + v_i0) ** 2)
            if v_loc > 0.5:
                inflow = math.atan2(v_axial + v_i0, v_tan)
                aoa0   = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, inflow + collective_rad))
                CL0_   = self.CL0 + self.CL_ALPHA * aoa0
                CD0_   = self.CD0 + CL0_ ** 2 / (math.pi * self.AR * self.OSWALD)
                q0     = 0.5 * self.RHO * v_loc ** 2
                T_est  = max(0.0, self.N_BLADES * q0 * self.S_blade
                             * (CL0_ * math.cos(inflow) - CD0_ * math.sin(inflow)))
                v_i0   = self._induced_velocity(T_est, v_axial)

        # ── Coleman skewed-wake parameters ────────────────────────────────────
        K_skew, chi = self._coleman_skew(v_inplane, v_axial + v_i0)
        self.last_K_skew         = K_skew
        self.last_skew_angle_deg = math.degrees(chi)

        # Direction of in-plane wind in disk frame (for ψ_skew calculation)
        # ψ_skew = azimuth of the upwind direction in the disk plane
        if v_inplane > 0.01:
            v_ip_unit = v_inplane_vec / v_inplane  # world frame, in disk plane
            # Project onto disk-frame x-axis (East projected onto disk plane)
            _EAST = np.array([0.0, 1.0, 0.0])   # NED: East = Y axis
            _ep   = _EAST - np.dot(_EAST, disk_normal) * disk_normal
            if np.linalg.norm(_ep) > 1e-6:
                _bx = _ep / np.linalg.norm(_ep)
            else:
                _NORTH = np.array([1.0, 0.0, 0.0])  # NED: North = X axis
                _ep    = _NORTH - np.dot(_NORTH, disk_normal) * disk_normal
                _bx    = _ep / np.linalg.norm(_ep)
            _by       = np.cross(disk_normal, _bx)
            # In-plane wind azimuth in disk frame
            psi_skew  = math.atan2(float(np.dot(v_ip_unit, _by)),
                                   float(np.dot(v_ip_unit, _bx)))
        else:
            psi_skew  = 0.0

        # ── Prandtl F per radial strip (vectorized) ───────────────────────────
        v_ax_eff    = abs(v_axial + v_i0)
        v_tan_j     = np.maximum(omega_abs * self._r_stations, 0.5)
        phi_j       = np.arctan2(v_ax_eff, v_tan_j)
        sin_phi_j   = np.maximum(np.abs(np.sin(phi_j)), 1e-4)
        r_ref       = max(self.R_ROOT, 0.01)
        f_tip       = (self.N_BLADES / 2.0) * (self.R_TIP - self._r_stations) / (self._r_stations * sin_phi_j)
        f_root      = (self.N_BLADES / 2.0) * (self._r_stations - self.R_ROOT) / (r_ref * sin_phi_j)
        F_tip_arr   = (2.0 / math.pi) * np.arccos(np.minimum(1.0, np.exp(-np.maximum(0.0, f_tip))))
        F_root_arr  = (2.0 / math.pi) * np.arccos(np.minimum(1.0, np.exp(-np.maximum(0.0, f_root))))
        F_per_strip = np.maximum(0.01, F_tip_arr * F_root_arr)

        self.last_F_prandtl = float(np.mean(F_per_strip))

        # ── Azimuth phase angles  ─────────────────────────────────────────────
        phi_az = spin_angle + self._phi_offsets   # (N_AB,)
        ca = np.cos(phi_az)
        sa = np.sin(phi_az)

        # ── Coleman non-uniform induction: v_i(r, ψ) ─────────────────────────
        # v_i(ψ, r) = v_i0 · (1 + K · (r/R) · cos(ψ − ψ_skew))
        # Shape: (N_AB, N_RADIAL)
        cos_psi_skew   = np.cos(phi_az - psi_skew)           # (N_AB,)
        vi_modulation  = (1.0 + K_skew * self._r_norm[None, :]
                          * cos_psi_skew[:, None])             # (N_AB, N_RADIAL)
        # v_i_local: non-uniform induced velocity per (az, blade, radial)
        v_i_local = v_i0 * vi_modulation                      # (N_AB, N_RADIAL)

        # ── Blade vectors in world frame  ─────────────────────────────────────
        e_span_body   = np.stack([ca, sa, np.zeros(self.N_AB)], axis=1)
        p_k           = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
        cp_k, sp_k    = np.cos(p_k), np.sin(p_k)
        e_chord_body  = np.stack([-sa * cp_k,  ca * cp_k,  sp_k], axis=1)
        e_normal_body = np.stack([ sa * sp_k, -ca * sp_k,  cp_k], axis=1)

        e_span_world   = e_span_body   @ R_hub.T
        e_chord_world  = e_chord_body  @ R_hub.T
        e_normal_world = e_normal_body @ R_hub.T
        e_tang         = np.cross(disk_normal[None, :], e_span_world)

        # ── Strip positions  ──────────────────────────────────────────────────
        # r_cp_world[i,j] = r_j · e_span_world[i]  (linearity of rotation)
        r_cp_world = e_span_world[:, None, :] * self._r_stations[None, :, None]

        # ── Apparent wind with non-uniform induction ──────────────────────────
        # Rotational velocity: ω × r_cp = ω · (disk_normal × r_cp)
        # Identity: disk_normal × (r · e_span) = r · (disk_normal × e_span) = r · e_tang
        v_rot  = omega_rotor * e_tang[:, None, :] * self._r_stations[None, :, None]
        # Non-uniform induction velocity (along disk normal, per strip)
        v_i_vec_field = v_i_local[..., None] * disk_normal[None, None, :]  # (N_AB, N_RADIAL, 3)
        v_base = wind_world - v_hub_world    # (3,)
        ua     = v_base[None, None, :] - v_rot - v_i_vec_field  # (N_AB, N_RADIAL, 3)

        # ── Velocity decomposition  ───────────────────────────────────────────
        ua_norm   = np.sqrt(np.einsum('ijk,ijk->ij', ua, ua))
        ua_chord  = np.einsum('ijk,ik->ij', ua, e_chord_world)
        ua_normal = np.einsum('ijk,ik->ij', ua, e_normal_world)
        valid     = (ua_norm >= 0.5) & (np.abs(ua_chord) >= 1e-6)

        if not np.any(valid):
            self.last_M_spin = np.zeros(3)
            self.last_M_cyc  = np.zeros(3)
            self.last_Q_spin = 0.0
            return AeroResult(
                F_world=np.zeros(3), M_orbital=np.zeros(3), Q_spin=0.0, M_spin=np.zeros(3)
            )

        safe_chord = np.where(np.abs(ua_chord) >= 1e-6, ua_chord, 1.0)
        alpha      = np.where(valid,
                              np.clip(-ua_normal / safe_chord,
                                      -self.AOA_LIMIT, self.AOA_LIMIT),
                              0.0)

        # ── Aerodynamic coefficients  ─────────────────────────────────────────
        CL    = self.CL0 + self.CL_ALPHA * alpha
        CD    = self.CD0 + CL ** 2 / (math.pi * self.AR * self.OSWALD)
        q_dyn = np.where(valid, 0.5 * self.RHO * ua_norm ** 2 * self._S_elem, 0.0)

        # Apply Prandtl F per radial strip (broadcast over N_AB)
        q_dyn = q_dyn * F_per_strip[None, :]

        # ── Lift direction and forces  ────────────────────────────────────────
        # Manual cross product avoids np.cross dispatch overhead on (N_AB,N_RADIAL,3)
        ua0, ua1, ua2 = ua[..., 0], ua[..., 1], ua[..., 2]
        es0 = e_span_world[:, None, 0]; es1 = e_span_world[:, None, 1]; es2 = e_span_world[:, None, 2]
        lift_raw  = np.stack([ua1*es2 - ua2*es1, ua2*es0 - ua0*es2, ua0*es1 - ua1*es0], axis=-1)
        lift_norm = np.sqrt(np.einsum('ijk,ijk->ij', lift_raw, lift_raw))[..., None]
        e_lift    = np.where(lift_norm > 1e-9,
                             lift_raw / np.maximum(lift_norm, 1e-9), 0.0)

        ua_unit  = ua / np.maximum(ua_norm[..., None], 1e-9)
        F_strip  = q_dyn[..., None] * (CL[..., None] * e_lift + CD[..., None] * ua_unit)
        # Manual cross product for M_strip = r_cp_world × F_strip
        rc0, rc1, rc2 = r_cp_world[...,0], r_cp_world[...,1], r_cp_world[...,2]
        fs0, fs1, fs2 = F_strip[...,0],    F_strip[...,1],    F_strip[...,2]
        M_strip  = np.stack([rc1*fs2 - rc2*fs1, rc2*fs0 - rc0*fs2, rc0*fs1 - rc1*fs0], axis=-1)
        Q_strip  = (np.einsum('ijk,ik->ij', F_strip, e_tang)
                    * self._r_stations[None, :] * spin_sign)

        # ── Accumulate  ───────────────────────────────────────────────────────
        F_acc = F_strip.sum(axis=(0, 1))
        M_acc = M_strip.sum(axis=(0, 1))

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # ── Diagnostics  ──────────────────────────────────────────────────────
        self.last_T         = float(np.dot(F_total, disk_normal))
        self.last_v_axial   = v_axial
        self.last_v_i       = v_i0
        self.last_v_inplane = v_inplane
        self.last_ramp      = ramp
        self.last_Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs ** 2)
        self.last_M_spin    = M_spin_world.copy()
        self.last_M_cyc     = M_cyc_world.copy()
        self.last_H_force   = float(np.linalg.norm(F_total - self.last_T * disk_normal))

        log.debug("t=%.3f T=%.2fN χ=%.1f° K=%.3f F_prandtl=%.3f",
                  t, self.last_T, self.last_skew_angle_deg, K_skew, self.last_F_prandtl)

        return AeroResult(
            F_world   = F_total.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs**2),
            M_spin    = M_spin_world.copy(),
        )
