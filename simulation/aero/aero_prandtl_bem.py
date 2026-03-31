"""
aero_prandtl_bem.py — BEM with Prandtl tip-loss and root-loss corrections.

Standard state-of-the-art enhancement over flat actuator-disk BEM:
  - Prandtl tip-loss factor  F_tip  = (2/π)·arccos(exp(−f_tip))
    f_tip  = (N/2)·(R−r)  / (r·|sin φ|)
  - Prandtl root-loss factor F_root = (2/π)·arccos(exp(−f_root))
    f_root = (N/2)·(r−r_root) / (r_root·|sin φ|)
  - Combined F = F_tip × F_root applied as a multiplier on each strip's
    aerodynamic force and moment.

The inflow angle φ used for F is evaluated per radial strip from the azimuth-
averaged axial + tangential velocity (standard engineering BEM approximation):
    φ(r) = atan2(|v_axial + v_i|, ω·r)

Structure is identical to DeSchutterAero (same vectorised layout over
N_AB = N_AZ × N_BLADES azimuths) so direct comparison is possible.

Interface
---------
Same as RotorAero / DeSchutterAero:
    forces = aero.compute_forces(collective_rad, tilt_lon, tilt_lat,
                                 R_hub, v_hub_world, omega_rotor,
                                 wind_world, t)
    aero.last_Q_spin   — empirical spin torque [N·m]
    aero.last_M_spin   — spin-axis moment vector [N·m]
    aero.last_F_prandtl — mean Prandtl factor across strips (diagnostic)

References
----------
Leishman J.G. (2006) *Principles of Helicopter Aerodynamics*, 2nd ed., §3.6.
Bramwell A.R.S., Done G., Balmford D. (2001) *Bramwell's Helicopter Dynamics*, §3.3.
Glauert H. (1935) Airplane Propellers, in *Aerodynamic Theory* Vol. IV, §XIV.2.
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

_PITCH_GAIN_RAD_DEFAULT = 0.3   # fallback when not supplied via rotor_definition


class PrandtlBEM:
    """
    BEM + Prandtl tip/root-loss aerodynamic model for RAWES.

    Adds Prandtl-Glauert tip- and root-loss correction to the per-blade
    strip-theory BEM.  Forces and moments are vectorised over all
    (azimuth × blade × radial) strips as in DeSchutterAero.

    Spin diagnostics
    ----------------
    last_Q_spin    — empirical spin ODE torque: k_drive·v_inplane − k_drag·ω²
    last_F_prandtl — mean Prandtl F across radial strips (1.0 = no correction)
    """

    N_AZ    = 8      # azimuth samples per revolution
    N_RADIAL = 12    # radial strips (more than base models — tip-loss needs resolution)

    def __init__(self, rotor, ramp_time: float = 5.0, **overrides):
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
        self.k_drive_spin   = float(p["k_drive_spin"])
        self.k_drag_spin    = float(p["k_drag_spin"])
        self.pitch_gain_rad = float(p.get("pitch_gain_rad", _PITCH_GAIN_RAD_DEFAULT))

        span          = self.R_TIP - self.R_ROOT
        self.S_blade  = span * self.CHORD
        self.AR       = float(p.get("aspect_ratio") or span / self.CHORD)
        self.R_CP     = self.R_ROOT + (2.0 / 3.0) * span
        self.disk_area = math.pi * (self.R_TIP ** 2 - self.R_ROOT ** 2)

        self._dr = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )
        self._S_elem = self.CHORD * self._dr

        # Precompute phase offsets (same layout as DeSchutterAero)
        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        # Diagnostics
        self.last_T            = 0.0
        self.last_v_axial      = 0.0
        self.last_v_i          = 0.0
        self.last_v_inplane    = 0.0
        self.last_ramp         = 0.0
        self.last_Q_spin       = 0.0
        self.last_M_spin       = np.zeros(3)
        self.last_M_cyc        = np.zeros(3)
        self.last_H_force      = 0.0
        self.last_F_prandtl    = 1.0   # mean Prandtl correction factor

    @classmethod
    def from_definition(cls, defn, **overrides) -> "PrandtlBEM":
        return cls(defn, **overrides)

    def _ramp_factor(self, t: float) -> float:
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _induced_velocity(self, T_guess: float, v_axial: float = 0.0) -> float:
        """Actuator-disk induced velocity (standard momentum theory)."""
        T_abs = max(abs(T_guess), 0.01)
        v_ax  = abs(v_axial)
        disc  = v_ax ** 2 + 2.0 * T_abs / (self.RHO * self.disk_area)
        return max(0.0, (-v_ax + math.sqrt(disc)) / 2.0)

    def _prandtl_F(self, r: float, phi_rad: float) -> float:
        """
        Combined Prandtl tip + root loss factor.

        Parameters
        ----------
        r       : radial station [m]
        phi_rad : inflow angle [rad]  (positive = wind flows up through disk)

        Returns
        -------
        F in [0, 1]  (1.0 = no loss, near tip/root → 0)
        """
        sin_phi = abs(math.sin(phi_rad))
        if sin_phi < 1e-4:
            sin_phi = 1e-4   # prevent division by zero at very low inflow

        # Tip loss
        f_tip  = (self.N_BLADES / 2.0) * (self.R_TIP - r) / (r * sin_phi)
        F_tip  = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_tip))))

        # Root loss
        r_ref  = max(self.R_ROOT, 0.01)
        f_root = (self.N_BLADES / 2.0) * (r - self.R_ROOT) / (r_ref * sin_phi)
        F_root = (2.0 / math.pi) * math.acos(min(1.0, math.exp(-max(0.0, f_root))))

        return max(0.01, F_tip * F_root)

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
        Compute aerodynamic wrench [Fx,Fy,Fz,Mx,My,Mz] in world ENU [N, N·m].

        Identical interface to RotorAero / DeSchutterAero.  Prandtl tip+root loss
        is applied per radial strip using the azimuth-averaged inflow angle.

        Diagnostics after call
        ----------------------
        last_Q_spin     — empirical spin torque [N·m]
        last_F_prandtl  — mean Prandtl factor (< 1 → tip-loss active)
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

        # ── Induction bootstrap (3 passes at R_CP) ───────────────────────────
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

        v_i_vec = v_i * disk_normal

        # ── Prandtl F per radial strip (azimuth-averaged inflow angle) ────────
        F_per_strip = np.zeros(self.N_RADIAL)
        v_ax_eff = abs(v_axial + v_i)
        for j, r_j in enumerate(self._r_stations):
            v_tan_j = omega_abs * r_j
            phi_j   = math.atan2(v_ax_eff, max(v_tan_j, 0.5))
            F_per_strip[j] = self._prandtl_F(r_j, phi_j)

        self.last_F_prandtl = float(np.mean(F_per_strip))

        # ── Azimuth + blade phases  ───────────────────────────────────────────
        phi_az = spin_angle + self._phi_offsets   # (N_AB,)
        ca = np.cos(phi_az)
        sa = np.sin(phi_az)

        # ── Span, chord, normal vectors in world frame ────────────────────────
        e_span_body   = np.stack([ca, sa, np.zeros(self.N_AB)], axis=1)
        p_k           = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
        cp_k, sp_k    = np.cos(p_k), np.sin(p_k)
        e_chord_body  = np.stack([-sa * cp_k,  ca * cp_k,  sp_k], axis=1)
        e_normal_body = np.stack([ sa * sp_k, -ca * sp_k,  cp_k], axis=1)

        e_span_world   = e_span_body   @ R_hub.T   # (N_AB, 3)
        e_chord_world  = e_chord_body  @ R_hub.T
        e_normal_world = e_normal_body @ R_hub.T
        e_tang         = np.cross(disk_normal[None, :], e_span_world)

        # ── Strip positions and apparent wind  ────────────────────────────────
        # r_cp_world[i,j] = r_j · e_span_world[i]  (linearity of rotation)
        r_cp_world = e_span_world[:, None, :] * self._r_stations[None, :, None]

        # v_rot = ω × r_cp = ω · (disk_normal × r_cp) = ω · r · e_tang
        v_rot  = omega_rotor * e_tang[:, None, :] * self._r_stations[None, :, None]
        v_base = wind_world - v_hub_world - v_i_vec
        ua     = v_base[None, None, :] - v_rot         # (N_AB, N_RADIAL, 3)

        # ── Velocity decomposition ────────────────────────────────────────────
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

        # ── Coefficients  ─────────────────────────────────────────────────────
        CL = self.CL0 + self.CL_ALPHA * alpha
        CD = self.CD0 + CL ** 2 / (math.pi * self.AR * self.OSWALD)
        q_dyn = np.where(valid, 0.5 * self.RHO * ua_norm ** 2 * self._S_elem, 0.0)

        # Apply Prandtl F per radial strip (broadcast over N_AB)
        q_dyn = q_dyn * F_per_strip[None, :]   # (N_AB, N_RADIAL)

        # ── Lift direction and per-strip force / moment  ──────────────────────
        lift_raw  = np.cross(ua, e_span_world[:, None, :])
        lift_norm = np.linalg.norm(lift_raw, axis=-1, keepdims=True)
        e_lift    = np.where(lift_norm > 1e-9,
                             lift_raw / np.maximum(lift_norm, 1e-9), 0.0)

        ua_unit  = ua / np.maximum(ua_norm[..., None], 1e-9)
        F_strip  = q_dyn[..., None] * (CL[..., None] * e_lift + CD[..., None] * ua_unit)
        M_strip  = np.cross(r_cp_world, F_strip)

        Q_strip  = (np.einsum('ijk,ik->ij', F_strip, e_tang)
                    * self._r_stations[None, :] * spin_sign)

        # ── Accumulate  ───────────────────────────────────────────────────────
        F_acc  = F_strip.sum(axis=(0, 1))
        M_acc  = M_strip.sum(axis=(0, 1))
        Q_drive_acc = float(Q_strip[Q_strip >= 0].sum())
        Q_drag_acc  = float(Q_strip[Q_strip  < 0].sum())

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # ── Diagnostics  ──────────────────────────────────────────────────────
        self.last_T         = float(np.dot(F_total, disk_normal))
        self.last_v_axial   = v_axial
        self.last_v_i       = v_i
        self.last_v_inplane = v_inplane
        self.last_ramp      = ramp
        self.last_Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs ** 2)
        self.last_M_spin    = M_spin_world.copy()
        self.last_M_cyc     = M_cyc_world.copy()
        self.last_H_force   = float(np.linalg.norm(F_total - self.last_T * disk_normal))

        log.debug("t=%.3f T=%.2fN H=%.2fN F_prandtl_mean=%.3f ramp=%.2f",
                  t, self.last_T, self.last_H_force, self.last_F_prandtl, ramp)

        return AeroResult(
            F_world   = F_total.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs**2),
            M_spin    = M_spin_world.copy(),
        )
