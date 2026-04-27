"""
aero_peters_he.py — BEM with Peters-He (1991) 3-state dynamic inflow.

State vector  v = [v0, v1c, v1s]  (physical units, m/s)
─────────────────────────────────────────────────────────
  v0  : uniform induced velocity
  v1c : longitudinal (cosine) inflow harmonic — driven by disk pitching moment
  v1s : lateral    (sine)    inflow harmonic — driven by disk rolling  moment

Inflow at blade element (r, ψ):
  v_i(r, ψ) = v0 + v1c·(r/R)·cos(ψ) + v1s·(r/R)·sin(ψ)

This is the non-uniform induction that:
  • Coleman approximates statically in SkewedWakeBEM
  • Glauert ignores entirely (uniform only)
  • Peters-He evolves dynamically via an ODE

ODE  (implicit-Euler, always stable):
  τ0 · dv0/dt  = v0_ss  − v0       uniform inflow lag
  τ1 · dv1c/dt = v1c_ss − v1c      longitudinal harmonic lag
  τ1 · dv1s/dt = v1s_ss − v1s      lateral harmonic lag

Steady-state targets  (from Glauert momentum theory + disk moments):
  V_T     = sqrt(v_inplane² + (v_axial + v0)²)       total effective velocity
  v0_ss  = T  / (2·ρ·A·V_T)                          Rankine-Froude / Glauert
  v1c_ss = My / (ρ·A·R·V_T)   My = disk pitching moment (body-y axis)
  v1s_ss = Mx / (ρ·A·R·V_T)   Mx = disk rolling  moment (body-x axis)

Time constants  (Pitt-Peters apparent mass matrix, Pitt & Peters 1981):
  τ0 = 8R  / (3π·V_T)     uniform
  τ1 = 16R / (45π·V_T)    harmonic  (≈ 5× faster than uniform)

Cold-start:  first call (dt=0) iterates to the quasi-static fixed point
             so subsequent ODE steps warm-start from a good initial state.

Interface:  identical to GlauertBEM — drop-in replacement.

References
──────────
Pitt D.M. & Peters D.A. (1981) "Theoretical Prediction of Dynamic-Inflow
  Derivatives", Vertica 5(1): 21–34.
Peters D.A. & He C.J. (1991) "Finite State Induced Flow Models Part II:
  Three-Dimensional Rotor Disk", J. Aircraft 28(5): 323–333.
Leishman J.G. (2006) Principles of Helicopter Aerodynamics, §10.5.
"""

import math
import logging
import sys
import os
import numpy as np

_SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

from aero import AeroResult

log = logging.getLogger(__name__)

# Pitt-Peters apparent-mass coefficients (non-dimensional, × ρAR gives kg/m²)
_M0_COEFF = 8.0  / (3.0  * math.pi)  # uniform:       ≈ 0.849
_M1_COEFF = 16.0 / (45.0 * math.pi)  # 1st harmonic:  ≈ 0.113  (τ1 ≈ τ0/7.5)
_M2_COEFF = 16.0 / (105.0 * math.pi) # 2nd harmonic:  ≈ 0.048  (τ2 ≈ τ1/2.3)
# _M2_COEFF is approximate — the exact value follows from the Peters-He (1991)
# finite-state Galerkin expansion (Table I).  This value gives τ2 < τ1, i.e.
# the second harmonic adapts faster than the first, which improves damping.


class PetersHeBEM:
    """
    BEM with Peters-He 3-state dynamic inflow.

    Drop-in replacement for GlauertBEM.  Same constructor, same
    compute_forces() signature.  Adds three inflow ODE states.
    """

    N_AZ     = 12
    N_RADIAL = 10

    def __init__(self, rotor, ramp_time: float = 5.0,
                 state_dict: dict | None = None, **overrides):
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
        self.k_drive_spin  = float(p["k_drive_spin"])
        self.k_drag_spin   = float(p["k_drag_spin"])
        self.pitch_gain_rad = float(p["pitch_gain_rad"])

        span = self.R_TIP - self.R_ROOT
        self.AR        = float(p["aspect_ratio"])
        self.R_CP      = self.R_ROOT + (2.0 / 3.0) * span   # representative radius (diagnostic)
        self.disk_area = math.pi * self.R_TIP**2   # full disk; consistent with Glauert

        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )
        self._S_elem = self.CHORD * self._dr

        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        # ── Peters-He inflow states ──────────────────────────────────────────
        if state_dict is not None:
            self._v0     = float(state_dict["v0"])
            self._v1c    = float(state_dict["v1c"])
            self._v1s    = float(state_dict["v1s"])
            self._v2c    = float(state_dict.get("v2c", 0.0))
            self._v2s    = float(state_dict.get("v2s", 0.0))
            self._last_t = float(state_dict.get("last_t", 0.0))
        else:
            self._v0     = 0.0   # uniform induced velocity [m/s]
            self._v1c    = 0.0   # longitudinal 1st harmonic [m/s]
            self._v1s    = 0.0   # lateral      1st harmonic [m/s]
            self._v2c    = 0.0   # longitudinal 2nd harmonic [m/s]
            self._v2s    = 0.0   # lateral      2nd harmonic [m/s]
            self._last_t = -1.0  # sentinel: triggers cold start on first call

        # ── Diagnostics (same names as GlauertBEM / DeSchutterAero) ─────────
        self.last_T         = 0.0
        self.last_v_axial   = 0.0
        self.last_v_i       = self._v0
        self.last_v_inplane = 0.0
        self.last_ramp      = 0.0
        self.last_Q_drive   = 0.0   # k_drive_spin * v_inplane
        self.last_Q_drag    = 0.0   # k_drag_spin  * omega^2
        self.last_M_spin    = np.zeros(3)
        self.last_M_cyc     = np.zeros(3)
        self.last_H_force   = 0.0
        self.last_F_prandtl      = 1.0
        self.last_v1c            = self._v1c
        self.last_v1s            = self._v1s
        self.last_tau0           = 0.0
        self.last_skew_angle_deg = 0.0

    def to_dict(self) -> dict:
        """Serialize inflow state to a plain dict (JSON-safe)."""
        return {
            "type":   "PetersHeBEM",
            "v0":     self._v0,
            "v1c":    self._v1c,
            "v1s":    self._v1s,
            "v2c":    self._v2c,
            "v2s":    self._v2s,
            "last_t": self._last_t,
        }

    @classmethod
    def from_definition(cls, defn, state_dict: dict | None = None,
                        **overrides) -> "PetersHeBEM":
        return cls(defn, state_dict=state_dict, **overrides)

    def _ramp_factor(self, t: float) -> float:
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

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

    def _strip_forces(
        self,
        v0: float,
        v1c: float,
        v1s: float,
        v2c: float,
        v2s: float,
        v_rel_world: np.ndarray,
        disk_normal: np.ndarray,
        R_hub: np.ndarray,
        omega_rotor: float,
        collective_rad: float,
        tilt_lon_rad: float,
        tilt_lat_rad: float,
        spin_angle: float,
    ):
        """
        Strip integration with Peters-He 5-state non-uniform inflow.

        Inflow at element (r, ψ):
            v_i = v0
                + v1c·(r/R)·cos(ψ)  + v1s·(r/R)·sin(ψ)
                + v2c·(r/R)²·cos(2ψ) + v2s·(r/R)²·sin(2ψ)

        Returns (F_acc, M_acc, M2c_disk, M2s_disk) in world frame.
        M2c_disk / M2s_disk are the 2nd-harmonic disk moments used to
        drive the v2c / v2s ODE states.
        """
        omega_abs = abs(float(omega_rotor))
        F_acc = np.zeros(3)
        M_acc = np.zeros(3)
        M2c_disk = 0.0
        M2s_disk = 0.0
        F_prandtl_sum = 0.0

        v_ax_eff = abs(float(np.dot(v_rel_world, disk_normal)) + v0)

        for i in range(self.N_AB):
            phi_az = spin_angle + self._phi_offsets[i]
            ca  = math.cos(phi_az)
            sa  = math.sin(phi_az)
            cos2 = 2.0 * ca * ca - 1.0   # cos(2ψ)
            sin2 = 2.0 * sa * ca          # sin(2ψ)

            p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
            cp_k = math.cos(p_k)
            sp_k = math.sin(p_k)

            e_span_body   = np.array([ ca,        sa,       0.0])
            e_chord_body  = np.array([-sa * cp_k, ca * cp_k, sp_k])
            e_normal_body = np.array([ sa * sp_k, -ca * sp_k, cp_k])

            e_span_world   = R_hub @ e_span_body
            e_chord_world  = R_hub @ e_chord_body
            e_normal_world = R_hub @ e_normal_body
            e_tang         = np.cross(disk_normal, e_span_world)

            for j in range(self.N_RADIAL):
                r = self._r_stations[j]

                inflow_angle = math.atan2(v_ax_eff, max(omega_abs * r, 0.5))
                F_p = self._prandtl_F(r, inflow_angle)
                if i == 0:
                    F_prandtl_sum += F_p

                r_norm    = r / self.R_TIP
                r_norm_sq = r_norm * r_norm
                v_i_local = (v0
                             + v1c * r_norm * ca  + v1s * r_norm * sa
                             + v2c * r_norm_sq * cos2 + v2s * r_norm_sq * sin2)
                v_induced = v_i_local * disk_normal

                v_rot = omega_rotor * r * e_tang
                ua    = v_rel_world - v_rot - v_induced

                ua_norm = float(np.linalg.norm(ua))
                if ua_norm < 0.5:
                    continue

                ua_chord  = float(np.dot(ua, e_chord_world))
                if abs(ua_chord) < 1e-6:
                    continue
                ua_normal = float(np.dot(ua, e_normal_world))

                alpha = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, -ua_normal / ua_chord))
                CL = self.CL0 + self.CL_ALPHA * alpha
                CD = self.CD0 + CL**2 / (math.pi * self.AR * self.OSWALD)

                q = 0.5 * self.RHO * ua_norm**2 * self._S_elem * F_p

                e_lift_raw = np.cross(ua, e_span_world)
                lift_norm  = float(np.linalg.norm(e_lift_raw))
                if lift_norm < 1e-9:
                    continue
                e_lift = e_lift_raw / lift_norm

                F_strip = q * (CL * e_lift + CD * (ua / ua_norm))
                r_cp    = r * e_span_world

                F_acc += F_strip
                M_acc += np.cross(r_cp, F_strip)

                # 2nd-harmonic disk moment accumulators
                f_axial   = float(np.dot(F_strip, disk_normal))
                M2c_disk += f_axial * r_norm_sq * cos2
                M2s_disk += f_axial * r_norm_sq * sin2

        if self.N_RADIAL > 0:
            self.last_F_prandtl = F_prandtl_sum / self.N_RADIAL

        return F_acc, M_acc, M2c_disk, M2s_disk

    def _ode_step(
        self,
        T: float,
        Mx_disk: float,
        My_disk: float,
        M2c_disk: float,
        M2s_disk: float,
        v_axial: float,
        v_inplane: float,
        dt: float,
    ) -> None:
        """
        Integrate Peters-He 5-state inflow ODE one step.

        Uses implicit (backward) Euler — unconditionally stable for any dt.
        """
        V_T = math.sqrt(v_inplane**2 + (v_axial + self._v0)**2 + 1e-6)
        rho_A = self.RHO * self.disk_area
        R2    = self.R_TIP ** 2

        # Steady-state targets
        v0_ss  = T        / (2.0 * rho_A * V_T)
        v1c_ss = My_disk  / (rho_A * self.R_TIP * V_T)
        v1s_ss = -Mx_disk / (rho_A * self.R_TIP * V_T)
        v2c_ss = M2c_disk / (rho_A * R2 * V_T)
        v2s_ss = -M2s_disk / (rho_A * R2 * V_T)

        # Time constants (Pitt-Peters apparent mass)
        tau0 = _M0_COEFF * self.R_TIP / max(V_T, 0.1)
        tau1 = _M1_COEFF * self.R_TIP / max(V_T, 0.1)
        tau2 = _M2_COEFF * self.R_TIP / max(V_T, 0.1)

        self.last_tau0 = tau0

        if dt <= 0.0:
            return

        # Implicit Euler: v_new = (v_old + dt/τ · v_ss) / (1 + dt/τ)
        α0 = dt / (tau0 + dt)
        α1 = dt / (tau1 + dt)
        α2 = dt / (tau2 + dt)
        self._v0  += α0 * (v0_ss  - self._v0)
        self._v1c += α1 * (v1c_ss - self._v1c)
        self._v1s += α1 * (v1s_ss - self._v1s)
        self._v2c += α2 * (v2c_ss - self._v2c)
        self._v2s += α2 * (v2s_ss - self._v2s)

    def _cold_start(
        self,
        v_rel_world: np.ndarray,
        disk_normal: np.ndarray,
        R_hub: np.ndarray,
        omega_rotor: float,
        collective_rad: float,
        tilt_lon_rad: float,
        tilt_lat_rad: float,
        v_axial: float,
        v_inplane: float,
    ) -> None:
        """
        Iterate to the quasi-static fixed point on the first call (dt=0).

        The V_T denominator in the steady-state target is undefined when v0=0
        and v_axial=0 (hover cold start).  Bootstrap v0 from the hover formula
        first so V_T is well-defined, then relax all three states to the fixed
        point with a 0.1/0.9 update — same convergence approach as GlauertBEM.
        """
        rho_A = self.RHO * self.disk_area

        # Step 1: get strip thrust at zero induction to bootstrap v0
        F0, _, _, _ = self._strip_forces(
            0.0, 0.0, 0.0, 0.0, 0.0,
            v_rel_world, disk_normal, R_hub, omega_rotor,
            collective_rad, tilt_lon_rad, tilt_lat_rad, spin_angle=0.0,
        )
        T0 = float(np.dot(F0 / self.N_AZ, disk_normal))
        # Hover formula gives a safe non-zero starting point
        self._v0 = math.sqrt(max(abs(T0), 0.1) / (2.0 * rho_A))

        # Step 2: relax all 5 states to fixed point (0.1 step = conservative)
        R2 = self.R_TIP ** 2
        for _ in range(60):
            F_acc, M_acc, M2c, M2s = self._strip_forces(
                self._v0, self._v1c, self._v1s, self._v2c, self._v2s,
                v_rel_world, disk_normal, R_hub, omega_rotor,
                collective_rad, tilt_lon_rad, tilt_lat_rad, spin_angle=0.0,
            )
            T_raw    = float(np.dot(F_acc / self.N_AZ, disk_normal))
            Mx_disk  = float(np.dot(M_acc / self.N_AZ, R_hub[:, 0]))
            My_disk  = float(np.dot(M_acc / self.N_AZ, R_hub[:, 1]))
            M2c_disk = M2c / self.N_AZ
            M2s_disk = M2s / self.N_AZ

            V_T = math.sqrt(v_inplane**2 + (v_axial + self._v0)**2 + 1e-6)
            V_T = max(V_T, self._v0 * 0.5 + 0.01)   # guard against V_T shrinking to 0

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

        # ── Cold start: iterate to quasi-static fixed point on first call ───
        if self._last_t < 0.0:
            self._cold_start(
                v_rel_world, disk_normal, R_hub, omega_rotor,
                collective_rad, tilt_lon_rad, tilt_lat_rad,
                v_axial, v_inplane,
            )

        # ── Strip integration with current inflow states ─────────────────────
        F_acc, M_acc, M2c_raw, M2s_raw = self._strip_forces(
            self._v0, self._v1c, self._v1s, self._v2c, self._v2s,
            v_rel_world, disk_normal, R_hub, omega_rotor,
            collective_rad, tilt_lon_rad, tilt_lat_rad, spin_angle,
        )

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        # ── Extract disk-frame moments for ODE forcing ───────────────────────
        T_disk   = float(np.dot(F_total, disk_normal))
        Mx_disk  = float(np.dot(M_total, R_hub[:, 0]))
        My_disk  = float(np.dot(M_total, R_hub[:, 1]))
        M2c_disk = float(M2c_raw * scale)
        M2s_disk = float(M2s_raw * scale)

        # ── ODE step ─────────────────────────────────────────────────────────
        dt = max(0.0, t - self._last_t) if self._last_t >= 0.0 else 0.0
        self._ode_step(T_disk, Mx_disk, My_disk, M2c_disk, M2s_disk,
                       v_axial, v_inplane, dt)
        self._last_t = t

        # ── Decompose into spin and orbital moments ───────────────────────────
        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # ── Diagnostics ───────────────────────────────────────────────────────
        Q_drive = float(self.k_drive_spin * v_inplane)
        Q_drag  = float(self.k_drag_spin * omega_abs**2)
        self.last_T         = T_disk
        self.last_v_axial   = v_axial
        self.last_v_i       = self._v0        # uniform component — comparable to GlauertBEM
        self.last_v_inplane = v_inplane
        self.last_ramp      = ramp
        self.last_Q_drive   = Q_drive
        self.last_Q_drag    = Q_drag
        self.last_M_spin    = np.array(M_spin_world)
        self.last_M_cyc     = np.array(M_cyc_world)
        self.last_H_force   = float(np.linalg.norm(
            F_total - T_disk * disk_normal
        ))
        self.last_v1c = self._v1c
        self.last_v1s = self._v1s
        chi = math.atan2(v_inplane, max(abs(v_axial + self._v0), 0.01))
        self.last_skew_angle_deg = math.degrees(chi)

        log.debug(
            "t=%.3f T=%.2fN v0=%.3f v1c=%.3f v1s=%.3f v_ax=%.2f v_ip=%.2f tau0=%.3f",
            t, self.last_T, self._v0, self._v1c, self._v1s,
            v_axial, v_inplane, self.last_tau0,
        )

        return AeroResult(
            F_world   = np.array(F_total),
            M_orbital = np.array(M_cyc_world),
            Q_spin    = Q_spin_scalar,
            M_spin    = np.array(M_spin_world),
        )
