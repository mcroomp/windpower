"""
aero_skewed_wake.py — BEM with Coleman skewed-wake quasi-static non-uniform inflow.

State:  v0 only (scalar, m/s) — warm-started across calls for iteration speed.
        v1c and v1s are derived algebraically each call; no ODE.
─────────────────────────────────────────────────────────────────────────────────
Inflow at blade element (r, ψ):
  v_i(r, ψ) = v0  +  v1c·(r/R)·cos(ψ)   (v1s = 0, no lateral gradient)

where:
  v1c  = K_x · v0               physical units [m/s]
  K_x  = tan(χ/2)               Coleman wake-skew correction
  χ    = arctan(v_inplane / (v_axial + v0))   wake skew angle

Self-consistency loop (runs every call, warm-started):
  1. Compute K_x from current v0 → v1c = K_x·v0
  2. Strip-integrate to get disk thrust T
  3. Update: v0 ← v0 + α·(v0_ss − v0)
                 where v0_ss = T / (2·ρ·A·V_T)
  4. Repeat until convergence (≤ 30 iterations, typically < 10 when warm)

Relationship to Peters-He
─────────────────────────
Coleman is the quasi-static fixed point of the Peters-He ODE:
  • v0  →  Glauert momentum theory (same denominator 2ρAV_T)
  • v1c →  tan(χ/2)·v0  (wake geometry; no dynamics, no memory of past Mx)
  • v1s →  0            (classical Coleman; symmetric about the longitudinal plane)

Interface: identical to PetersHeBEM — drop-in replacement.

References
──────────
Coleman R.P., Feingold A.M., Stempin C.W. (1945) "Evaluation of the
  Induced-Velocity Field of an Idealized Helicopter Rotor", NACA WR L-126.
Glauert H. (1926) "A General Theory of the Autogyro", ARC R&M No. 1111.
Leishman J.G. (2006) Principles of Helicopter Aerodynamics, §3.4, §10.4.
Pitt D.M. & Peters D.A. (1981) "Theoretical Prediction of Dynamic-Inflow
  Derivatives", Vertica 5(1): 21–34.
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

# tan(χ/2) saturates at χ → π/2 (edgewise flight); cap below tan(85°) ≈ 11.4
_KX_MAX = math.tan(math.radians(85.0))


class SkewedWakeBEM2:
    """
    BEM with Coleman skewed-wake quasi-static non-uniform inflow.

    Drop-in replacement for GlauertBEM and PetersHeBEM.  Same constructor,
    same compute_forces() signature.  Inflow is iterated to its quasi-static
    fixed point at every step — no ODE, no inflow lag.
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
        self.pitch_gain_rad = float(p["pitch_gain_rad"])

        span = self.R_TIP - self.R_ROOT
        self.AR        = float(p["aspect_ratio"])
        self.R_CP      = self.R_ROOT + (2.0 / 3.0) * span
        self.disk_area = math.pi * (self.R_TIP**2 - self.R_ROOT**2)

        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )
        self._S_elem = self.CHORD * self._dr

        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        # ── Inflow state: only v0 persists; v1c/v1s are derived each call ───
        if state_dict is not None:
            self._v0     = float(state_dict["v0"])
            self._v1c    = float(state_dict.get("v1c", 0.0))
            self._v1s    = 0.0
            self._last_t = float(state_dict.get("last_t", 0.0))
        else:
            self._v0     = 0.0
            self._v1c    = 0.0
            self._v1s    = 0.0
            self._last_t = -1.0   # sentinel: triggers bootstrap on first call

        # ── Diagnostics (same names as GlauertBEM / PetersHeBEM) ────────────
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = self._v0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_H_force        = 0.0
        self.last_F_prandtl      = 1.0
        self.last_v1c            = self._v1c
        self.last_v1s            = 0.0
        self.last_tau0           = 0.0   # static model — no time constant
        self.last_skew_angle_deg = 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Serialisation
    # ─────────────────────────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Serialize inflow state to a plain dict (JSON-safe)."""
        return {
            "type":   "SkewedWakeBEM2",
            "v0":     self._v0,
            "v1c":    self._v1c,
            "v1s":    0.0,
            "last_t": self._last_t,
        }

    @classmethod
    def from_definition(cls, defn, state_dict: dict | None = None,
                        **overrides) -> "SkewedWakeBEM2":
        return cls(defn, state_dict=state_dict, **overrides)

    # ─────────────────────────────────────────────────────────────────────────
    # Internal helpers  (identical to PetersHeBEM)
    # ─────────────────────────────────────────────────────────────────────────

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
        v_rel_world: np.ndarray,
        disk_normal: np.ndarray,
        R_hub: np.ndarray,
        omega_rotor: float,
        collective_rad: float,
        tilt_lon_rad: float,
        tilt_lat_rad: float,
        spin_angle: float,
        psi_skew: float = 0.0,
    ):
        """
        Strip integration with non-uniform inflow v_i(r,ψ) = v0 + v1c*(r/R)*cos(ψ-ψ_skew)
        + v1s*(r/R)*sin(ψ).  Returns (F_acc, M_acc) in world frame.
        psi_skew: azimuth of in-plane wind in disk frame (Coleman gradient orientation).
        """
        omega_abs = abs(float(omega_rotor))
        F_acc = np.zeros(3)
        M_acc = np.zeros(3)

        v_ax_eff = abs(float(np.dot(v_rel_world, disk_normal)) + v0)

        # Pre-compute Prandtl F once per radial strip (independent of azimuth).
        F_prandtl = []
        for r in self._r_stations:
            inflow_angle = math.atan2(v_ax_eff, max(omega_abs * r, 0.5))
            F_prandtl.append(self._prandtl_F(r, inflow_angle))
        self.last_F_prandtl = float(np.mean(F_prandtl))

        for i in range(self.N_AB):
            phi_az = spin_angle + self._phi_offsets[i]
            ca = math.cos(phi_az)
            sa = math.sin(phi_az)

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
                F_p = F_prandtl[j]

                r_norm    = r / self.R_TIP
                v_i_local = v0 + v1c * r_norm * math.cos(phi_az - psi_skew) + v1s * r_norm * sa
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

        return F_acc, M_acc

    # ─────────────────────────────────────────────────────────────────────────
    # Coleman inflow iteration
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _coleman_kx(v0: float, v_axial: float, v_inplane: float) -> tuple[float, float]:
        """
        Return (K_x, χ_rad) for the Coleman skewed-wake gradient.

        χ = arctan(v_inplane / |v_axial + v0|)   ∈ [0, π/2]
        K_x = tan(χ/2)                            ∈ [0, _KX_MAX]
        """
        chi = math.atan2(v_inplane, max(abs(v_axial + v0), 0.01))
        return min(math.tan(chi / 2.0), _KX_MAX), chi

    def _update_inflow(
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
        spin_angle: float,
        psi_skew: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Iterate v0 to the quasi-static Coleman fixed point.

        Algorithm
        ─────────
        1. Bootstrap v0 from the hover formula if starting near zero
           (avoids V_T = 0 singularity on first call).
        2. Relax loop:
             K_x  = tan(χ/2);  v1c = K_x·v0;  v1s = 0
             strip-integrate  →  T
             v0_ss = T / (2·ρ·A·V_T);  v0 += α·(v0_ss − v0)
        3. Return the last (F_acc, M_acc) so compute_forces() avoids a
           redundant strip integration.

        First call (cold): 60 iterations, α = 0.10 — robust to v0 = 0.
        Subsequent calls:  30 iterations, α = 0.25 — warm start converges fast.
        """
        rho_A   = self.RHO * self.disk_area
        cold    = self._last_t < 0.0
        n_iter  = 60  if cold else 30
        alpha   = 0.10 if cold else 0.25

        # Bootstrap: prevent V_T → 0 singularity when v0 ≈ 0
        if self._v0 < 1e-3:
            F0, _ = self._strip_forces(
                0.0, 0.0, 0.0,
                v_rel_world, disk_normal, R_hub, omega_rotor,
                collective_rad, tilt_lon_rad, tilt_lat_rad, spin_angle, psi_skew,
            )
            T0 = float(np.dot(F0 / self.N_AZ, disk_normal))
            self._v0 = math.sqrt(max(abs(T0), 0.1) / (2.0 * rho_A))

        F_acc = M_acc = None

        for _ in range(n_iter):
            K_x, _ = self._coleman_kx(self._v0, v_axial, v_inplane)
            self._v1c = K_x * self._v0
            self._v1s = 0.0

            F_acc, M_acc = self._strip_forces(
                self._v0, self._v1c, self._v1s,
                v_rel_world, disk_normal, R_hub, omega_rotor,
                collective_rad, tilt_lon_rad, tilt_lat_rad, spin_angle, psi_skew,
            )

            T_raw = float(np.dot(F_acc / self.N_AZ, disk_normal))
            V_T   = math.sqrt(v_inplane**2 + (v_axial + self._v0)**2 + 1e-6)
            v0_ss = max(0.0, T_raw) / (2.0 * rho_A * max(V_T, 0.1))

            err = abs(v0_ss - self._v0)
            self._v0 += alpha * (v0_ss - self._v0)
            if err < 1e-4:
                break

        return F_acc, M_acc  # type: ignore[return-value]

    # ─────────────────────────────────────────────────────────────────────────
    # Public interface
    # ─────────────────────────────────────────────────────────────────────────

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
        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # Coleman gradient orientation: azimuth of in-plane wind in the disk frame.
        if v_inplane > 0.01:
            v_ip_unit = v_inplane_vec / v_inplane
            psi_skew = math.atan2(float(np.dot(v_ip_unit, R_hub[:, 1])),
                                  float(np.dot(v_ip_unit, R_hub[:, 0])))
        else:
            psi_skew = 0.0

        # ── Iterate to quasi-static Coleman fixed point ──────────────────────
        F_acc, M_acc = self._update_inflow(
            v_rel_world, disk_normal, R_hub, omega_rotor,
            collective_rad, tilt_lon_rad, tilt_lat_rad,
            v_axial, v_inplane, spin_angle, psi_skew,
        )
        self._last_t = t

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        # ── Disk-frame scalars ────────────────────────────────────────────────
        T_disk  = float(np.dot(F_total, disk_normal))

        # ── Spin / cyclic decomposition ───────────────────────────────────────
        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # ── Diagnostics ────────────────────────────────────────────────────────
        _, chi  = self._coleman_kx(self._v0, v_axial, v_inplane)

        self.last_T              = T_disk
        self.last_v_axial        = v_axial
        self.last_v_i            = self._v0
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_M_spin         = np.array(M_spin_world)
        self.last_M_cyc          = np.array(M_cyc_world)
        self.last_H_force        = float(np.linalg.norm(F_total - T_disk * disk_normal))
        self.last_v1c            = self._v1c
        self.last_v1s            = 0.0
        self.last_tau0           = 0.0   # static model
        self.last_skew_angle_deg = math.degrees(chi)

        log.debug(
            "t=%.3f T=%.2fN v0=%.3f v1c=%.3f chi=%.1f° v_ax=%.2f v_ip=%.2f",
            t, self.last_T, self._v0, self._v1c,
            self.last_skew_angle_deg, v_axial, v_inplane,
        )

        return AeroResult(
            F_world   = np.array(F_total),
            M_orbital = np.array(M_cyc_world),
            Q_spin    = Q_spin_scalar,
            M_spin    = np.array(M_spin_world),
        )
