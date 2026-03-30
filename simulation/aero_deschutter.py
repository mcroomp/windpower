"""
aero_deschutter.py — Per-blade strip-theory BEM aerodynamic model (DeSchutterAero)

Computes the aerodynamic wrench (force + moment) acting on the rotor hub
in the world (ENU) frame using De Schutter et al. (2018) per-blade formulation
with multi-strip radial integration.

Per-blade model:
  - N_BLADES blades sampled at N_AZ equally-spaced azimuths (revolution averaging)
  - N_RADIAL radial strips per blade from R_ROOT to R_TIP
  - Each strip sees its own rotational velocity omega × r_i
  - AoA per strip: α = −ua_normal / ua_chord  (Kutta-Joukowski sign convention)
  - Lift direction: Kutta-Joukowski cross product (ua × e_span)
  - Cyclic moments emerge from per-blade force asymmetry — no K_cyc needed

Advantages over RotorAero:
  - Correct H-force in any disk orientation including edgewise flight
  - Natural cyclic moment distribution from advancing/retreating blade physics
  - Strip theory captures radially-varying inflow angle

Spin dynamics:
  last_Q_spin uses an empirical model:  Q = k_drive × v_inplane − k_drag × ω²
  Equilibrium: ω_eq = sqrt(k_drive × v_inplane / k_drag)
  Fitted so ω_eq ≈ 20 rad/s at v_inplane ≈ 5 m/s (tethered hover, 30° elevation, 10 m/s wind).
"""

import math
import logging
import numpy as np

log = logging.getLogger(__name__)


class DeSchutterAero:
    """
    Per-blade strip-theory aerodynamic model — De Schutter et al. 2018.

    Forces are summed over N_BLADES blades at N_AZ equally-spaced azimuthal
    reference positions and averaged, giving smooth revolution-averaged forces.
    Each blade is integrated over N_RADIAL radial strips from R_ROOT to R_TIP.

    Spin: last_Q_spin = k_drive_spin * v_inplane - k_drag_spin * omega²
    Use:  omega_spin += aero.last_Q_spin / I_spin * dt

    Parameters
    ----------
    rotor : RotorDefinition
        Rotor geometry, airfoil, and autorotation parameters.
    ramp_time : float
        Spin-up ramp duration [s] (simulation artifact, not a rotor property).
    n_radial : int
        Number of radial BEM strips per blade (default 8).
    **overrides : optional keyword overrides for any aero_kwargs() field.
    """

    N_AZ          = 8            # azimuthal sample points for revolution averaging
    CP_FRAC       = 2.0 / 3.0   # bootstrap CP at 2/3 of span from root
    PITCH_MAX_DEG = 15.0         # max normalised cyclic → pitch [deg]

    def __init__(self, rotor, ramp_time: float = 5.0, n_radial: int = 8, **overrides):
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
        self.k_drive_spin = float(p["k_drive_spin"])
        self.k_drag_spin  = float(p["k_drag_spin"])

        span            = self.R_TIP - self.R_ROOT
        self.S_blade    = span * self.CHORD
        self.AR         = float(p.get("aspect_ratio") or span ** 2 / self.S_blade)
        self.R_CP       = self.R_ROOT + self.CP_FRAC * span   # bootstrap only
        self.pitch_gain = math.radians(self.PITCH_MAX_DEG)
        self.disk_area  = math.pi * (self.R_TIP ** 2 - self.R_ROOT ** 2)

        # Strip theory: N_RADIAL equally-spaced stations
        self.N_RADIAL    = int(n_radial)
        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array([
            self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)
        ])
        self._S_elem = self.CHORD * self._dr   # planform area per strip [m²]

        # Diagnostics
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_collective_rad = 0.0
        self.last_tilt_lon       = 0.0
        self.last_tilt_lat       = 0.0
        self.last_Q_spin         = 0.0   # empirical spin torque [N·m] — use for spin ODE
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_Q_drive        = 0.0
        self.last_Q_drag         = 0.0
        self.last_H_force        = 0.0

    @classmethod
    def from_definition(cls, defn, **overrides) -> "DeSchutterAero":
        """Alias for DeSchutterAero(rotor, **overrides) — kept for backwards compatibility."""
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
    ) -> np.ndarray:
        """
        Compute aerodynamic wrench [Fx,Fy,Fz,Mx,My,Mz] in world ENU [N, N·m].

        After the call:
          last_Q_spin  — empirical spin torque; use for spin ODE (omega += Q/I * dt)
          last_M_spin  — spin-axis moment vector; subtract from total to get orbital moment
          last_v_inplane — in-plane wind speed [m/s]
        """
        ramp = self._ramp_factor(t)

        disk_normal = R_hub[:, 2]
        spin_sign   = float(np.sign(omega_rotor)) if omega_rotor != 0.0 else 1.0
        omega_abs   = abs(float(omega_rotor))

        tilt_lon_rad = tilt_lon * self.pitch_gain
        tilt_lat_rad = tilt_lat * self.pitch_gain

        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # Bootstrap induction (3 passes at representative CP radius)
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

        # Per-blade, per-strip accumulation
        F_acc       = np.zeros(3)
        M_acc       = np.zeros(3)
        Q_drive_acc = 0.0
        Q_drag_acc  = 0.0
        n_evaluated = 0

        for az in range(self.N_AZ):
            phi_base = spin_angle + az * (2.0 * math.pi / self.N_AZ)

            for k in range(self.N_BLADES):
                phi_k = phi_base + k * (2.0 * math.pi / self.N_BLADES)
                ca, sa = math.cos(phi_k), math.sin(phi_k)

                e_span_body  = np.array([ca, sa, 0.0])
                e_span_world = R_hub @ e_span_body

                p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
                cp_k = math.cos(p_k)
                sp_k = math.sin(p_k)
                e_chord_world  = R_hub @ np.array([-sa * cp_k,  ca * cp_k,  sp_k])
                e_normal_world = R_hub @ np.array([ sa * sp_k, -ca * sp_k,  cp_k])
                e_tang         = np.cross(disk_normal, e_span_world)

                for r_i in self._r_stations:
                    r_cp_world = R_hub @ (r_i * e_span_body)
                    v_rot      = omega_rotor * np.cross(disk_normal, r_cp_world)
                    ua         = wind_world - v_hub_world - v_rot - v_i_vec

                    ua_norm = float(np.linalg.norm(ua))
                    if ua_norm < 0.5:
                        continue

                    ua_chord  = float(np.dot(ua, e_chord_world))
                    ua_normal = float(np.dot(ua, e_normal_world))
                    if abs(ua_chord) < 1e-6:
                        continue

                    # e_chord points in direction of blade motion → ua_chord < 0
                    # Negating restores correct sign: positive collective → positive AoA
                    alpha_k = float(np.clip(-ua_normal / ua_chord,
                                            -self.AOA_LIMIT, self.AOA_LIMIT))

                    CL_k  = self.CL0 + self.CL_ALPHA * alpha_k
                    CD_k  = self.CD0 + CL_k ** 2 / (math.pi * self.AR * self.OSWALD)
                    q_dyn = 0.5 * self.RHO * ua_norm ** 2 * self._S_elem

                    lift_raw = np.cross(ua, e_span_world)
                    lift_norm = float(np.linalg.norm(lift_raw))
                    if lift_norm < 1e-9:
                        continue
                    e_lift = lift_raw / lift_norm

                    F_k = q_dyn * (CL_k * e_lift + CD_k * (ua / ua_norm))
                    M_k = np.cross(r_cp_world, F_k)

                    F_acc += F_k
                    M_acc += M_k

                    Q_k = float(np.dot(F_k, e_tang)) * r_i * spin_sign
                    if Q_k >= 0:
                        Q_drive_acc += Q_k
                    else:
                        Q_drag_acc  += Q_k

                    n_evaluated += 1

        if n_evaluated == 0:
            self.last_M_spin = np.zeros(3)
            self.last_M_cyc  = np.zeros(3)
            self.last_Q_spin = 0.0
            return np.zeros(6)

        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale
        Q_drive = Q_drive_acc * scale
        Q_drag  = Q_drag_acc  * scale

        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        self.last_T              = float(np.dot(F_total, disk_normal))
        self.last_v_axial        = v_axial
        self.last_v_i            = v_i
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_collective_rad = collective_rad
        self.last_tilt_lon       = tilt_lon
        self.last_tilt_lat       = tilt_lat
        self.last_Q_spin         = (self.k_drive_spin * v_inplane
                                    - self.k_drag_spin * omega_abs ** 2)
        self.last_Q_drive        = Q_drive
        self.last_Q_drag         = Q_drag
        self.last_H_force        = float(np.linalg.norm(
            F_total - self.last_T * disk_normal))
        self.last_M_spin         = M_spin_world.copy()
        self.last_M_cyc          = M_cyc_world.copy()

        out    = np.zeros(6)
        out[:3] = F_total
        out[3:] = M_total
        return out
