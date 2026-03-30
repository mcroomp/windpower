"""
aero_rotor.py — Lumped-blade BEM aerodynamic model (RotorAero)

Computes the aerodynamic wrench (force + moment) acting on the rotor hub
in the world (ENU) frame.

Aerodynamic model — SG6042 lift + Oswald drag polar:
  - Lift:       CL = CL0 + CL_alpha × α
  - Drag polar: CD = CD0 + CL²/(π·AR·Oe)  (De Schutter Eq. 25 induced drag)
  - AoA clamp:  |α| ≤ alpha_stall  (De Schutter Eq. 28 constraint)
  - Induction:  exact quadratic solution of momentum equation (De Schutter Eq. 17)
                T = 2ρA·v_i·(|v_axial| + v_i)  →  v_i = (−|v_axial| + √(v_axial² + 2T/ρA)) / 2

Force model — single-point lumped blade (De Schutter Eq. 30–31):
  Forces are evaluated at r_cp = r_root + (2/3)×span using total blade area S_w.

All default parameter values are loaded from rotor_definitions/beaupoil_2026.yaml
via rotor_definition.default().aero_kwargs().  Override by passing kwargs to __init__.

Spin dynamics:
  last_Q_spin uses an empirical model:  Q = k_drive × v_inplane − k_drag × ω²
  Equilibrium: ω_eq = sqrt(k_drive × v_inplane / k_drag)
"""

import math
import logging
import numpy as np

log = logging.getLogger(__name__)



class RotorAero:
    """
    BEM rotor aerodynamic force/moment model.

    Lift model — Weyel (2025) empirical SG6042:
        CL = CL0 + CL_alpha × α   (CL0=0.11, CL_alpha=0.87 /rad)
    Drag polar — De Schutter Eq. 25:
        CD = CD0 + CL² / (π × AR × Oe)
    Induction per De Schutter Eq. 17 (exact quadratic):
        v_i = (−|v_axial| + √(v_axial² + 2T/(ρA))) / 2

    Spin: last_Q_spin = k_drive_spin * v_inplane - k_drag_spin * omega²
    Use:  omega_spin += aero.last_Q_spin / I_spin * dt

    Parameters
    ----------
    rotor : RotorDefinition
        Rotor geometry, airfoil, and autorotation parameters.
    ramp_time : float
        Spin-up ramp duration [s] (simulation artifact, not a rotor property).
    **overrides : optional keyword overrides for any aero_kwargs() field.
    """

    def __init__(self, rotor, ramp_time: float = 5.0, **overrides):
        p = {**rotor.aero_kwargs(), **overrides}
        self.n_blades     = int(p["n_blades"])
        self.r_root       = float(p["r_root"])
        self.r_tip        = float(p["r_tip"])
        self.chord        = float(p["chord"])
        self.rho          = float(p["rho"])
        self.aspect_ratio = float(p["aspect_ratio"])
        self.oswald_eff   = float(p["oswald_eff"])
        self.CD0          = float(p["CD0"])
        self.CL0          = float(p["CL0"])
        self.CL_alpha     = float(p["CL_alpha"])
        self.K_cyc        = float(p["K_cyc"])
        self.aoa_limit    = float(p["aoa_limit"])
        self.ramp_time    = float(ramp_time)
        self.k_drive_spin = float(p["k_drive_spin"])
        self.k_drag_spin  = float(p["k_drag_spin"])

        # De Schutter lumped-blade geometry (Eq. 30–31)
        span          = self.r_tip - self.r_root
        self.r_cp     = self.r_root + (2.0 / 3.0) * span
        self.S_w      = self.n_blades * self.chord * span
        self.disk_area = math.pi * (self.r_tip ** 2 - self.r_root ** 2)

        # Telemetry — readable after each compute_forces() call
        self.last_T              = 0.0
        self.last_v_axial        = 0.0
        self.last_v_i            = 0.0
        self.last_v_inplane      = 0.0
        self.last_ramp           = 0.0
        self.last_collective_rad = 0.0
        self.last_tilt_lon       = 0.0
        self.last_tilt_lat       = 0.0
        self.last_Q_drag         = 0.0
        self.last_Q_drive        = 0.0
        self.last_H_force        = 0.0
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)
        self.last_Q_spin         = 0.0   # empirical spin torque [N·m] — use for spin ODE

    @classmethod
    def from_definition(cls, defn, **overrides) -> "RotorAero":
        """Alias for RotorAero(rotor, **overrides) — kept for backwards compatibility."""
        return cls(defn, **overrides)

    def _ramp_factor(self, t: float) -> float:
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _induced_velocity(self, T_guess: float, v_axial: float = 0.0) -> float:
        """Exact actuator-disk induced velocity (De Schutter Eq. 17)."""
        T_abs = max(abs(T_guess), 0.01)
        v_ax  = abs(v_axial)
        disc  = v_ax ** 2 + 2.0 * T_abs / (self.rho * self.disk_area)
        return max(0.0, (-v_ax + math.sqrt(disc)) / 2.0)

    def _bem_integrate(self, v_eff: float, omega_abs: float,
                       collective_rad: float) -> tuple:
        """De Schutter (2018) lumped-blade BEM — Eq. 25, 30–31."""
        v_tan = omega_abs * self.r_cp
        v_loc = math.sqrt(v_tan ** 2 + v_eff ** 2)
        if v_loc < 0.5:
            return 0.0, 0.0, 0.0

        inflow_ang  = math.atan2(v_eff, v_tan)
        aoa         = inflow_ang + collective_rad
        aoa_clamped = max(-self.aoa_limit, min(self.aoa_limit, aoa))

        CL = self.CL0 + self.CL_alpha * aoa_clamped
        CD = self.CD0 + CL ** 2 / (math.pi * self.aspect_ratio * self.oswald_eff)
        q  = 0.5 * self.rho * v_loc ** 2

        T = q * self.S_w * (CL * math.cos(inflow_ang) - CD * math.sin(inflow_ang))
        T = max(0.0, T)

        Q = q * self.S_w * self.r_cp * (CL * math.sin(inflow_ang) - CD * math.cos(inflow_ang))
        return T, max(0.0, Q), min(0.0, Q)

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
    ) -> np.ndarray:
        """
        Compute aerodynamic wrench [Fx,Fy,Fz,Mx,My,Mz] in world ENU [N, N·m].

        After the call:
          last_Q_spin  — empirical spin torque; use for spin ODE (omega += Q/I * dt)
          last_M_spin  — spin-axis moment vector; subtract from total to get orbital moment
          last_v_inplane — in-plane wind speed [m/s]
        """
        ramp = self._ramp_factor(t)

        disk_normal   = R_hub[:, 2]
        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))
        omega_abs     = abs(omega_rotor)

        # Coupled BEM + induction iteration (3 passes)
        v_i = 0.0
        for _ in range(3):
            T_raw, Q_pos, Q_neg = self._bem_integrate(v_axial + v_i, omega_abs, collective_rad)
            v_i = self._induced_velocity(T_raw, v_axial)
        T_raw, Q_pos, Q_neg = self._bem_integrate(v_axial + v_i, omega_abs, collective_rad)

        T       = T_raw * ramp
        F_world = T * disk_normal

        # H-force (in-plane drag from advancing/retreating asymmetry)
        H = 0.0
        if v_inplane > 0.01 and omega_abs > 0.1:
            mu            = v_inplane / (omega_abs * self.r_tip)
            H             = 0.5 * mu * T * ramp
            F_world       = F_world + H * (v_inplane_vec / v_inplane)

        # Spin moment
        Q_total      = Q_pos + Q_neg
        spin_sign    = float(np.sign(omega_rotor)) if omega_rotor != 0.0 else 1.0
        M_spin_world = Q_total * ramp * spin_sign * disk_normal
        Q_drive      = Q_pos * ramp * spin_sign
        Q_drag       = Q_neg * ramp * spin_sign

        # Cyclic moments from swashplate tilt
        from swashplate import _PITCH_GAIN_RAD as pitch_gain
        tilt_lon_rad = tilt_lon * pitch_gain
        tilt_lat_rad = tilt_lat * pitch_gain

        _EAST  = np.array([1.0, 0.0, 0.0])
        _ep    = _EAST - np.dot(_EAST, disk_normal) * disk_normal
        if np.linalg.norm(_ep) > 1e-6:
            _bx = _ep / np.linalg.norm(_ep)
        else:
            _NORTH = np.array([0.0, 1.0, 0.0])
            _np2   = _NORTH - np.dot(_NORTH, disk_normal) * disk_normal
            _bx    = _np2 / np.linalg.norm(_np2)
        R_orb       = np.column_stack([_bx, np.cross(disk_normal, _bx), disk_normal])
        M_cyc_world = R_orb @ np.array([-self.K_cyc * tilt_lon_rad * T,
                                          self.K_cyc * tilt_lat_rad * T,
                                          0.0])

        M_world = M_spin_world + M_cyc_world

        result    = np.zeros(6)
        result[:3] = F_world
        result[3:] = M_world

        self.last_M_spin         = M_spin_world.copy()
        self.last_M_cyc          = M_cyc_world.copy()
        self.last_T              = T
        self.last_v_axial        = v_axial
        self.last_v_i            = v_i
        self.last_v_inplane      = v_inplane
        self.last_ramp           = ramp
        self.last_collective_rad = collective_rad
        self.last_tilt_lon       = tilt_lon
        self.last_tilt_lat       = tilt_lat
        self.last_Q_drag         = float(Q_drag)
        self.last_Q_drive        = float(Q_drive)
        self.last_H_force        = float(H)
        self.last_Q_spin         = (self.k_drive_spin * v_inplane
                                    - self.k_drag_spin * omega_abs ** 2)

        log.debug("t=%.3f T=%.2fN H=%.2fN v_axial=%.2f v_i=%.2f ramp=%.2f",
                  t, T, H, v_axial, v_i, ramp)
        return result

    def compute_anti_rotation_moment(self, omega_rotor: float,
                                     gear_ratio: float = 80.0 / 44.0) -> float:
        """
        Counter-torque from GB4008 motor (Mz about disk axis).

        The motor is geared to the rotor axle at gear_ratio = 80/44.  Motor shaft
        speed is mechanically locked to gear_ratio × omega_rotor (opposite sign), so
        no speed sensor or closed-loop speed control is needed.  The motor actively
        drives the electronics assembly against bearing drag; the ESC runs at a fixed
        setpoint.  Counter-torque scales with rotor speed through the gear ratio.

        Simplified model: M = K_motor × gear_ratio × omega_rotor
        K_motor is not yet calibrated — placeholder value used here.

        Not called in the current single-body model — motor torque is an internal
        force and cancels in the equations of motion.  Reserved for the future
        two-body model (separate spinning-hub and stationary-electronics bodies).
        """
        # Placeholder motor torque constant [N·m·s/rad at motor shaft]
        # To calibrate: measure stall torque at known omega_rotor and back-calculate.
        _K_motor = 0.5   # N·m·s/rad — UNCALIBRATED ESTIMATE
        return -_K_motor * gear_ratio * omega_rotor
