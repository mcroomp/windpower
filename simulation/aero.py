"""
aero.py — De Schutter (2018) lumped-blade aerodynamic model for RAWES

Computes the aerodynamic wrench (force + moment) acting on the rotor hub
in the world (ENU) frame, given:
  - Swashplate-derived blade pitch angles
  - Hub pose (rotation matrix) and velocity
  - Ambient wind vector
  - Simulation time (for spin-up ramp)

Aerodynamic model — Weyel (2025) empirical SG6042 lift + De Schutter (2018) drag polar:
  - Lift:       CL = CL0 + CL_alpha × α           (Weyel Eq. empirical, SG6042 at Re≈127k)
                CL0 = 0.11 (camber offset), CL_alpha = 0.87 /rad (6× below thin-plate theory)
  - Drag polar: CD = CD0 + CL²/(π·AR·Oe)          (De Schutter Eq. 25 induced drag)
  - AoA clamp:  |α| ≤ 15° (De Schutter Eq. 28 constraint)
  - Induction:  exact quadratic solution of momentum equation (De Schutter Eq. 17)
                T = 2ρA·v_i·(|v_axial| + v_i)  →  v_i = (−|v_axial| + √(v_axial² + 2T/ρA)) / 2

Force model — single-point lumped blade (De Schutter Eq. 30–31):
  Forces are evaluated at the aerodynamic centre of pressure
      r_cp = r_root + (2/3)×(r_tip − r_root)
  using total blade area S_w = N_blades × chord × span, not integrated over radial strips.
  Per De Schutter: r'_{cp,k} is at "2/3 of span from root" for each wing k.

Geometry (4-blade, 2 m blade, actual hardware):
  ρ = 1.22 kg/m³, chord = 0.15 m, R_root = 0.5 m, R_tip = 2.5 m, N = 4
  span = R_tip − R_root = 2.0 m,  S_w = N × chord × span = 1.2 m²
  r_cp = 0.5 + (2/3)×2.0 = 1.833 m
  AR = span / chord = 2.0 / 0.15 = 13.3
"""

import math
import logging
import numpy as np
from typing import Optional

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Default physical parameters
# ---------------------------------------------------------------------------
_DEFAULTS = dict(
    n_blades     = 4,
    r_root       = 0.5,      # blade root radius [m]
    r_tip        = 2.5,      # blade tip radius [m]
    chord        = 0.15,     # blade chord [m]
    rho          = 1.22,     # air density [kg/m³]
    aspect_ratio = 13.3,     # blade span / chord = 2.0 / 0.15  (De Schutter R=12 for 1.5m/0.125m)
    oswald_eff   = 0.8,      # Oswald efficiency (De Schutter Table I)
    CD0          = 0.01,     # zero-lift drag coefficient (De Schutter Table I)
    CL0          = 0.11,     # zero-AoA lift coefficient (Weyel 2025, SG6042 camber)
    CL_alpha     = 0.87,     # lift slope [/rad] (Weyel 2025, empirical SG6042 at Re≈127k)
    K_cyc        = 0.4,      # cyclic moment scaling factor
    aoa_limit    = 0.26,     # AoA clamp [rad] ≈ ±15° (De Schutter Eq. 28)
    ramp_time    = 5.0,      # spin-up ramp duration [s]
)


class RotorAero:
    """
    BEM rotor aerodynamic force/moment model.

    Lift model — Weyel (2025) empirical SG6042:
        CL = CL0 + CL_alpha × α   (CL0=0.11, CL_alpha=0.87 /rad)
    Drag polar — De Schutter Eq. 25:
        CD = CD0 + CL² / (π × AR × Oe)
    Induction per De Schutter Eq. 17 (exact quadratic):
        v_i = (−|v_axial| + √(v_axial² + 2T/(ρA))) / 2

    Parameters
    ----------
    **kwargs : override any of the default physical parameters listed in
               _DEFAULTS above.
    """

    def __init__(self, **kwargs):
        p = {**_DEFAULTS, **kwargs}
        self.n_blades    = int(p["n_blades"])
        self.r_root      = float(p["r_root"])
        self.r_tip       = float(p["r_tip"])
        self.chord       = float(p["chord"])
        self.rho         = float(p["rho"])
        self.aspect_ratio= float(p["aspect_ratio"])
        self.oswald_eff  = float(p["oswald_eff"])
        self.CD0         = float(p["CD0"])
        self.CL0         = float(p["CL0"])
        self.CL_alpha    = float(p["CL_alpha"])
        self.K_cyc       = float(p["K_cyc"])
        self.aoa_limit   = float(p["aoa_limit"])
        self.ramp_time   = float(p["ramp_time"])

        # De Schutter lumped-blade geometry (Eq. 30–31)
        # r_cp: centre of pressure at 2/3 of span from root (De Schutter r'_{cp,k})
        # S_w:  total blade area = N_blades × chord × span
        span          = self.r_tip - self.r_root
        self.r_cp     = self.r_root + (2.0 / 3.0) * span
        self.S_w      = self.n_blades * self.chord * span

        # Annular disk area for induction (De Schutter A_K = π(R_tip² − R_root²))
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
        # Separated moment components (used by mediator to split orbital vs spin)
        self.last_M_spin         = np.zeros(3)
        self.last_M_cyc          = np.zeros(3)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ramp_factor(self, t: float) -> float:
        """Linear ramp from 0 to 1 over ramp_time seconds."""
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _induced_velocity(self, T_guess: float, v_axial: float = 0.0) -> float:
        """
        Exact actuator-disk induced velocity from momentum theory.

        Solves: T = 2ρA · v_i · (|v_axial| + v_i)

        Quadratic in v_i: 2ρA·v_i² + 2ρA·|v_axial|·v_i − T = 0
        Positive root:    v_i = (−|v_axial| + √(v_axial² + 2T/(ρA))) / 2

        This always converges (discriminant is always positive for T > 0).
        The induction factor a = v_i/|v_axial| ≤ 0.5 provided T ≤ 1.5·ρA·v_axial²
        (De Schutter AD validity condition Eq. 19: a ≤ 0.5).
        disk_area here is the annular area A_K = π(R_tip² − R_root²) per De Schutter Eq. 16.

        Parameters
        ----------
        T_guess  : float   Thrust estimate [N]  (T > 0)
        v_axial  : float   Axial wind component through the disk [m/s]

        Returns
        -------
        float   Induced velocity v_i ≥ 0 [m/s]
        """
        T_abs  = max(abs(T_guess), 0.01)
        v_ax   = abs(v_axial)
        # Discriminant is always positive: v_ax² + 2T/(ρA) > 0
        disc   = v_ax ** 2 + 2.0 * T_abs / (self.rho * self.disk_area)
        return max(0.0, (-v_ax + math.sqrt(disc)) / 2.0)

    def _bem_integrate(
        self,
        v_eff:         float,
        omega_abs:     float,
        collective_rad: float,
    ) -> tuple:
        """
        De Schutter (2018) lumped-blade aerodynamic model — Eq. 25, 30–31.

        Forces are evaluated at the single aerodynamic centre of pressure
            r_cp = r_root + (2/3)×(r_tip − r_root)
        using total blade area S_w = N_blades × chord × span.

        This matches De Schutter's formulation: r'_{cp,k} at 2/3 of span from
        root for each wing k, with blade area S_w as the aerodynamic reference.
        The previous 20-strip radial BEM integration applied the whole-wing CL
        formula per strip, over-weighting the high-q outer strips and giving
        thrust magnitudes ~5–7× above De Schutter's reference values.

        Parameters
        ----------
        v_eff          : effective axial inflow (v_axial + v_i) [m/s]
        omega_abs      : |rotor spin rate| [rad/s]
        collective_rad : collective blade pitch [rad]

        Returns
        -------
        (T, Q_positive, Q_negative) : thrust [N] and torque components [N·m]
        """
        v_tan = omega_abs * self.r_cp
        v_loc = math.sqrt(v_tan ** 2 + v_eff ** 2)

        if v_loc < 0.5:
            return 0.0, 0.0, 0.0

        # Inflow angle and angle of attack at r_cp
        inflow_ang  = math.atan2(v_eff, v_tan)
        aoa         = inflow_ang + collective_rad
        aoa_clamped = max(-self.aoa_limit, min(self.aoa_limit, aoa))

        # Weyel (2025) empirical SG6042 lift + De Schutter Eq. 25 drag polar
        CL = self.CL0 + self.CL_alpha * aoa_clamped
        CD = self.CD0 + CL ** 2 / (math.pi * self.aspect_ratio * self.oswald_eff)

        q = 0.5 * self.rho * v_loc ** 2

        # Thrust — axial component of blade force (De Schutter Eq. 30–31)
        # T = q · S_w · (CL·cos φ − CD·sin φ)
        T = q * self.S_w * (CL * math.cos(inflow_ang) - CD * math.sin(inflow_ang))
        T = max(0.0, T)

        # Spin torque at r_cp — drives rotation when CL·sin φ > CD·cos φ
        Q = q * self.S_w * self.r_cp * (CL * math.sin(inflow_ang) - CD * math.cos(inflow_ang))
        Q_positive = max(0.0, Q)
        Q_negative = min(0.0, Q)

        return T, Q_positive, Q_negative

    # ------------------------------------------------------------------
    # Main computation
    # ------------------------------------------------------------------

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
        Compute aerodynamic wrench on the rotor hub in the world (ENU) frame.

        Algorithm:
          1. Resolve axial and in-plane relative wind at disk.
          2. Coupled BEM + induction iteration (3 passes):
               v_i = 0 → BEM → T → quadratic(T, v_axial) → v_i → repeat.
          3. Thrust along disk normal; drag torque about disk axis.
          4. H-force (in-plane drag from advance ratio asymmetry).
          5. Cyclic moments from swashplate tilt (proportional to thrust).
          6. Apply spin-up ramp.

        Parameters
        ----------
        collective_rad : float   Collective blade pitch [rad]
        tilt_lon       : float   Normalised longitudinal swashplate tilt [-1, 1]
        tilt_lat       : float   Normalised lateral swashplate tilt [-1, 1]
        R_hub          : (3,3)   Rotation matrix body→world
        v_hub_world    : (3,)    Hub velocity in world ENU [m/s]
        omega_rotor    : float   Rotor angular speed [rad/s]
        wind_world     : (3,)    Ambient wind in world ENU [m/s]
        t              : float   Simulation time [s]

        Returns
        -------
        np.ndarray, shape (6,)
            [Fx, Fy, Fz, Mx, My, Mz] in world (ENU) frame [N, N·m]
        """
        ramp = self._ramp_factor(t)

        # ----------------------------------------------------------
        # 1. Disk normal and relative wind
        # ----------------------------------------------------------
        disk_normal = R_hub[:, 2]   # body Z expressed in world

        v_rel_world  = wind_world - v_hub_world
        v_axial      = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        omega_abs = abs(omega_rotor)

        # ----------------------------------------------------------
        # 2. Coupled BEM + induction iteration
        #    Start with v_i = 0; iterate: BEM → T → quadratic → v_i
        #    3 passes are sufficient for convergence at typical conditions.
        # ----------------------------------------------------------
        v_i = 0.0
        T_raw      = 0.0
        Q_positive = 0.0
        Q_negative = 0.0

        for _ in range(3):
            v_eff = v_axial + v_i
            T_raw, Q_positive, Q_negative = self._bem_integrate(
                v_eff, omega_abs, collective_rad
            )
            v_i = self._induced_velocity(T_raw, v_axial)

        # Final BEM pass with converged v_i
        v_eff = v_axial + v_i
        T_raw, Q_positive, Q_negative = self._bem_integrate(
            v_eff, omega_abs, collective_rad
        )

        # ----------------------------------------------------------
        # 3. Thrust force vector
        # ----------------------------------------------------------
        T = T_raw * ramp

        F_world = T * disk_normal

        # ----------------------------------------------------------
        # 4. H-force (in-plane drag from advancing/retreating asymmetry)
        #    H ≈ μ·T/2 where μ = v_inplane / (ω·R_tip) is advance ratio
        # ----------------------------------------------------------
        H = 0.0
        if v_inplane > 0.01 and omega_abs > 0.1:
            mu = v_inplane / (omega_abs * self.r_tip)
            H  = 0.5 * mu * T * ramp
            v_inplane_hat = v_inplane_vec / v_inplane
            F_world = F_world + H * v_inplane_hat

        # ----------------------------------------------------------
        # 5. Spin torque (BEM dQ integral)
        # ----------------------------------------------------------
        Q_total      = Q_positive + Q_negative
        spin_sign    = float(np.sign(omega_rotor)) if omega_rotor != 0.0 else 1.0
        M_spin_world = Q_total * ramp * spin_sign * disk_normal

        Q_drive = Q_positive * ramp * spin_sign
        Q_drag  = Q_negative * ramp * spin_sign

        # ----------------------------------------------------------
        # 6. Cyclic moments from swashplate tilt
        # ----------------------------------------------------------
        from swashplate import _PITCH_GAIN_RAD as pitch_gain

        tilt_lon_rad = tilt_lon * pitch_gain
        tilt_lat_rad = tilt_lat * pitch_gain

        # Body-frame cyclic moments in ENU-convention body frame (body Z = disk Up)
        #   tilt_lon > 0 = forward/North → disk normal tilts toward ENU +Y (North)
        #     Rx(θ) tilts body Z toward −body Y, so need Mx < 0 → Mx = −K_cyc·tilt_lon·T
        #   tilt_lat > 0 = right/East   → disk normal tilts toward ENU +X (East)
        #     Ry(θ) tilts body Z toward +body X, so My = +K_cyc·tilt_lat·T
        Mx_body     = -self.K_cyc * tilt_lon_rad * T
        My_body     =  self.K_cyc * tilt_lat_rad * T
        M_cyc_world = R_hub @ np.array([Mx_body, My_body, 0.0])

        M_world = M_spin_world + M_cyc_world

        result = np.zeros(6)
        result[0:3] = F_world
        result[3:6] = M_world

        # Telemetry
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

        log.debug(
            "t=%.3f T=%.2fN H=%.2fN v_axial=%.2f v_i=%.2f ramp=%.2f F=%s M=%s",
            t, T, H, v_axial, v_i, ramp, F_world.round(2), M_world.round(2),
        )

        return result

    def compute_anti_rotation_moment(
        self,
        esc_normalized: float,
        omega_rotor:    float,
        T:              float,
    ) -> float:
        """
        Counter-torque contribution from the GB4008 motor (Mz).

        NOTE: not applied in the current single-body model — motor and bearing
        drag are internal forces that cancel.  Retained for future two-body model.
        """
        _K_drag_nom = 0.1
        max_motor_torque = (_K_drag_nom * 2.0 * 0.5 * 1.22
                            * self.disk_area * 70.0 ** 2 * self.r_tip)
        esc_cmd = max(0.0, esc_normalized)
        return esc_cmd * max_motor_torque * np.sign(omega_rotor)


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.WARNING)
    print("RotorAero smoke test (De Schutter formulas)")
    print(f"  CL_alpha_3d = {RotorAero().CL_alpha_3d:.4f} /rad  "
          f"(De Schutter R=12 → {2*math.pi/(1+2/12):.4f} /rad)")

    aero = RotorAero()
    R    = np.eye(3)
    v_h  = np.zeros(3)
    wind = np.array([10.0, 0.0, 0.0])

    forces = aero.compute_forces(0.0, 0.0, 0.0, R, v_h, 28.0, wind, t=0.0)
    assert np.allclose(forces, 0.0, atol=1e-10), f"t=0 forces not zero: {forces}"
    print("  t=0 ramp: OK (forces=zero)")

    forces_ss = aero.compute_forces(0.1, 0.0, 0.0, R, v_h, 28.0, wind, t=10.0)
    print(f"  Steady-state: F={forces_ss[:3].round(2)}  M={forces_ss[3:].round(2)}")
    assert forces_ss[2] > 0, f"Expected positive Fz, got {forces_ss[2]}"
    print("  Fz > 0: OK")
    assert abs(forces_ss[5]) < abs(forces_ss[2]), \
        f"|Mz|={abs(forces_ss[5]):.2f} should be < |Fz|={abs(forces_ss[2]):.2f}"
    print(f"  |Mz| < |Fz|: OK")

    forces_cyc = aero.compute_forces(0.1, 0.0, 0.5, R, v_h, 28.0, wind, t=10.0)
    assert forces_cyc[3] != forces_ss[3], "Cyclic tilt should change Mx"
    print("  Cyclic tilt changes Mx: OK")

    # Induction should be physically valid (a < 0.5) for axial flow
    aero2 = RotorAero()
    aero2.compute_forces(0.1, 0, 0, np.eye(3), np.zeros(3), 28.0,
                         np.array([0.0, 0.0, 10.0]), t=10.0)
    a = aero2.last_v_i / abs(aero2.last_v_axial) if abs(aero2.last_v_axial) > 0.1 else 0
    print(f"  Induction factor a={a:.3f} (axial flow, should be < 0.5)")
    assert a < 0.5, f"Induction factor {a:.3f} out of range — quadratic solver broken"
    print("All smoke tests passed.")
    sys.exit(0)
