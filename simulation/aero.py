"""
aero.py — Simplified Blade Element Momentum aerodynamic model for RAWES

Computes the aerodynamic wrench (force + moment) acting on the rotor hub
in the world (ENU) frame, given:
  - Swashplate-derived blade pitch angles
  - Hub pose (rotation matrix) and velocity from MBDyn
  - Ambient wind vector
  - Simulation time (for spin-up ramp)

Model assumptions:
  - Simplified BEM: each annular strip contributes independently (no wake swirl)
  - Linear aerodynamics (CL = CL0 + CLα·AoA) valid for AoA < ±15°
  - Rotor drag torque approximated as fraction of thrust (simple figure of merit)
  - Cyclic moments from swashplate tilt are proportional to thrust
  - No flap-lag coupling or blade flexibility
  - Induced velocity from actuator disk theory (Rankine-Froude)

Physical parameters from CLAUDE.md / Weyel 2025 (adapted for 4-blade, 2m geometry):
  ρ = 1.22 kg/m³, chord = 0.15 m, R_root = 0.5 m, R_tip = 2.5 m, N_blades = 4
"""

import math
import logging
import numpy as np
from typing import Optional

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Default physical parameters (match CLAUDE.md)
# ---------------------------------------------------------------------------
_DEFAULTS = dict(
    n_blades   = 4,
    r_root     = 0.5,      # blade root radius [m]
    r_tip      = 2.5,      # blade tip radius [m]
    chord      = 0.15,     # blade chord [m]
    rho        = 1.22,     # air density [kg/m³]
    CL0        = 0.11,     # zero-AoA lift coefficient
    CL_alpha   = 0.87,     # lift curve slope [1/rad]
    CD0        = 0.007,    # zero-lift drag
    CD_alpha   = 0.00013,  # drag polar coefficient [1/rad²]
    n_strips   = 20,       # BEM integration strips
    K_drag     = 0.1,      # drag torque fraction: Q_drag = K_drag * T * r_tip
    K_auto     = 0.00684,  # autorotative driving torque coefficient
                           # tuned so Q_drive = Q_drag at nominal conditions:
                           # v_inplane=10 m/s, omega=28 rad/s, T≈164 N
                           # Q_drive = K_auto * rho * disk_area * v_inplane² * r_tip
    K_cyc      = 0.4,      # cyclic moment scaling factor
    aoa_limit  = 0.26,     # AoA clamp [rad] ≈ ±15° (linear model validity)
    ramp_time  = 5.0,      # spin-up ramp duration [s]
)


class RotorAero:
    """
    Simplified BEM rotor aerodynamic force/moment model.

    Parameters
    ----------
    **kwargs : override any of the default physical parameters listed in
               _DEFAULTS above.
    """

    def __init__(self, **kwargs):
        p = {**_DEFAULTS, **kwargs}
        self.n_blades  = int(p["n_blades"])
        self.r_root    = float(p["r_root"])
        self.r_tip     = float(p["r_tip"])
        self.chord     = float(p["chord"])
        self.rho       = float(p["rho"])
        self.CL0       = float(p["CL0"])
        self.CL_alpha  = float(p["CL_alpha"])
        self.CD0       = float(p["CD0"])
        self.CD_alpha  = float(p["CD_alpha"])
        self.n_strips  = int(p["n_strips"])
        self.K_drag    = float(p["K_drag"])
        self.K_auto    = float(p["K_auto"])
        self.K_cyc     = float(p["K_cyc"])
        self.aoa_limit = float(p["aoa_limit"])
        self.ramp_time = float(p["ramp_time"])

        # Disk area = π * R_tip²
        self.disk_area = math.pi * self.r_tip ** 2

        # BEM strip edges (r_root to r_tip, n_strips+1 boundaries)
        self._r_edges  = np.linspace(self.r_root, self.r_tip, self.n_strips + 1)
        # Strip centres and widths
        self._r_mid    = 0.5 * (self._r_edges[:-1] + self._r_edges[1:])
        self._dr       = self._r_edges[1:] - self._r_edges[:-1]

        # Last computed internals — readable after each compute_forces() call
        # for telemetry logging without changing the return API.
        self.last_T             = 0.0
        self.last_v_axial       = 0.0
        self.last_v_i           = 0.0
        self.last_v_inplane     = 0.0
        self.last_ramp          = 0.0
        self.last_collective_rad= 0.0
        self.last_tilt_lon      = 0.0
        self.last_tilt_lat      = 0.0
        self.last_Q_drag        = 0.0
        self.last_Q_drive       = 0.0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ramp_factor(self, t: float) -> float:
        """
        Linear ramp from 0 to 1 over ramp_time seconds.
        Prevents impulsive loads at simulation start before steady autorotation.
        """
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _induced_velocity(self, T_guess: float) -> float:
        """
        Actuator disk induced velocity from momentum theory.
            T = 2 * ρ * A * v_i * (v_i + v_axial)
        For hover-like conditions, the simple formula:
            v_i ≈ sqrt(T / (2 * ρ * A))
        is used as a first-order approximation.

        Parameters
        ----------
        T_guess : float   Thrust estimate [N]

        Returns
        -------
        float   Induced velocity [m/s]
        """
        T_abs = max(abs(T_guess), 0.01)   # avoid sqrt(negative) or zero
        return math.sqrt(T_abs / (2.0 * self.rho * self.disk_area))

    # ------------------------------------------------------------------
    # Main computation
    # ------------------------------------------------------------------

    def compute_forces(
        self,
        collective_rad: float,
        tilt_lon:       float,
        tilt_lat:       float,
        R_hub:          np.ndarray,   # (3,3) body→world rotation matrix
        v_hub_world:    np.ndarray,   # (3,) hub CoM velocity in world ENU [m/s]
        omega_rotor:    float,        # rotor spin rate [rad/s] (+ = CCW from above)
        wind_world:     np.ndarray,   # (3,) ambient wind in world ENU [m/s]
        t:              float,        # simulation time [s]
    ) -> np.ndarray:
        """
        Compute aerodynamic wrench on the rotor hub in the world (ENU) frame.

        Algorithm:
          1. Determine rotor disk normal from hub rotation matrix.
          2. Compute relative wind at disk (ambient – hub velocity).
          3. Estimate induced velocity using actuator disk theory.
          4. BEM integration over 20 radial strips.
          5. Thrust along disk normal; drag torque about disk axis.
          6. Cyclic moments from swashplate tilt (proportional to thrust).
          7. Apply spin-up ramp for first ramp_time seconds.

        Parameters
        ----------
        collective_rad : float   Collective blade pitch [rad]
        tilt_lon       : float   Normalised longitudinal swashplate tilt [-1,1]
        tilt_lat       : float   Normalised lateral swashplate tilt [-1,1]
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
        # 1. Disk normal vector (rotor Z-axis in world frame)
        #    R_hub columns: [x_body, y_body, z_body] expressed in world
        #    Disk normal = rotor body Z = R_hub @ [0,0,1]
        # ----------------------------------------------------------
        disk_normal = R_hub[:, 2]   # third column

        # ----------------------------------------------------------
        # 2. Relative wind at disk (world frame)
        # ----------------------------------------------------------
        v_rel_world = wind_world - v_hub_world

        # Axial component (through disk)
        v_axial = float(np.dot(v_rel_world, disk_normal))

        # In-plane (tangential) component magnitude — contributes to AoA
        v_inplane_vec  = v_rel_world - v_axial * disk_normal
        v_inplane      = float(np.linalg.norm(v_inplane_vec))

        # ----------------------------------------------------------
        # 3. Initial induced velocity estimate (iterative refinement)
        # ----------------------------------------------------------
        T_est    = 0.5 * self.rho * self.disk_area * v_axial**2   # rough guess
        v_i      = self._induced_velocity(T_est)

        # One-pass iteration for improved accuracy
        v_eff = v_axial + v_i
        for _ in range(3):
            T_est_new = 2.0 * self.rho * self.disk_area * v_i * abs(v_eff)
            v_i       = self._induced_velocity(T_est_new)
            v_eff     = v_axial + v_i

        # ----------------------------------------------------------
        # 4. BEM strip integration
        # ----------------------------------------------------------
        dT_total = 0.0   # incremental thrust [N]

        omega_abs = abs(omega_rotor)

        for r, dr in zip(self._r_mid, self._dr):
            # Tangential velocity at strip radius
            v_tan = omega_abs * r

            # Resultant local velocity
            v_loc = math.sqrt(v_tan**2 + v_eff**2)

            if v_loc < 0.5:
                # Skip strips with negligible flow (avoids divide-by-zero at startup)
                continue

            # Geometric inflow angle [rad]
            inflow_ang = math.atan2(v_eff, v_tan)

            # AoA = inflow angle + collective pitch, clamped for linear validity
            aoa = inflow_ang + collective_rad
            aoa_clamped = max(-self.aoa_limit, min(self.aoa_limit, aoa))

            # Lift and drag coefficients
            CL = self.CL0 + self.CL_alpha * aoa_clamped
            CD = self.CD0 + self.CD_alpha * (aoa_clamped**2)

            # Dynamic pressure
            q = 0.5 * self.rho * v_loc**2

            # Strip thrust contribution (lift component normal to disk)
            # dT per blade = q * chord * CL * dr, total for N blades
            # Note: CL acts perpendicular to local velocity, but for thrust
            # we project along disk normal (simplified BEM).
            dT = q * self.chord * CL * dr * self.n_blades

            # Strip drag (in-plane, opposes rotation) — captured via K_drag
            # Not integrated here; handled as Q = K_drag * T * r_tip below

            dT_total += dT

        # Net thrust (always ≥ 0 in autorotation from below)
        T = max(0.0, dT_total)

        # Apply in-plane wind enhancement: scale thrust by ratio of total to
        # purely axial flow so cross-wind contributes to autorotation lift.
        v_combined = math.sqrt(v_axial**2 + 0.3 * v_inplane**2)
        if abs(v_axial) > 0.01:
            T *= min(2.0, v_combined / max(abs(v_axial), 0.1))

        # Apply spin-up ramp
        T *= ramp

        # ----------------------------------------------------------
        # 5. Thrust force vector in world frame (along disk normal)
        # ----------------------------------------------------------
        F_world = T * disk_normal

        # ----------------------------------------------------------
        # 6. Rotor spin torques about disk normal
        #
        # Drag torque: aerodynamic resistance opposes rotation.
        #   Q_drag = -K_drag * T * r_tip
        #
        # Autorotative driving torque: in-plane wind accelerates the rotor.
        #   The in-plane wind component acts on the blade profile and drives
        #   autorotation. Modelled as proportional to rho * A * v_inplane².
        #   K_auto is tuned so Q_drive = Q_drag at nominal conditions, giving
        #   a stable equilibrium spin rate at the design tip-speed-ratio (λ=7).
        #   As omega rises, T rises (∝ omega²), so Q_drag grows until it
        #   balances Q_drive — providing the negative feedback for spin stability.
        #
        # Both terms are ramped with thrust so that spin-up is smooth.
        # ----------------------------------------------------------
        Q_drag  = -np.sign(omega_rotor) * self.K_drag * T * self.r_tip
        Q_drive =  np.sign(omega_rotor) * self.K_auto * self.rho * self.disk_area * v_inplane**2 * self.r_tip * ramp
        M_spin_world = (Q_drag + Q_drive) * disk_normal   # [N·m] in world frame

        # ----------------------------------------------------------
        # 7. Cyclic moments from swashplate tilt
        #    Tilting the rotor disk generates pitching/rolling moments
        #    proportional to thrust and tilt angles.
        #    In body frame: Mx_body = K_cyc * tilt_lat * T
        #                   My_body = K_cyc * tilt_lon * T
        #    (tilt in normalised units, scaled by PITCH_GAIN inside cyclic)
        # ----------------------------------------------------------
        from swashplate import _PITCH_GAIN_RAD as pitch_gain

        tilt_lon_rad = tilt_lon * pitch_gain
        tilt_lat_rad = tilt_lat * pitch_gain

        # Body-frame moments
        Mx_body = self.K_cyc * tilt_lat_rad * T
        My_body = self.K_cyc * tilt_lon_rad * T
        Mz_body = 0.0

        M_cyc_body  = np.array([Mx_body, My_body, Mz_body])
        # Rotate to world frame
        M_cyc_world = R_hub @ M_cyc_body

        # Total moment
        M_world = M_spin_world + M_cyc_world

        result = np.zeros(6)
        result[0:3] = F_world
        result[3:6] = M_world

        # Store internals for telemetry access without API change
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

        log.debug(
            "t=%.3f T=%.2fN v_axial=%.2f v_i=%.2f ramp=%.2f F=%s M=%s",
            t, T, v_axial, v_i, ramp, F_world.round(2), M_world.round(2),
        )

        return result

    def compute_anti_rotation_moment(
        self,
        esc_normalized: float,
        omega_rotor:    float,
        T:              float,
    ) -> float:
        """
        Compute the counter-torque contribution from the GB4008 motor (Mz).

        At trim, this should approximately cancel the rotor drag torque:
            Q_drag = K_drag * T * R_tip
        ESC output maps linearly to motor torque.

        Parameters
        ----------
        esc_normalized : float   ESC throttle in [-1, 1] (servo[3] normalized)
        omega_rotor    : float   Rotor spin rate [rad/s]
        T              : float   Current rotor thrust [N]

        Returns
        -------
        float   Counter-torque [N·m], positive = same sign as omega_rotor
        """
        # Motor authority: able to cancel ~full drag torque at nominal tip speed.
        # NOTE: in the current single-body MBDyn model this is NOT applied as an
        # external couple — motor and bearing drag are internal forces that cancel.
        # This method is retained for future two-body (hub + electronics) models.
        max_motor_torque = self.K_drag * 2.0 * 0.5 * 1.22 * self.disk_area * 70.0**2 * self.r_tip
        esc_cmd = max(0.0, esc_normalized)
        return esc_cmd * max_motor_torque * np.sign(omega_rotor)


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.WARNING)
    print("RotorAero smoke test")

    aero = RotorAero()

    R    = np.eye(3)
    v_h  = np.zeros(3)
    wind = np.array([10.0, 0.0, 0.0])

    # At t < ramp_time, thrust should be 0
    forces = aero.compute_forces(0.0, 0.0, 0.0, R, v_h, 28.0, wind, t=0.0)
    assert np.allclose(forces, 0.0, atol=1e-10), f"t=0 forces not zero: {forces}"
    print("  t=0 ramp: OK (forces=zero)")

    # After ramp, should have positive upward thrust
    forces_ss = aero.compute_forces(0.1, 0.0, 0.0, R, v_h, 28.0, wind, t=10.0)
    print(f"  Steady-state forces: F={forces_ss[0:3].round(2)}  M={forces_ss[3:6].round(2)}")
    assert forces_ss[2] > 0, f"Expected positive Fz, got {forces_ss[2]}"
    print("  Fz > 0 (upward thrust): OK")

    # Drag torque should oppose rotation
    assert forces_ss[5] < 0, f"Expected negative Mz (drag), got {forces_ss[5]}"
    print("  Mz < 0 (drag opposes CCW spin): OK")

    # Cyclic tilt should produce rolling moment
    forces_cyc = aero.compute_forces(0.1, 0.0, 0.5, R, v_h, 28.0, wind, t=10.0)
    print(f"  Cyclic tilt_lat=0.5: F={forces_cyc[0:3].round(2)}  M={forces_cyc[3:6].round(2)}")

    print("All smoke tests passed.")
    sys.exit(0)
