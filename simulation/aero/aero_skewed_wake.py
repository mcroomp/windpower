"""
aero_skewed_wake.py — BEM with Coleman skewed-wake non-uniform induction.

Overview
--------
Blade Element Momentum (BEM) theory computes rotor forces by dividing each
blade into radial strips, computing lift and drag on each strip from the
local apparent wind, then summing over all strips and blade positions.

"Apparent wind" at a strip has three contributions:
  1. Hub-relative freestream wind (wind_world - v_hub_world)
  2. Rotational velocity of the strip (ω × r, tangential)
  3. Induced velocity — the rotor slows the axial airflow through the disk;
     momentum theory gives the magnitude v_i from the thrust estimate

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

The cos(ψ − ψ_skew) term means the induction is strongest on the upwind side
of the disk (where the wake hasn't blown away yet) and weakest on the downwind
side.  This non-uniformity:
  • Increases lift on the advancing blade (less local induction → higher AoA)
  • Decreases lift on the retreating blade (more induction → lower AoA)
  • Produces a pitching moment on the disk (longitudinal flapping moment)
  • Is the primary cause of H-force asymmetry in forward flight

The Prandtl tip-loss correction is also applied here: near the blade tips and
roots the finite number of blades allows air to flow around the ends, reducing
effective lift.  F_tip · F_root (both < 1) scales down the dynamic pressure at
each radial strip to account for this.

This file is the reference (pure-Python) implementation.  The numerically
identical JIT-compiled version is in aero_skewed_wake_jit.py.  The two are
compared by test_skewed_wake_jit.py to guard against drift.

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

    Algorithm summary (one call to compute_forces)
    -----------------------------------------------
    1. Decompose wind into axial (through disk) and in-plane components.
    2. Bootstrap uniform induced velocity v_i0 via 3-iteration momentum theory
       at the representative radius R_CP (2/3 span).
    3. Compute Coleman skew angle χ and factor K = tan(χ/2) from the ratio
       of in-plane to axial effective velocity.
    4. Find ψ_skew — the azimuth of the upwind direction in the disk frame —
       so the cos(ψ − ψ_skew) modulation is aligned with the actual wind.
    5. Compute Prandtl tip/root loss factor F(r) once per radial strip
       (it does not vary with azimuth).
    6. Double loop over N_AB azimuth×blade positions and N_RADIAL radial strips:
         a. Compute blade unit vectors (span, chord, normal) at this azimuth.
         b. Apply Coleman modulation to get local induced velocity v_i_local.
         c. Build apparent wind vector ua = freestream - rotation - induction.
         d. Decompose ua into chord-wise and normal components → angle of attack.
         e. Compute CL, CD from linear lift curve + induced-drag polar.
         f. Scale dynamic pressure by Prandtl F; compute strip force and moment.
    7. Sum forces and moments; scale by ramp/N_AZ.
    8. Decompose total moment into spin-axis torque and cyclic (tilt) components.

    Key diagnostic attributes (set after each compute_forces call)
    --------------------------------------------------------------
    last_T              — axial thrust [N] (positive = along disk_normal)
    last_v_axial        — component of wind through disk [m/s]
    last_v_i            — uniform induced velocity v_i0 [m/s]
    last_v_inplane      — magnitude of in-plane wind component [m/s]
    last_skew_angle_deg — wake skew angle chi [deg] (0 = axial, 90 = edgewise)
    last_K_skew         — Coleman K = tan(chi/2) — induction asymmetry factor
    last_F_prandtl      — mean Prandtl tip/root loss factor across radial strips
    last_Q_spin         — empirical spin ODE torque [N·m]
    last_M_spin         — spin-axis moment vector [N·m]
    last_M_cyc          — cyclic (tilt-producing) moment vector [N·m]
    last_H_force        — in-plane force magnitude [N]
    """

    N_AZ     = 12    # azimuth sample count per revolution (skew effect is azimuth-sensitive)
    N_RADIAL = 10    # radial strip count per blade

    def __init__(self, rotor, ramp_time: float = 5.0, **overrides):
        """
        Parameters
        ----------
        rotor       : RotorDefinition
            Rotor geometry and aerodynamic parameters.
        ramp_time   : float
            Duration [s] over which forces ramp up from zero at startup.
            Prevents an impulse at t=0 when the simulation starts with the
            rotor already spinning but no prior flow history.
        **overrides : dict
            Key/value pairs that override individual rotor parameters.

        Precomputed arrays (set once, reused every call)
        ------------------------------------------------
        _r_stations : (N_RADIAL,)  radial centres of each strip [m]
        _r_norm     : (N_RADIAL,)  r / R_tip — normalised radius for Coleman modulation
        _S_elem     : float        planform area of one radial strip [m^2]
        _phi_offsets: (N_AB,)      combined azimuth + blade phase angles [rad]
                      The N_AZ azimuth samples and N_BLADES blade offsets are
                      flattened into a single list of N_AB = N_AZ * N_BLADES
                      phase angles.  Iterating over these visits every
                      (azimuth_sample, blade) pair exactly once.
        """
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
        self.S_blade   = span * self.CHORD                            # one blade planform area [m^2]
        self.AR        = float(p["aspect_ratio"])
        self.R_CP      = self.R_ROOT + (2.0 / 3.0) * span            # representative radius for v_i bootstrap [m]
        self.disk_area = math.pi * (self.R_TIP ** 2 - self.R_ROOT ** 2)  # annular disk area [m^2]

        # Radial strip centres: N_RADIAL equally-spaced strips, centred in each strip
        self._dr         = span / self.N_RADIAL
        self._r_stations = np.array(
            [self.R_ROOT + (i + 0.5) * self._dr for i in range(self.N_RADIAL)]
        )
        self._r_norm = self._r_stations / self.R_TIP   # r/R_tip, used in Coleman modulation
        self._S_elem = self.CHORD * self._dr            # planform area of one strip [m^2]

        # Combined azimuth × blade phase offsets.
        # For N_AZ=12 samples and N_BLADES=4, this produces 48 phase angles covering
        # all unique (azimuth sample, blade) pairs in a single flat array.
        az_offsets    = (2.0 * math.pi / self.N_AZ)     * np.arange(self.N_AZ)
        blade_offsets = (2.0 * math.pi / self.N_BLADES) * np.arange(self.N_BLADES)
        self._phi_offsets = (az_offsets[:, None] + blade_offsets[None, :]).ravel()
        self.N_AB = self.N_AZ * self.N_BLADES

        # Diagnostics — initialised to zero/safe defaults
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
        """
        Linear ramp from 0 to 1 over [0, ramp_time].

        Forces are multiplied by this factor so they grow smoothly from zero
        at simulation start, avoiding a step impulse when the rotor is
        initialised already spinning with no prior induced-velocity history.
        """
        if t >= self.ramp_time:
            return 1.0
        return max(0.0, t / self.ramp_time)

    def _induced_velocity(self, T_guess: float, v_axial: float = 0.0) -> float:
        """
        Solve momentum theory for induced velocity given a thrust estimate.

        From actuator disk theory, thrust T = rho * A * v_i * (v_axial + 2*v_i),
        which rearranges to a quadratic in v_i.  Returns the physical root
        (positive v_i, meaning induction opposes the freestream).

        Parameters
        ----------
        T_guess  : thrust estimate [N] from the current BEM iteration
        v_axial  : axial wind speed through the disk [m/s] (may be negative
                   for vortex-ring states, which this model does not handle)
        """
        T_abs = max(abs(T_guess), 0.01)   # avoid sqrt(negative) if T_guess is tiny
        v_ax  = abs(v_axial)
        disc  = v_ax ** 2 + 2.0 * T_abs / (self.RHO * self.disk_area)
        return max(0.0, (-v_ax + math.sqrt(disc)) / 2.0)

    def _prandtl_F(self, r: float, phi_rad: float) -> float:
        """
        Prandtl tip/root loss factor F(r) in [0.01, 1].

        Models the fact that a finite number of blades allows air to leak
        around the blade tips and roots.  The induction (and therefore lift)
        falls off near the ends.  F = 1 at midspan; F < 1 near tip and root.

        Parameters
        ----------
        r       : radial position [m]
        phi_rad : inflow angle at this strip [rad] = atan2(v_axial_eff, v_tan)
                  used to compute how 'steep' the helical wake is

        Formula (Prandtl, 1919):
            f_tip  = (N/2) * (R_tip - r) / (r * sin(phi))
            F_tip  = (2/pi) * arccos(exp(-f_tip))     [tip correction]
            f_root = (N/2) * (r - R_root) / (R_root * sin(phi))
            F_root = (2/pi) * arccos(exp(-f_root))    [root correction]
            F      = F_tip * F_root
        """
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

        The wake skew angle chi is the angle between the disk normal and the
        direction the wake trails behind the disk.  For a purely axial inflow,
        chi = 0 (wake straight behind).  For purely edgewise flow, chi = 90 deg.

        Parameters
        ----------
        v_inplane     : magnitude of wind component in the rotor disk plane [m/s]
        v_axial_eff   : effective axial velocity including induced component [m/s]
                        = v_axial + v_i0

        Returns
        -------
        (K_skew, chi_rad)
            K_skew   : Coleman induction asymmetry factor K = tan(chi/2)
            chi_rad  : wake skew angle [rad]
        """
        chi = math.atan2(v_inplane, max(abs(v_axial_eff), 0.01))
        K   = math.tan(min(chi / 2.0, math.radians(89.0)))   # clamp: prevents K → inf near chi = 180 deg
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
        Compute aerodynamic wrench on the rotor hub in world NED frame.

        Parameters
        ----------
        collective_rad : float
            Mean blade pitch angle [rad], applied equally at all azimuths.
            Positive = more lift (nose-up for a horizontal blade).
        tilt_lon : float
            Longitudinal cyclic swashplate input, normalised to [-1, 1].
            Converted to radians via pitch_gain_rad.  Positive tilts the disk
            forward (in the body X direction).
        tilt_lat : float
            Lateral cyclic swashplate input, normalised to [-1, 1].
            Converted to radians via pitch_gain_rad.  Positive tilts the disk
            to the right (in the body Y direction).
        R_hub : (3, 3) ndarray
            Hub rotation matrix, columns are body-frame axes in world (NED):
              R_hub[:, 0] = body X (longitudinal), R_hub[:, 1] = body Y (lateral),
              R_hub[:, 2] = disk normal (spin axis, pointing up through disk).
        v_hub_world : (3,) ndarray
            Hub translational velocity in NED [m/s].
        omega_rotor : float
            Rotor spin rate [rad/s].  Sign follows right-hand rule around
            disk_normal (positive = CCW when viewed from above the disk).
        wind_world : (3,) ndarray
            Wind velocity in NED [m/s].
        t : float
            Simulation time [s], used only for the startup ramp.
        spin_angle : float
            Current rotor phase angle [rad].  Shifts all blade azimuth
            positions uniformly; allows time-varying results when the caller
            tracks the rotor phase.  Default 0 is appropriate when the model
            is used for quasi-steady averaging.

        Returns
        -------
        AeroResult
            .F_world   : (3,) total aerodynamic force in NED [N]
            .M_orbital : (3,) cyclic (tilt-producing) moment in NED [N·m]
            .Q_spin    : spin-axis drive torque for the rotor ODE [N·m]
                         Uses empirical model: k_drive * v_inplane - k_drag * omega^2
            .M_spin    : (3,) spin-axis moment vector in NED [N·m]
        """
        ramp = self._ramp_factor(t)

        # disk_normal is the third column of R_hub — the spin axis direction in world frame
        disk_normal  = R_hub[:, 2]
        omega_abs    = abs(float(omega_rotor))

        # Convert normalised cyclic inputs to radians using the physical pitch gain
        tilt_lon_rad = tilt_lon * self.pitch_gain_rad
        tilt_lat_rad = tilt_lat * self.pitch_gain_rad

        # --- Step 1: Decompose relative wind into axial and in-plane components ---
        # v_axial is positive when wind flows through the disk in the direction of disk_normal
        # (i.e. the wind is driving autorotation).
        # v_inplane_vec is the remaining component that lies in the rotor disk plane.
        v_rel_world   = wind_world - v_hub_world
        v_axial       = float(np.dot(v_rel_world, disk_normal))
        v_inplane_vec = v_rel_world - v_axial * disk_normal
        v_inplane     = float(np.linalg.norm(v_inplane_vec))

        # --- Step 2: Bootstrap uniform induced velocity (3-iteration momentum theory) ---
        # We need v_i0 to compute the true inflow angle, but v_i0 itself depends on
        # thrust which depends on the inflow angle.  Three iterations of the fixed-point
        # loop converges to <1% error for typical RAWES operating conditions.
        # The computation uses the representative radius R_CP (2/3 span) as a proxy
        # for the whole disk — only for bootstrapping; the full strip loop below uses
        # the Coleman-modulated v_i per strip.
        v_i0 = 0.0
        for _ in range(3):
            v_tan = omega_abs * self.R_CP                         # tangential speed at R_CP
            v_loc = math.sqrt(v_tan ** 2 + (v_axial + v_i0) ** 2)
            if v_loc > 0.5:
                inflow = math.atan2(v_axial + v_i0, v_tan)       # inflow angle at R_CP
                aoa0   = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, inflow + collective_rad))
                CL_    = self.CL0 + self.CL_ALPHA * aoa0
                CD_    = self.CD0 + CL_ ** 2 / (math.pi * self.AR * self.OSWALD)
                q0     = 0.5 * self.RHO * v_loc ** 2
                T_est  = max(0.0, self.N_BLADES * q0 * self.S_blade
                             * (CL_ * math.cos(inflow) - CD_ * math.sin(inflow)))
                v_i0   = self._induced_velocity(T_est, v_axial)  # momentum theory → new v_i0

        # --- Step 3: Coleman skewed-wake parameters ---
        # chi is the angle between the disk normal and the wake trailing direction.
        # K = tan(chi/2) is the Coleman asymmetry factor.
        K_skew, chi = self._coleman_skew(v_inplane, v_axial + v_i0)
        self.last_K_skew         = K_skew
        self.last_skew_angle_deg = math.degrees(chi)

        # --- Step 4: Find ψ_skew — the azimuth of the upwind direction in the disk plane ---
        # The Coleman correction is v_i(r, ψ) = v_i0 · (1 + K · (r/R) · cos(ψ − ψ_skew)).
        # ψ_skew is the azimuth in the disk frame where induction is maximum (the
        # upwind side).  We project the in-plane wind direction onto an orthonormal
        # basis {_bx, _by} constructed in the disk plane.
        if v_inplane > 0.01:
            v_ip_unit = v_inplane_vec / v_inplane
            # Primary disk-plane x-axis: project East onto the disk plane.
            # Fall back to North if the disk is facing East (East is nearly normal to disk).
            _EAST = np.array([0.0, 1.0, 0.0])   # NED: East = Y axis
            _ep   = _EAST - np.dot(_EAST, disk_normal) * disk_normal
            if np.linalg.norm(_ep) > 1e-6:
                _bx = _ep / np.linalg.norm(_ep)
            else:
                _NORTH = np.array([1.0, 0.0, 0.0])  # NED: North = X axis
                _ep    = _NORTH - np.dot(_NORTH, disk_normal) * disk_normal
                _bx    = _ep / np.linalg.norm(_ep)
            _by      = np.cross(disk_normal, _bx)   # completes the right-hand disk frame
            psi_skew = math.atan2(float(np.dot(v_ip_unit, _by)),
                                  float(np.dot(v_ip_unit, _bx)))
        else:
            psi_skew = 0.0   # negligible in-plane wind → skew direction undefined, set to zero

        # --- Step 5: Prandtl tip/root loss per radial strip ---
        # F(r) depends only on r and the local inflow angle, not on azimuth.
        # Computed once per radial station, then looked up in the inner loop.
        # The inflow angle uses the uniform v_i0 (not the per-strip Coleman value)
        # because this is how Prandtl's finite-blade correction is conventionally applied.
        v_ax_eff = abs(v_axial + v_i0)
        F_prandtl = []
        for r in self._r_stations:
            inflow_angle = math.atan2(v_ax_eff, max(omega_abs * r, 0.5))
            F_prandtl.append(self._prandtl_F(r, inflow_angle))
        self.last_F_prandtl = float(np.mean(F_prandtl))

        # --- Step 6: Strip integration — sum forces over all azimuth positions and radii ---
        # The outer loop iterates over N_AB = N_AZ * N_BLADES combined (azimuth, blade) positions.
        # _phi_offsets[i] = az_sample_angle + blade_angle, so each iteration represents
        # one blade at one of the N_AZ evenly-spaced azimuth sample positions.
        # spin_angle shifts all blades uniformly, tracking the actual rotor phase.
        F_acc = np.zeros(3)
        M_acc = np.zeros(3)

        for i in range(self.N_AB):
            phi_az = spin_angle + self._phi_offsets[i]   # absolute azimuth of this blade [rad]
            ca = math.cos(phi_az)
            sa = math.sin(phi_az)

            # Blade pitch: collective sets the mean, cyclic modulates sinusoidally with azimuth.
            # tilt_lon (longitudinal) has a sin(ψ) signature; tilt_lat (lateral) has cos(ψ).
            # This is the standard 1P swashplate mixing law.
            p_k  = collective_rad + tilt_lon_rad * sa + tilt_lat_rad * ca
            cp_k = math.cos(p_k)
            sp_k = math.sin(p_k)

            # Blade unit vectors in the hub body frame.
            # The blade points radially outward in the disk plane at azimuth phi_az:
            #   e_span  = (cos φ, sin φ, 0)  — radially outward along blade
            #   e_chord = blade leading-edge direction, rotated by blade pitch p_k
            #             = (−sin φ, cos φ, 0) rotated by p_k about span axis
            #   e_normal = blade suction-surface normal, orthogonal to span and chord
            e_span_body   = np.array([ ca,        sa,       0.0 ])
            e_chord_body  = np.array([-sa * cp_k, ca * cp_k, sp_k])
            e_normal_body = np.array([ sa * sp_k, -ca * sp_k, cp_k])

            # Rotate blade vectors from hub body frame to world (NED) frame.
            # R_hub maps body columns to world, so:  v_world = R_hub @ v_body
            e_span_world   = R_hub @ e_span_body
            e_chord_world  = R_hub @ e_chord_body
            e_normal_world = R_hub @ e_normal_body

            # Tangential direction at this azimuth: the direction of rotational velocity.
            # For a point on the blade at radius r: v_rot = omega * r * e_tang
            # e_tang = disk_normal × e_span  (right-hand rule around the spin axis)
            e_tang = np.cross(disk_normal, e_span_world)

            # Coleman induction modulation at this azimuth.
            # cos(φ − ψ_skew) is +1 on the upwind side (maximum induction) and
            # −1 on the downwind side (minimum induction).
            cos_psi_skew = math.cos(phi_az - psi_skew)

            for j in range(self.N_RADIAL):
                r = self._r_stations[j]

                # Non-uniform induced velocity at this (azimuth, radius) position.
                # Coleman modulation increases/decreases the uniform v_i0 proportionally
                # to K * (r/R), stronger at larger radii and larger skew.
                v_i_local = v_i0 * (1.0 + K_skew * self._r_norm[j] * cos_psi_skew)

                # Strip centre position vector in world frame.
                r_cp = r * e_span_world

                # Apparent wind at this strip: three contributions combined.
                # v_rot is subtracted because the strip moves in the direction of e_tang
                # (the rotation carries the blade), so from the blade's perspective the
                # rotational contribution is a headwind in the −e_tang direction.
                # v_induced opposes the axial inflow (induction slows the through-disk wind).
                v_rot     = omega_rotor * r * e_tang
                v_induced = v_i_local * disk_normal
                ua        = v_rel_world - v_rot - v_induced

                ua_norm = float(np.linalg.norm(ua))
                if ua_norm < 0.5:
                    continue   # negligible apparent wind — no meaningful lift/drag

                # Decompose ua into chord-wise and normal-to-chord components.
                # ua_chord: component along the chord line (drives rotation of inflow angle).
                # ua_normal: component perpendicular to chord, along the blade normal.
                # The angle of attack is defined as alpha = atan(-ua_normal / ua_chord),
                # i.e. positive alpha when ua comes from below the chord line.
                ua_chord = float(np.dot(ua, e_chord_world))
                if abs(ua_chord) < 1e-6:
                    continue   # blade aligned with apparent wind — undefined AoA
                ua_normal = float(np.dot(ua, e_normal_world))

                # Angle of attack: small-angle-friendly decomposition.
                # The negative sign: if ua has a component in the +normal direction
                # (wind pushes on the suction side), the AoA is negative in this convention.
                # Clamped to AOA_LIMIT to avoid deep-stall extrapolation.
                alpha = max(-self.AOA_LIMIT, min(self.AOA_LIMIT, -ua_normal / ua_chord))

                # Linear lift curve and induced-drag polar (no profile-drag correction for camber)
                CL = self.CL0 + self.CL_ALPHA * alpha
                CD = self.CD0 + CL ** 2 / (math.pi * self.AR * self.OSWALD)   # induced drag

                # Dynamic pressure for this strip, scaled by the Prandtl loss factor.
                # _S_elem = chord * dr is the planform area of this strip.
                q = 0.5 * self.RHO * ua_norm ** 2 * self._S_elem * F_prandtl[j]

                # Lift direction: perpendicular to the apparent wind and the span axis,
                # in the plane defined by ua and e_span.  This is the aerodynamic lift
                # direction — it lies in the plane of ua and e_span, 90° from ua.
                e_lift_raw = np.cross(ua, e_span_world)
                lift_norm  = float(np.linalg.norm(e_lift_raw))
                if lift_norm < 1e-9:
                    continue   # ua parallel to span — degenerate geometry
                e_lift = e_lift_raw / lift_norm

                # Strip aerodynamic force: lift (perpendicular to ua) + drag (along ua)
                F_strip = q * (CL * e_lift + CD * (ua / ua_norm))

                F_acc += F_strip
                M_acc += np.cross(r_cp, F_strip)   # moment about hub origin

        # --- Step 7: Scale accumulated sums ---
        # Dividing by N_AZ converts the discrete azimuth sum to a per-revolution average:
        # instead of integrating dψ/(2π) continuously, we sampled at N_AZ points,
        # so each sample represents 1/N_AZ of the revolution.
        # Multiplying by ramp applies the startup fade-in.
        scale   = ramp / self.N_AZ
        F_total = F_acc * scale
        M_total = M_acc * scale

        # --- Step 8: Decompose total moment into spin-axis and cyclic components ---
        # The component of M_total along disk_normal is the net aerodynamic torque
        # about the spin axis — this drives the rotor speed ODE.
        # The remainder (M_cyc) is the moment that tilts the disk — this is what
        # the swashplate controller commands to steer the rotor.
        Q_spin_scalar = float(np.dot(M_total, disk_normal))
        M_spin_world  = Q_spin_scalar * disk_normal
        M_cyc_world   = M_total - M_spin_world

        # --- Diagnostics ---
        self.last_T         = float(np.dot(F_total, disk_normal))
        self.last_v_axial   = v_axial
        self.last_v_i       = v_i0
        self.last_v_inplane = v_inplane
        self.last_ramp      = ramp
        # Q_spin for the rotor ODE uses an empirical model rather than the BEM moment,
        # because BEM spin torque is sensitive to inflow-angle sign near stall.
        # k_drive scales with v_inplane (wind drives spin); k_drag opposes spin quadratically.
        self.last_Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs ** 2)
        self.last_M_spin    = M_spin_world.copy()
        self.last_M_cyc     = M_cyc_world.copy()
        self.last_H_force   = float(np.linalg.norm(F_total - self.last_T * disk_normal))

        log.debug("t=%.3f T=%.2fN chi=%.1f deg K=%.3f F_prandtl=%.3f",
                  t, self.last_T, self.last_skew_angle_deg, K_skew, self.last_F_prandtl)

        return AeroResult(
            F_world   = F_total.copy(),
            M_orbital = M_cyc_world.copy(),
            Q_spin    = float(self.k_drive_spin * v_inplane - self.k_drag_spin * omega_abs ** 2),
            M_spin    = M_spin_world.copy(),
        )
