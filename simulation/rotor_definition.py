"""
rotor_definition.py — RAWES rotor configuration loader and validated geometry API.

A RotorDefinition holds all blade geometry, mass, airfoil, control, and
autorotation parameters for one rotor variant.  Load from a YAML file:

    import rotor_definition as rd
    rotor = rd.load("beaupoil_2026")          # built-in name
    rotor = rd.load("/path/to/my_rotor.yaml") # explicit path
    rotor = rd.default()                       # Beaupoil 2026

The object pre-computes derived geometry and non-dimensional parameters
(r_cp, S_w, disk_area, σ, AR, …) as properties, and exposes factory
helpers for RotorAero and RigidBodyDynamics:

    aero     = RotorAero(**rotor.aero_kwargs())
    dynamics = RigidBodyDynamics(**rotor.dynamics_kwargs(), pos0=…, vel0=…)

Built-in validation
-------------------
Call ``rotor.validate()`` to check geometry consistency, physical plausibility,
and flag TBD fields.  Returns a list of ValidationIssue objects:

    issues = rotor.validate()
    for i in issues:
        print(f"[{i.level}] {i.field}: {i.message}")

The validation rules are documented in _VALIDATION_RULES below.

Built-in definitions
--------------------
  beaupoil_2026     — Actual hardware (4 blades, 2 m, 5 kg, SG6042, Kaman flaps)
  de_schutter_2018  — Thesis reference (3 blades, 1.5 m, 40 kg, De Schutter 2018)
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

try:
    import yaml as _yaml
    _YAML_AVAILABLE = True
except ImportError:  # pragma: no cover
    _YAML_AVAILABLE = False

_HERE     = Path(__file__).parent
_DEFS_DIR = _HERE / "rotor_definitions"

BUILTIN: dict = {
    "beaupoil_2026":    _DEFS_DIR / "beaupoil_2026.yaml",
    "de_schutter_2018": _DEFS_DIR / "de_schutter_2018.yaml",
    "pca2_1934":        _DEFS_DIR / "pca2_1934.yaml",
    "naca_tr_552_c":    _DEFS_DIR / "naca_tr_552_c.yaml",
}

# Cache of resolved path -> RotorDefinition.  RotorDefinition is treated as
# read-only after load; callers must not mutate the returned instance.
_LOAD_CACHE: dict = {}


# ---------------------------------------------------------------------------
# Validation result
# ---------------------------------------------------------------------------

@dataclass
class ValidationIssue:
    """A single validation finding from RotorDefinition.validate()."""
    level:   str   # "ERROR", "WARNING", "INFO"
    field:   str   # YAML path, e.g. "rotor.chord_m"
    message: str

    def __str__(self):
        return f"[{self.level}] {self.field}: {self.message}"


# ---------------------------------------------------------------------------
# Kaman flap sub-record
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class KamanFlap:
    """
    Kaman servo-flap parameters (US3217809, Kaman/Bossler 1965).

    A trailing-edge control surface on each blade driven by push-rods from
    the non-rotating swashplate.  Small flap deflection γ creates an
    aerodynamic pitching moment on the blade, providing pitch authority with
    significantly reduced swashplate mechanical loads.

    Fields marked Optional/None are TBD until blade design is finalised.
    """
    enabled:                  bool            = False
    chord_fraction:           Optional[float] = None   # e_f = c_flap/c_blade
    span_start_m:             Optional[float] = None   # inboard flap end [m from axis]
    span_end_m:               Optional[float] = None   # outboard flap end [m from axis]
    tau:                      Optional[float] = None   # flap effectiveness ∂β/∂γ
    CM_gamma_per_rad:         Optional[float] = None   # ∂CM/∂γ [/rad]
    swashplate_load_fraction: Optional[float] = None   # 0=full Kaman, 1=no flap
    notes:                    str             = ""

    # ── Derived (computed in __post_init__) ───────────────────────────────────
    # flap_span_m: Optional[float] = field(init=False, default=None)
    # flap_area_m2: Optional[float] = field(init=False, default=None)

    def is_geometry_defined(self) -> bool:
        """True when enough geometry is known to compute flap aerodynamics."""
        return (
            self.chord_fraction is not None and
            self.span_start_m   is not None and
            self.span_end_m     is not None
        )

    def is_fully_specified(self) -> bool:
        """True when all aerodynamic parameters are known (nothing TBD)."""
        return self.is_geometry_defined() and self.tau is not None

    def swashplate_load_reduction_pct(self) -> Optional[float]:
        """
        Estimated swashplate mechanical load reduction [%] from Kaman flap.

        100% = full Kaman (swashplate carries no aerodynamic moment, only
        mechanical friction).  0% = no flap benefit.

        Returns None if swashplate_load_fraction is TBD.
        """
        if self.swashplate_load_fraction is None:
            return None
        return (1.0 - self.swashplate_load_fraction) * 100.0


# ---------------------------------------------------------------------------
# RotorDefinition — main class
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RotorDefinition:
    """
    Complete aerodynamic and structural specification of one RAWES rotor variant.

    Follows standard rotor parameter conventions from:
      Leishman (2006) "Principles of Helicopter Aerodynamics"
      De Schutter et al. (2018) RAWES optimal control paper
      NeuralFoil XFOIL surrogate (Re=490k, beaupoil_2026)

    Geometry parameters (all SI)
    ----------------------------
    n_blades       N  — number of blades
    radius_m       R  — rotor tip radius [m]
    root_cutout_m  r_root — root cutout radius [m]  (blade starts here)
    chord_m        c  — blade chord [m]  (constant span: no taper)
    twist_deg      θ_tw — linear twist, root→tip [°]

    Derived (read-only properties)
    --------------------------------
    span_m         b  = R − r_root
    r_cp_m             = r_root + 2/3 · span  (De Schutter centre of pressure)
    S_w_m2         S  = N · c · span          (total blade area)
    disk_area_m2   A  = π(R² − r_root²)       (annular disk area)
    aspect_ratio   AR = span / c
    solidity       σ  = N·c / (π·R)
    """

    # ── Identity ──────────────────────────────────────────────────────────────
    name:        str = "unknown"
    description: str = ""

    # ── Rotor geometry ────────────────────────────────────────────────────────
    n_blades:       int   = 4
    radius_m:       float = 2.5      # R — tip radius
    root_cutout_m:  float = 0.5      # r_root
    chord_m:        float = 0.15     # c
    twist_deg:      float = 0.0      # θ_tw
    taper_ratio:    float = 1.0      # c_tip / c_root

    # ── Airfoil section ───────────────────────────────────────────────────────
    airfoil_name:        str           = "SG6042"
    airfoil_source:      str           = ""
    polar_csv:           Optional[str] = None   # filename relative to rotor_definitions/; loads CL/CD table
    CD_structural:       float         = 0.0   # C_{D,T} — parasitic drag from connecting beams/cables
                                                # De Schutter 2018 Eq. 29, 31.  Added to blade CD.
                                                # Normalised to blade planform area S_w.
    Re_design:           Optional[int] = None
    Re_operating:        Optional[int] = None

    # ── Linear lift model (used by Peters-He BEM) ────────────────────────────
    # CL = CL0 + CL_alpha_per_rad · alpha,  CD = CD0 + CL²/(π·AR·oswald_efficiency)
    CL0:               Optional[float] = None   # zero-AoA lift coefficient  [–]
    CL_alpha_per_rad:  Optional[float] = None   # lift slope [/rad]
    CD0:               Optional[float] = None   # zero-lift drag coefficient [–]
    oswald_efficiency: Optional[float] = None   # Oswald span efficiency factor [–]
    alpha_stall_deg:   Optional[float] = None   # AoA clamp for linear model [°]

    # ── Autorotation ODE torque coefficients ─────────────────────────────────
    K_drive_Nms_m:    Optional[float] = None   # drive torque per (m/s) in-plane wind [N·m·s/m]
    K_drag_Nms2_rad2: Optional[float] = None   # spin braking torque coefficient [N·m·s²/rad²]

    # ── Inertia ───────────────────────────────────────────────────────────────
    mass_kg:           float          = 5.0
    I_body_kgm2:       list           = field(default_factory=lambda: [5.0, 5.0, 10.0])
    I_spin_kgm2:       Optional[float] = None   # None = compute from blade_mass_kg; 0.0 = disabled
    blade_mass_kg:     Optional[float] = None   # per-blade mass [kg] — used to derive I_spin
    # Stationary assembly: axle, GB4008 motor, electronics, lower swashplate.
    # Excluded from I_spin because it does not rotate with the rotor.
    stationary_assembly_mass_kg: Optional[float] = None
    # Spinning outer shell mass (hub shell + upper swashplate ring + push-rods).
    # Does NOT include blade mass — blades are in blade_mass_kg.
    # When all three component masses are set (blade, stationary, shell), mass_kg
    # is derived: n_blades×blade + stationary + shell.  No need to set mass_kg
    # explicitly in the YAML when the component breakdown is available.
    spinning_hub_shell_mass_kg: Optional[float] = None
    I_blade_flap_kgm2: Optional[float] = None   # I_b — for Lock number

    # ── Tether attachment ─────────────────────────────────────────────────────
    axle_attachment_length_m: float = 0.3   # distance from hub CoM to tether attach along -body_Z [m]
                                             # Creates restoring torque M = r_attach × F_tether
                                             # that passively aligns body_z with tether (pendulum).
                                             # beaupoil_2026: 0.3 m (bottom of axle).

    # ── Control ───────────────────────────────────────────────────────────────
    K_cyc:                    float     = 0.4
    swashplate_phase_deg:     float     = 0.0   # cyclic phase advance for gyroscopic comp [°]
    swashplate_pitch_gain_rad: float = 0.3    # max blade pitch per unit normalised tilt [rad]
    servo_slew_rate_deg_s:    float     = 1200.0  # DS113MG at 6 V: 60 deg in 0.05 s
    servo_travel_deg:         float     = 60.0    # DS113MG total angular travel [deg]
    kaman_flap:               KamanFlap = field(default_factory=KamanFlap)

    # ── Autorotation ODE ─────────────────────────────────────────────────────
    I_ode_kgm2:         float          = 10.0
    omega_min_rad_s:    float          = 0.5
    omega_eq_rad_s:     Optional[float] = None

    # ── Environment ──────────────────────────────────────────────────────────
    rho_kg_m3: float = 1.22

    # =========================================================================
    # Derived geometry (read-only properties)
    # =========================================================================

    @property
    def span_m(self) -> float:
        """b = R − r_root [m]"""
        return self.radius_m - self.root_cutout_m

    @property
    def r_cp_m(self) -> float:
        """Centre of pressure per De Schutter: r_root + 2/3 · span [m]"""
        return self.root_cutout_m + (2.0 / 3.0) * self.span_m

    @property
    def r_eff_thrust_m(self) -> float:
        """
        q²-weighted effective radius for thrust (correct for radial integration).

        r_eff = √((R³ − r_root³) / (3·span))

        This is the radius at which evaluating q = ½ρ(Ωr)² gives the same
        thrust as integrating over all strips.  De Schutter uses r_cp which
        is ~14% higher (overestimates q by ~30%).
        """
        return math.sqrt((self.radius_m**3 - self.root_cutout_m**3) / (3.0 * self.span_m))

    @property
    def S_w_m2(self) -> float:
        """Total blade area S = N · c · span [m²]"""
        return self.n_blades * self.chord_m * self.span_m

    @property
    def disk_area_m2(self) -> float:
        """Annular disk area A = π(R² − r_root²) [m²]"""
        return math.pi * (self.radius_m**2 - self.root_cutout_m**2)

    @property
    def aspect_ratio(self) -> float:
        """AR = span / chord  [–]"""
        return self.span_m / self.chord_m

    @property
    def solidity(self) -> float:
        """σ = N · c / (π · R)  [–]  — fraction of disk area covered by blades"""
        return self.n_blades * self.chord_m / (math.pi * self.radius_m)

    @property
    def disk_loading_N_m2(self) -> float:
        """DL = W / A = m·g / A  [N/m²]  — disk loading at hover"""
        return self.mass_kg * 9.81 / self.disk_area_m2

    @property
    def I_spin_blade_kgm2(self) -> Optional[float]:
        """
        Spin-axis moment of inertia estimated from blade geometry [kg·m²].

        Only the SPINNING parts contribute: blades + outer hub shell.
        The stationary assembly (axle, motor, electronics, lower swashplate)
        does not rotate and is excluded via ``stationary_assembly_mass_kg``.

        Formula:
            I_blade       = m_blade × (R² + R·r_root + r_root²) / 3  (uniform rod)
            m_hub_spin    = mass_kg − n_blades × blade_mass_kg − stationary_assembly_mass_kg
            I_hub_spin    = m_hub_spin × r_root² / 2                  (solid disk)
            I_spin        = n_blades × I_blade + I_hub_spin
        """
        if self.blade_mass_kg is None:
            return None
        R, r = self.radius_m, self.root_cutout_m
        I_blade = self.blade_mass_kg * (R**2 + R * r + r**2) / 3.0
        stationary = self.stationary_assembly_mass_kg or 0.0
        m_hub = max(0.0, self.mass_kg - self.n_blades * self.blade_mass_kg - stationary)
        I_hub = m_hub * r**2 / 2.0
        return self.n_blades * I_blade + I_hub

    @property
    def I_spin_effective_kgm2(self) -> float:
        """
        Spin-axis inertia to use in dynamics [kg·m²].

        Priority:
          1. ``I_spin_kgm2`` if explicitly set (not None).
          2. ``I_spin_blade_kgm2`` if ``blade_mass_kg`` is set.
          3. 0.0 (gyroscopic coupling disabled).
        """
        if self.I_spin_kgm2 is not None:
            return float(self.I_spin_kgm2)
        computed = self.I_spin_blade_kgm2
        return float(computed) if computed is not None else 0.0

    @property
    def lock_number(self) -> Optional[float]:
        """
        γ = ρ · CL_alpha · c · R⁴ / I_b  [–]

        Governs aerodynamic/inertia coupling in flapping dynamics.
        Typical helicopters: γ = 5–12.  Returns None if I_b is unknown.
        """
        if self.I_blade_flap_kgm2 is None or self.I_blade_flap_kgm2 <= 0:
            return None
        return (self.rho_kg_m3 * self.CL_alpha_3D_per_rad * self.chord_m *
                self.radius_m**4 / self.I_blade_flap_kgm2)

    @property
    def CL_alpha_3D_per_rad(self) -> float:
        """
        Finite-wing 3D CL_alpha from Prandtl lifting-line theory [/rad].

        CL_alpha_3D = 2π / (1 + 2/AR)

        Upper bound — does not account for Re losses, unsteady effects, or
        tip losses.  Actual measured value should be below this.
        """
        return 2.0 * math.pi / (1.0 + 2.0 / self.aspect_ratio)

    @property
    def max_cyclic_moment_Nm(self) -> float:
        """
        Analytical upper bound on cyclic moment at full swashplate deflection [N·m].

        Derived by integrating blade lift × moment arm over the disk at design
        spin rate, then averaging over azimuth:

            M_cyc_max = (N/4) · ρ · ω_eq² · c · CL_α · β_max · (R⁴ − r_root⁴) / 4

        where β_max = swashplate_pitch_gain_rad (= blade pitch at normalised tilt = 1.0).

        Overestimates the BEM result by ~25% because it omits Prandtl tip loss
        and induction effects.  Use as a conservative upper bound.
        """
        if self.omega_eq_rad_s is None:
            raise ValueError("omega_eq_rad_s must be set to compute max_cyclic_moment_Nm")
        omega = self.omega_eq_rad_s
        return (
            (self.n_blades / 4.0)
            * self.rho_kg_m3
            * omega ** 2
            * self.chord_m
            * self.CL_alpha_3D_per_rad
            * self.swashplate_pitch_gain_rad
            * (self.radius_m ** 4 - self.root_cutout_m ** 4)
            / 4.0
        )

    @property
    def max_body_z_rate_rad_s(self) -> float:
        """
        Physical upper bound on body_z tilt rate [rad/s].

        A spinning rotor precesses when a cyclic moment is applied.  The
        maximum precession rate is:

            ω_tilt_max = M_cyc_max / H
            H           = I_spin · ω_eq   (angular momentum)

        This is the fastest the disk CAN tilt when the swashplate is at full
        deflection.  Because the angular momentum H is small for this rotor
        (~80 kg·m²/s), the physical limit is high (~15–20 rad/s) and is
        never the binding constraint.

        The practical planner slew rate is set by the CONTROL BANDWIDTH, not
        by this limit.  As a rule of thumb:

            body_z_slew_rate ≈ 0.02–0.05 × max_body_z_rate_rad_s

        The lower end (×0.02) applies when commanding via 10 Hz MAVLink RC
        overrides (stack test / hardware).  The upper end (×0.05) applies for
        a 400 Hz on-board controller (Mode_RAWES firmware).

        Returns inf when I_spin = 0 (gyroscopic coupling disabled).
        """
        I = self.I_spin_effective_kgm2
        if I <= 0.0:
            return float("inf")
        if self.omega_eq_rad_s is None:
            raise ValueError("omega_eq_rad_s must be set to compute max_body_z_rate_rad_s")
        return self.max_cyclic_moment_Nm / (I * self.omega_eq_rad_s)

    @property
    def body_z_slew_rate_rad_s(self) -> float:
        """
        Recommended body_z planner slew rate for 10 Hz MAVLink control [rad/s].

        Set to 2% of the physical maximum (max_body_z_rate_rad_s).  This
        fraction is chosen so the slew is fast enough to complete a 55° reel-in
        transition in ~3–4 s, while staying within the closed-loop bandwidth
        of a 10 Hz RC-override attitude controller.

        For a 400 Hz on-board controller (Mode_RAWES firmware), use
        ``5% × max_body_z_rate_rad_s`` instead.
        """
        return 0.02 * self.max_body_z_rate_rad_s

    @property
    def sim_servo_speed(self) -> float:
        """
        SIM_SERVO_SPEED value for ArduPilot SITL.

        Units: full servo output range (0..1) per second — the same units
        ArduPilot uses for SIM_SERVO_SPEED.

        Derived from the physical servo spec:
            SIM_SERVO_SPEED = servo_slew_rate_deg_s / servo_travel_deg

        For DS113MG at 6 V: 545 / 100 = 5.45
        """
        return self.servo_slew_rate_deg_s / self.servo_travel_deg

    # =========================================================================
    # Polar table loader
    # =========================================================================

    def polar_arrays(self) -> Optional[Tuple]:
        """
        Load and return (alpha_rad, CL, CD) numpy arrays from polar_csv.

        Returns None if polar_csv is not set.  The arrays are sorted by alpha
        and cover the full range present in the CSV file.
        """
        if self.polar_csv is None:
            return None
        try:
            import numpy as _np
        except ImportError:
            return None
        csv_path = _DEFS_DIR / self.polar_csv
        alphas, CLs, CDs = [], [], []
        with csv_path.open() as fh:
            for line in fh:
                if line.strip().startswith("Alpha"):
                    break
            for line in fh:
                parts = line.strip().split(",")
                if len(parts) < 3:
                    continue
                try:
                    alphas.append(float(parts[0]))
                    CLs.append(float(parts[1]))
                    CDs.append(float(parts[2]))
                except ValueError:
                    continue
        if not alphas:
            return None
        alpha_rad = _np.radians(_np.array(alphas, dtype=_np.float64))
        return alpha_rad, _np.array(CLs, dtype=_np.float64), _np.array(CDs, dtype=_np.float64)

    # =========================================================================
    # Factory helpers
    # =========================================================================

    def aero_kwargs(self) -> dict:
        """
        Return kwargs for RotorAero(**rotor.aero_kwargs()).

        Maps RotorDefinition fields to the RotorAero constructor parameter names.
        When polar_csv is set, also includes polar_alpha_rad / polar_CL / polar_CD
        arrays for Peters-He table lookup.
        """
        d = dict(
            n_blades         = self.n_blades,
            r_root           = self.root_cutout_m,
            r_tip            = self.radius_m,
            chord            = self.chord_m,
            rho              = self.rho_kg_m3,
            aspect_ratio     = self.aspect_ratio,
            CD_structural    = self.CD_structural,
            K_cyc            = self.K_cyc,
            pitch_gain_rad   = self.swashplate_pitch_gain_rad,
        )
        polar = self.polar_arrays()
        if polar is not None:
            d["polar_alpha_rad"] = polar[0]
            d["polar_CL"]        = polar[1]
            d["polar_CD"]        = polar[2]
        # Linear lift model fields (required by Peters-He BEM)
        if self.CL0               is not None: d["CL0"]          = self.CL0
        if self.CL_alpha_per_rad  is not None: d["CL_alpha"]     = self.CL_alpha_per_rad
        if self.CD0               is not None: d["CD0"]          = self.CD0
        if self.oswald_efficiency is not None: d["oswald_eff"]   = self.oswald_efficiency
        if self.alpha_stall_deg   is not None: d["aoa_limit"]    = math.radians(self.alpha_stall_deg)
        if self.K_drive_Nms_m     is not None: d["k_drive_spin"] = self.K_drive_Nms_m
        if self.K_drag_Nms2_rad2  is not None: d["k_drag_spin"]  = self.K_drag_Nms2_rad2
        return d

    def dynamics_kwargs(self) -> dict:
        """
        Return partial kwargs for RigidBodyDynamics.

        Does NOT include pos0/vel0/R0/omega0 — those come from the config/mission.
        Usage:
            dyn = RigidBodyDynamics(**rotor.dynamics_kwargs(), pos0=…, vel0=…)
        """
        return dict(
            mass   = self.mass_kg,
            I_body = list(self.I_body_kgm2),
            I_spin = self.I_spin_effective_kgm2,
        )

    # =========================================================================
    # Reporting
    # =========================================================================

    def summary(self) -> str:
        """One-line human-readable description for logging."""
        return (
            f"{self.name}: {self.n_blades}×{self.span_m:.1f}m blades  "
            f"R={self.radius_m:.2f}m  σ={self.solidity:.4f}  "
            f"AR={self.aspect_ratio:.1f}  S_w={self.S_w_m2:.3f}m²  "
            f"mass={self.mass_kg:.1f}kg  CL_α_3D={self.CL_alpha_3D_per_rad:.3f}/rad"
        )

    def report(self) -> str:
        """
        Multi-line human-readable parameter report.

        Prints all geometry, non-dimensional parameters, and validation status.
        """
        kf = self.kaman_flap
        kf_str = (
            f"enabled (geometry {'defined' if kf.is_geometry_defined() else 'TBD'})"
            if kf.enabled else "disabled"
        )
        ln = self.lock_number
        issues = self.validate()
        issue_lines = "\n".join(f"  {i}" for i in issues) if issues else "  (none)"
        return (
            f"RotorDefinition: {self.name}\n"
            f"  {self.description.strip()}\n"
            f"\n"
            f"  Geometry:\n"
            f"    N={self.n_blades} blades  R={self.radius_m}m  "
            f"r_root={self.root_cutout_m}m  span={self.span_m:.3f}m\n"
            f"    chord={self.chord_m}m  twist={self.twist_deg}°  "
            f"taper={self.taper_ratio:.2f}\n"
            f"\n"
            f"  Derived geometry:\n"
            f"    r_cp={self.r_cp_m:.3f}m  r_eff_T={self.r_eff_thrust_m:.3f}m  "
            f"(De Schutter q-overestimate: {(self.r_cp_m/self.r_eff_thrust_m)**2:.3f}×)\n"
            f"    S_w={self.S_w_m2:.4f}m²  A={self.disk_area_m2:.4f}m²  "
            f"DL={self.disk_loading_N_m2:.2f}N/m²\n"
            f"    AR={self.aspect_ratio:.3f}  σ={self.solidity:.4f}\n"
            f"\n"
            f"  Airfoil: {self.airfoil_name} ({self.airfoil_source})\n"
            f"    CL_alpha_3D={self.CL_alpha_3D_per_rad:.3f}/rad  (Prandtl lifting-line, AR={self.aspect_ratio:.1f})\n"
            f"    polar_csv={self.polar_csv or '(none)'}\n"
            f"\n"
            f"  Inertia:\n"
            f"    mass={self.mass_kg}kg  I_body={self.I_body_kgm2}kg·m²\n"
            f"    I_spin={self.I_spin_kgm2}kg·m²  I_b="
            f"{'TBD' if self.I_blade_flap_kgm2 is None else f'{self.I_blade_flap_kgm2}kg·m²'}"
            f"  Lock γ={'TBD' if ln is None else f'{ln:.2f}'}\n"
            f"\n"
            f"  Control:\n"
            f"    K_cyc={self.K_cyc}  "
            f"Kaman flap: {kf_str}\n"
            f"\n"
            f"  Autorotation:\n"
            f"    I_ode={self.I_ode_kgm2}kg*m2  omega_min={self.omega_min_rad_s}rad/s\n"
            f"    omega_eq_nominal={'TBD' if self.omega_eq_rad_s is None else f'{self.omega_eq_rad_s}rad/s'}\n"
            f"\n"
            f"  Validation ({len(issues)} issue{'s' if len(issues)!=1 else ''}):\n"
            f"{issue_lines}\n"
        )

    # =========================================================================
    # Built-in validation
    # =========================================================================

    def validate(self) -> List[ValidationIssue]:
        """
        Physics and consistency validation.  Returns list of ValidationIssue.
        Empty list means all checks passed.

        Rules checked
        -------------
        Geometry:
          G1  span > 0 (R > r_root)
          G2  N ≥ 2 (need ≥ 2 blades for RAWES operation)
          G3  solidity σ in [0.02, 0.20] (typical range for rotors)
          G4  AR in [5, 25] (typical range; outside → unusual aspect ratio)
          G5  r_cp ∈ (r_root, R) (sanity check on formula)
          G6  r_cp > r_eff_T (De Schutter overestimate direction)

        Airfoil:
          A1  If Re_operating and Re_design given: warn if operating Re > 2× design Re

        Inertia:
          I1  mass > 0
          I2  I_body positive and Ixx+Iyy ≥ Izz (physical moments of inertia)
          I3  disk_loading < 50 N/m²  (above 50 → probably wrong mass or geometry)

        Kaman flap (when enabled):
          K1  If geometry not fully defined: INFO about TBD fields
          K2  If tau defined: tau in [0.1, 1.0]
          K3  If swashplate_load_fraction defined: in [0, 1]
          K4  If span_start_m < root_cutout_m: ERROR (flap in hub)
          K5  If span_end_m > radius_m: ERROR (flap beyond tip)

        Autorotation:
          S1  omega_min > 0

        Completeness (INFO only):
          C1  I_blade_flap_kgm2 not set → Lock number unknown
          C2  Re_design not set → cannot check regime mismatch
        """
        issues: List[ValidationIssue] = []

        def err(field, msg):
            issues.append(ValidationIssue("ERROR", field, msg))

        def warn(field, msg):
            issues.append(ValidationIssue("WARNING", field, msg))

        def info(field, msg):
            issues.append(ValidationIssue("INFO", field, msg))

        # ── G: Geometry ──────────────────────────────────────────────────────
        if self.span_m <= 0:
            err("rotor.span", f"span={self.span_m:.3f}m ≤ 0 (R={self.radius_m} ≤ r_root={self.root_cutout_m})")
        if self.n_blades < 2:
            err("rotor.n_blades", f"n_blades={self.n_blades} — need ≥ 2 for RAWES")
        if not (0.02 <= self.solidity <= 0.20):
            warn("rotor.solidity",
                 f"σ={self.solidity:.4f} outside typical range [0.02, 0.20]. "
                 f"N={self.n_blades} c={self.chord_m}m R={self.radius_m}m")
        if not (5.0 <= self.aspect_ratio <= 25.0):
            warn("rotor.aspect_ratio",
                 f"AR={self.aspect_ratio:.2f} outside typical range [5, 25]")
        if not (self.root_cutout_m < self.r_cp_m < self.radius_m):
            err("rotor.r_cp",
                f"r_cp={self.r_cp_m:.3f}m not in (r_root={self.root_cutout_m}, R={self.radius_m})")
        if self.span_m > 0 and self.r_cp_m <= self.r_eff_thrust_m:
            warn("rotor.r_cp",
                 f"r_cp={self.r_cp_m:.3f}m ≤ r_eff_T={self.r_eff_thrust_m:.3f}m — "
                 f"De Schutter r_cp should be above q²-weighted r_eff for the overestimate to cancel "
                 f"the low empirical CL_alpha")

        # ── A: Airfoil ────────────────────────────────────────────────────────
        if (self.Re_design is not None and self.Re_operating is not None
                and self.Re_operating > 2.0 * self.Re_design):
            warn("airfoil.Re_operating",
                 f"Operating Re={self.Re_operating} is "
                 f"{self.Re_operating/self.Re_design:.1f}× calibration Re={self.Re_design}.")

        # ── I: Inertia ────────────────────────────────────────────────────────
        if self.mass_kg <= 0:
            err("inertia.mass_kg", f"mass={self.mass_kg} ≤ 0")
        Ix, Iy, Iz = self.I_body_kgm2[0], self.I_body_kgm2[1], self.I_body_kgm2[2]
        for label, val in [("Ixx", Ix), ("Iyy", Iy), ("Izz", Iz)]:
            if val <= 0:
                err("inertia.I_body_kgm2", f"{label}={val} ≤ 0")
        if Ix + Iy < Iz * 0.8:
            warn("inertia.I_body_kgm2",
                 f"Ixx+Iyy={Ix+Iy:.1f} < 0.8·Izz={Iz:.1f} — "
                 f"violates triangle inequality for solid bodies")
        dl = self.disk_loading_N_m2
        if dl > 50.0:
            warn("inertia.mass_kg",
                 f"Disk loading DL={dl:.1f}N/m² > 50 N/m² — "
                 f"very high for an autorotating RAWES. Check mass or geometry.")
        if dl < 0.5:
            warn("inertia.mass_kg",
                 f"Disk loading DL={dl:.2f}N/m² < 0.5 N/m² — unusually low. "
                 f"Check mass={self.mass_kg}kg and disk_area={self.disk_area_m2:.1f}m².")

        # ── K: Kaman flap ─────────────────────────────────────────────────────
        kf = self.kaman_flap
        if kf.enabled:
            if not kf.is_geometry_defined():
                info("control.kaman_flap",
                     "Flap geometry (chord_fraction, span_start_m, span_end_m) is TBD. "
                     "Kaman flap is documented but not yet influencing aerodynamic model.")
            if kf.span_start_m is not None and kf.span_start_m < self.root_cutout_m:
                err("control.kaman_flap.span_start_m",
                    f"Flap start {kf.span_start_m}m < root_cutout {self.root_cutout_m}m — "
                    f"flap would be inside hub")
            if kf.span_end_m is not None and kf.span_end_m > self.radius_m:
                err("control.kaman_flap.span_end_m",
                    f"Flap end {kf.span_end_m}m > R={self.radius_m}m — "
                    f"flap would extend beyond blade tip")
            if kf.tau is not None and not (0.1 <= kf.tau <= 1.0):
                warn("control.kaman_flap.tau",
                     f"Flap effectiveness τ={kf.tau:.3f} outside typical range [0.1, 1.0]")
            if kf.swashplate_load_fraction is not None:
                if not (0.0 <= kf.swashplate_load_fraction <= 1.0):
                    err("control.kaman_flap.swashplate_load_fraction",
                        f"swashplate_load_fraction={kf.swashplate_load_fraction} must be in [0, 1]")
            else:
                info("control.kaman_flap",
                     "swashplate_load_fraction TBD — "
                     "designed to provide significant swashplate load reduction")

        # ── S: Autorotation ───────────────────────────────────────────────────
        if self.omega_min_rad_s <= 0:
            err("autorotation.omega_min_rad_s", f"omega_min={self.omega_min_rad_s} ≤ 0")

        # ── C: Completeness ───────────────────────────────────────────────────
        if self.I_blade_flap_kgm2 is None:
            info("inertia.I_blade_flap_kgm2",
                 "Blade flap inertia I_b not set — Lock number cannot be computed. "
                 "Measure or estimate from CAD for complete specification.")
        if self.Re_design is None:
            info("airfoil.Re_design",
                 "Design Reynolds number not set — cannot check regime mismatch.")

        return issues


# ---------------------------------------------------------------------------
# YAML loader
# ---------------------------------------------------------------------------

def _require_yaml():
    if not _YAML_AVAILABLE:
        raise ImportError(
            "PyYAML is required to load rotor definition YAML files.\n"
            "Install with:  pip install pyyaml\n"
            "Or in the RAWES venv:  "
            "simulation\\tests\\unit\\.venv\\Scripts\\python.exe -m pip install pyyaml"
        )


def load(path_or_name: str) -> RotorDefinition:
    """
    Load a RotorDefinition from a YAML file or built-in name.

    Parameters
    ----------
    path_or_name : str
        - Built-in name: "beaupoil_2026", "de_schutter_2018"
        - Absolute or relative path to a .yaml file

    Returns
    -------
    RotorDefinition

    Raises
    ------
    FileNotFoundError  if name not found and path does not exist
    ImportError        if PyYAML is not installed
    """
    _require_yaml()

    p = Path(path_or_name)
    if not p.is_absolute() and not p.exists():
        if path_or_name in BUILTIN:
            p = BUILTIN[path_or_name]
        else:
            raise FileNotFoundError(
                f"No built-in rotor named {path_or_name!r} and file not found. "
                f"Built-in options: {list(BUILTIN)}"
            )

    cache_key = str(p.resolve())
    if cache_key in _LOAD_CACHE:
        return _LOAD_CACHE[cache_key]

    with p.open(encoding="utf-8") as f:
        data = _yaml.safe_load(f)

    ro = data.get("rotor", {})
    af = data.get("airfoil", {})
    _m = data.get("inertia", {})
    ct = data.get("control", {})
    ar = data.get("autorotation", {})
    ev = data.get("environment", {})
    kf_data = ct.get("kaman_flap", {})

    kaman = KamanFlap(
        enabled                  = bool(kf_data.get("enabled", False)),
        chord_fraction           = kf_data.get("chord_fraction"),
        span_start_m             = kf_data.get("span_start_m"),
        span_end_m               = kf_data.get("span_end_m"),
        tau                      = kf_data.get("tau"),
        CM_gamma_per_rad         = kf_data.get("CM_gamma_per_rad"),
        swashplate_load_fraction = kf_data.get("swashplate_load_fraction"),
        notes                    = kf_data.get("notes", ""),
    )

    _rotor = RotorDefinition(
        name        = str(data.get("name", p.stem)),
        description = str(data.get("description", "")),

        n_blades       = _require_int(ro, "n_blades", p),
        radius_m       = float(ro.get("radius_m",    2.5)),
        root_cutout_m  = float(ro.get("root_cutout_m", 0.5)),
        chord_m        = float(ro.get("chord_m",     0.15)),
        twist_deg      = float(ro.get("twist_deg",   0.0)),
        taper_ratio    = float(ro.get("taper_ratio", 1.0)),

        airfoil_name      = str(af.get("designation",      "unknown")),
        airfoil_source    = str(af.get("source",           "")),
        polar_csv         = af.get("polar_csv"),
        CD_structural     = float(af.get("CD_structural",  0.0)),
        Re_design         = _m_int(af.get("Re_design")),
        Re_operating      = _m_int(af.get("Re_operating")),

        CL0               = _m_float(af.get("CL0")),
        CL_alpha_per_rad  = _m_float(af.get("CL_alpha_per_rad")),
        CD0               = _m_float(af.get("CD0")),
        oswald_efficiency = _m_float(af.get("oswald_efficiency")),
        alpha_stall_deg   = _m_float(af.get("alpha_stall_deg")),
        K_drive_Nms_m     = _m_float(ar.get("K_drive_Nms_m")),
        K_drag_Nms2_rad2  = _m_float(ar.get("K_drag_Nms2_rad2")),

        mass_kg           = _load_mass_kg(_m, n_blades=int(ro.get("n_blades", 4))),
        I_body_kgm2       = [float(v) for v in _m.get("I_body_kgm2", [5.0, 5.0, 10.0])],
        I_spin_kgm2                  = _m_float(_m.get("I_spin_kgm2")),
        blade_mass_kg                = _m_float(_m.get("blade_mass_kg")),
        stationary_assembly_mass_kg  = _load_stationary_mass(_m.get("stationary_assembly_mass_kg")),
        spinning_hub_shell_mass_kg   = _m_float(_m.get("spinning_hub_shell_mass_kg")),
        I_blade_flap_kgm2            = _m_float(_m.get("I_blade_flap_kgm2")),

        axle_attachment_length_m = float(ct.get("axle_attachment_length_m", 0.3)),
        K_cyc                    = float(ct.get("K_cyc",              0.4)),
        swashplate_phase_deg     = float(ct.get("swashplate_phase_deg", 0.0)),
        swashplate_pitch_gain_rad = _require_float(ct, "swashplate_pitch_gain_rad",
                                                   "control", p),
        servo_slew_rate_deg_s = float(ct.get("servo_slew_rate_deg_s", 1200.0)),
        servo_travel_deg      = float(ct.get("servo_travel_deg",       60.0)),
        kaman_flap           = kaman,

        I_ode_kgm2        = float(ar.get("I_ode_kgm2",       10.0)),
        omega_min_rad_s   = float(ar.get("omega_min_rad_s",  0.5)),
        omega_eq_rad_s    = _m_float(ar.get("omega_eq_rad_s")),

        rho_kg_m3 = float(ev.get("rho_kg_m3", 1.22)),
    )
    _LOAD_CACHE[cache_key] = _rotor
    return _rotor



def default() -> RotorDefinition:
    """Return the Beaupoil 2026 rotor definition (actual hardware)."""
    return load("beaupoil_2026")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _require_int(section: dict, key: str, path) -> int:
    """Return int(section[key]), raising ValueError if the key is absent."""
    if key not in section:
        raise ValueError(
            f"Required field '{key}' is missing from rotor YAML: {path}"
        )
    return int(section[key])


def _require_float(section: dict, key: str, section_name: str, path) -> float:
    """Return float(section[key]), raising ValueError if the key is absent."""
    if key not in section:
        raise ValueError(
            f"Required field '{section_name}.{key}' is missing from rotor YAML: {path}"
        )
    return float(section[key])


def _m_float(v) -> Optional[float]:
    """Parse optional float: None/null → None, else float(v)."""
    return None if v is None else float(v)


def _load_mass_kg(inertia_section: dict, n_blades: int = 4) -> float:
    """
    Determine total mass from the inertia YAML section.

    If blade_mass_kg, stationary_assembly_mass_kg (dict or float), and
    spinning_hub_shell_mass_kg are all present, mass_kg is computed as:
        n_blades × blade_mass_kg + stationary_assembly_mass_kg + spinning_hub_shell_mass_kg
    so it does not need to be stated explicitly in the YAML.
    Falls back to the explicit mass_kg field (default 5.0).
    """
    blade      = _m_float(inertia_section.get("blade_mass_kg"))
    stationary = _load_stationary_mass(inertia_section.get("stationary_assembly_mass_kg"))
    shell      = _m_float(inertia_section.get("spinning_hub_shell_mass_kg"))

    if blade is not None and stationary is not None and shell is not None:
        return blade * n_blades + stationary + shell

    explicit = inertia_section.get("mass_kg")
    if explicit is not None:
        return float(explicit)
    return 5.0


def _load_stationary_mass(v) -> Optional[float]:
    """
    Parse stationary_assembly_mass_kg: accepts either a plain float/int or a
    dict of {component_name: mass_kg} items whose values are summed.
    """
    if v is None:
        return None
    if isinstance(v, dict):
        return float(sum(float(x) for x in v.values()))
    return float(v)


def _m_int(v) -> Optional[int]:
    """Parse optional int: None/null → None, else int(v)."""
    return None if v is None else int(v)
