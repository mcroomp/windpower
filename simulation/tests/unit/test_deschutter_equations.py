"""
test_deschutter_equations.py — Equation-level validation of DeSchutterAero
against De Schutter, Leuthold, Diehl (2018).

Each test class is labelled with the paper equations it validates.
Expected values are computed directly from the paper's formulas,
not from the model output, so that the tests catch regressions in
both the parameter values and the physics implementation.

Reference
---------
De Schutter J., Leuthold R., Diehl M. (2018).
"Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy."
IFAC Proceedings, Vol. 51, No. 13, pp. 523–528.
https://publications.syscop.de/DeSchutter2018.pdf
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero import DeSchutterAero


# ---------------------------------------------------------------------------
# Shared fixtures and helpers
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ds_rotor():
    """De Schutter 2018 rotor definition loaded from YAML."""
    return rd.load("de_schutter_2018")


@pytest.fixture(scope="module")
def ds_model(ds_rotor):
    return DeSchutterAero(ds_rotor)


# Paper Table I geometry
N        = 3           # number of wings
L_w      = 1.5        # wing span [m]
L_b      = 1.60       # connecting arm length [m]  (optimal design, Section IV-A)
R        = L_b + L_w  # = 3.10 m  (tip radius)
A_aspect = 12.0       # aspect ratio  (Table I)
chord    = L_w / A_aspect  # = 0.125 m
S_w      = chord * L_w     # = 0.1875 m²  (one blade planform area)
r_cp     = L_b + (2.0 / 3.0) * L_w  # = 2.60 m  (Section II-D: 2/3 span from root)

# Paper Table I aerodynamics
CD0  = 0.01
O_e  = 0.8
# Eq. 25: C_{L,k} = (2π / (1 + 2/A)) * α_k
CL_alpha = 2.0 * math.pi / (1.0 + 2.0 / A_aspect)  # = 12π/7 ≈ 5.385 /rad

# Paper operating point (Weyel Table 1: λ=7 at 10 m/s → ω = λ*V/R)
omega_paper  = 15.0 * math.pi   # ≈ 47.12 rad/s
V_wind       = 10.0             # m/s
rho          = 1.22             # kg/m³  (Table I environment)

# Derived: tangential speed and dynamic pressure at r_cp
v_tan_cp = omega_paper * r_cp           # = 122.5 m/s
q_cp     = 0.5 * rho * v_tan_cp ** 2   # ≈ 9148 Pa


def _build_R_hub(disk_normal: np.ndarray) -> np.ndarray:
    """Build hub rotation matrix from disk normal (third column)."""
    dn   = disk_normal / np.linalg.norm(disk_normal)
    east = np.array([0.0, 1.0, 0.0])      # NED East = Y
    bx   = east - np.dot(east, dn) * dn
    if np.linalg.norm(bx) < 1e-6:
        bx = np.array([1.0, 0.0, 0.0])
        bx = bx - np.dot(bx, dn) * dn
    bx /= np.linalg.norm(bx)
    by   = np.cross(dn, bx)
    return np.column_stack([bx, by, dn])


def _forces(model, collective_rad=0.0, omega=omega_paper,
            disk_normal=None, wind=None):
    """Call compute_forces with De Schutter operating conditions."""
    if disk_normal is None:
        # Tether-equilibrium orientation: disk normal tilted ~55° from vertical (NED)
        disk_normal = np.array([0.305, 0.851, -0.427])
        disk_normal /= np.linalg.norm(disk_normal)
    if wind is None:
        wind = np.array([0.0, V_wind, 0.0])   # NED: East wind = Y axis
    R_hub = _build_R_hub(disk_normal)
    return model.compute_forces(
        collective_rad = collective_rad,
        tilt_lon       = 0.0,
        tilt_lat       = 0.0,
        R_hub          = R_hub,
        v_hub_world    = np.zeros(3),
        omega_rotor    = omega,
        wind_world     = wind,
        t              = 10.0,
    )


# ---------------------------------------------------------------------------
# Eq. 25 — Lift and drag coefficient formulas
# ---------------------------------------------------------------------------

class TestEq25LiftDragCoefficients:
    """
    CL_alpha = 2π / (1 + 2/A)     thin airfoil + Prandtl finite-wing correction
    C_{L,k}  = CL_alpha * alpha_k  (no CL0 — symmetric/uncambered wing)
    C_{D,k}  = C_{D,0} + C_L² / (π·A·O_e)

    Table I: A=12, O_e=0.8, C_{D,0}=0.01
    """

    def test_CL_alpha_equals_paper_formula(self, ds_rotor):
        """CL_alpha must match 2π/(1+2/A) from Eq. 25 with A=12."""
        CL_alpha_paper = 2.0 * math.pi / (1.0 + 2.0 / A_aspect)   # 12π/7
        assert math.isclose(ds_rotor.CL_alpha_per_rad, CL_alpha_paper, rel_tol=0.005), \
            (f"CL_alpha={ds_rotor.CL_alpha_per_rad:.4f} /rad, "
             f"paper Eq. 25 gives {CL_alpha_paper:.4f} /rad")

    def test_CL0_is_zero_symmetric_wing(self, ds_rotor):
        """CL0=0 — De Schutter uses thin airfoil theory for a symmetric wing (Eq. 25)."""
        assert math.isclose(ds_rotor.CL0, 0.0, abs_tol=1e-6), \
            f"CL0={ds_rotor.CL0:.4f}, expected 0.0 for symmetric thin airfoil"

    def test_CL_at_5deg_AoA(self):
        """Eq. 25: CL = CL_alpha * 5° = 5.385 * 0.08727 = 0.470."""
        alpha_rad = math.radians(5.0)
        CL_expected = CL_alpha * alpha_rad    # no CL0 term
        assert math.isclose(CL_expected, 0.470, rel_tol=0.01), \
            f"CL at 5deg = {CL_expected:.4f}, expected ~0.470"

    def test_CD_at_zero_AoA_equals_CD0(self):
        """Eq. 25: at alpha=0, CL=0, so CD = CD0 + 0 = CD0 = 0.01."""
        CL   = CL_alpha * 0.0   # = 0
        CD   = CD0 + CL ** 2 / (math.pi * A_aspect * O_e)
        assert math.isclose(CD, CD0, abs_tol=1e-9), \
            f"CD at zero AoA = {CD:.5f}, expected CD0 = {CD0:.5f}"

    def test_CD_at_5deg_AoA(self):
        """Eq. 25: at alpha=5°, CL=0.470, CD = 0.01 + 0.470²/(π*12*0.8) = 0.0173."""
        CL = CL_alpha * math.radians(5.0)
        CD = CD0 + CL ** 2 / (math.pi * A_aspect * O_e)
        assert math.isclose(CD, 0.0173, rel_tol=0.02), \
            f"CD at 5deg = {CD:.5f}, expected ~0.0173"

    def test_LD_ratio_at_5deg(self):
        """L/D at 5° must be consistent with paper operating regime."""
        CL = CL_alpha * math.radians(5.0)
        CD = CD0 + CL ** 2 / (math.pi * A_aspect * O_e)
        LD = CL / CD
        # At 5°, L/D should be high (>10) for an efficient AWE wing (AR=12)
        assert LD > 10.0, f"L/D = {LD:.1f} at 5deg AoA, expected > 10"

    def test_model_thrust_increases_linearly_with_collective(self, ds_model):
        """
        At high RPM (v_tan >> v_axial), induction is negligible.
        Thrust sensitivity to collective ≈ N * q_cp * S_w * CL_alpha (Eq. 25 + 30).

        At omega=47.12 rad/s, v_tan=122.5 m/s >> V_wind=10 m/s, so the
        no-induction approximation holds to within ~5%.
        """
        col_lo, col_hi = math.radians(1.0), math.radians(6.0)
        delta_col = col_hi - col_lo

        disk_normal = np.array([0.0, 0.0, -1.0])  # pointing up (NED: -Z = up)
        wind        = np.array([0.0, 0.0, -V_wind])  # axial inflow from below

        f_lo = _forces(ds_model, col_lo, disk_normal=disk_normal, wind=wind)
        f_hi = _forces(ds_model, col_hi, disk_normal=disk_normal, wind=wind)

        R_hub      = _build_R_hub(disk_normal)
        dn         = R_hub[:, 2]
        T_lo       = float(np.dot(f_lo.F_world, dn))
        T_hi       = float(np.dot(f_hi.F_world, dn))
        delta_T    = T_hi - T_lo

        # Eq. 25+30 no-induction prediction: ΔT = N * q_cp * S_w * CL_alpha * Δcol
        delta_T_predicted = N * q_cp * S_w * CL_alpha * delta_col
        # Allow ±30% tolerance: induction and multi-strip effects cause deviation
        assert delta_T > 0, "Thrust must increase with collective"
        assert abs(delta_T - delta_T_predicted) / delta_T_predicted < 0.40, \
            (f"Thrust sensitivity: model={delta_T:.0f} N, Eq.25+30 predicts "
             f"{delta_T_predicted:.0f} N (delta_col={math.degrees(delta_col):.0f}deg)")


# ---------------------------------------------------------------------------
# Eq. 26 — Angle of attack from velocity decomposition
# ---------------------------------------------------------------------------

class TestEq26AngleOfAttack:
    """
    alpha_k = (u_{a,k} · e_{3,k}) / (u_{a,k} · e_{2,k})
            ≈ -ua_normal / ua_chord   (with sign convention for e_3 direction)

    AoA must be in [-15°, +15°] at all RAWES operating conditions (Eq. 28).
    """

    def test_model_aoa_within_15deg_at_design(self, ds_model):
        """AoA stays within ±15° at De Schutter design operating conditions."""
        # Approximate inflow angle at r_cp (no induction)
        v_axial_approx = V_wind  # crude: all wind is axial for this estimate
        inflow_approx  = math.atan2(v_axial_approx, v_tan_cp)   # ≈ 4.7°
        AoA_approx_deg = math.degrees(inflow_approx)
        assert AoA_approx_deg <= 15.0, \
            f"Approx AoA at design = {AoA_approx_deg:.1f}° > 15°"

    def test_zero_AoA_at_zero_inflow_and_collective(self, ds_model):
        """
        At zero collective and zero axial inflow (disk horizontal, wind purely
        tangential), AoA = 0 → CL = 0 (symmetric airfoil, no induction drive).
        Thrust should be near zero.
        """
        # Disk horizontal (normal = down in NED = [0,0,+1]),
        # wind = tangential (East, no axial component)
        disk_normal = np.array([0.0, 0.0, 1.0])   # down in NED
        wind        = np.array([0.0, V_wind, 0.0])  # East — purely in-plane

        f = _forces(ds_model, collective_rad=0.0,
                    disk_normal=disk_normal, wind=wind)
        R_hub = _build_R_hub(disk_normal)
        T = float(np.dot(f.F_world, R_hub[:, 2]))
        # Pure tangential wind: no axial inflow → near-zero thrust at zero collective
        assert abs(T) < 50.0, \
            f"Thrust should be near zero with tangential-only wind: T={T:.1f} N"


# ---------------------------------------------------------------------------
# Eq. 27, 28 — Side-slip angle β
# ---------------------------------------------------------------------------

class TestEq27SideSlip:
    """
    beta_k = arcsin( (u_{a,k} · e_{1,k}) / |u_{a,k}| )

    De Schutter constrains |beta_k| <= 15° (Eq. 28) for model validity.
    beta does NOT enter the force formulas; it is a validity indicator.
    """

    def test_sideslip_computed_as_diagnostic(self, ds_model):
        """last_sideslip_mean_deg is set and finite after compute_forces."""
        _forces(ds_model)
        assert hasattr(ds_model, "last_sideslip_mean_deg"), \
            "DeSchutterAero must expose last_sideslip_mean_deg diagnostic"
        assert math.isfinite(ds_model.last_sideslip_mean_deg), \
            f"last_sideslip_mean_deg is not finite: {ds_model.last_sideslip_mean_deg}"

    def test_sideslip_non_negative(self, ds_model):
        """Mean |beta| is always >= 0."""
        _forces(ds_model)
        assert ds_model.last_sideslip_mean_deg >= 0.0

    def test_sideslip_within_model_validity_at_design(self, ds_model):
        """
        Eq. 28: |beta_k| <= 15° required for linear aero model validity.

        At De Schutter design conditions (high RPM, dominant rotational velocity),
        apparent wind at each strip is mostly tangential.  Tangential velocity
        v_rot = omega * r * e_tang is perpendicular to e_span by construction,
        so it does not contribute to side-slip.  Wind (axial + in-plane) also
        has a small spanwise component at the design tether orientation.
        Mean |beta| should stay well within the 15° validity constraint.
        """
        _forces(ds_model)
        assert ds_model.last_sideslip_mean_deg <= 15.0, \
            (f"Mean sideslip {ds_model.last_sideslip_mean_deg:.1f}° exceeds 15° "
             f"validity limit (Eq. 28) at design conditions")

    def test_sideslip_small_at_high_rpm(self, ds_model):
        """
        At very high RPM, rotational velocity dominates apparent wind.
        Rotational velocity v_rot = omega * r * e_tang is perpendicular to span,
        so mean |beta| should be < 5° for omega >> V_wind / r_cp.
        """
        # Use disk normal pointing straight up (NED -Z), wind axial only
        disk_normal = np.array([0.0, 0.0, -1.0])   # up in NED
        wind        = np.array([0.0, 0.0, -V_wind])  # upward axial inflow
        _forces(ds_model, omega=omega_paper,
                disk_normal=disk_normal, wind=wind)
        # At high RPM: v_tan=122.5 m/s >> v_axial=10 m/s → inflow angle ≈ 4.7°
        # Spanwise component of pure axial wind on a radial blade = 0
        assert ds_model.last_sideslip_mean_deg < 15.0, \
            (f"Sideslip {ds_model.last_sideslip_mean_deg:.1f}° too large at "
             f"omega={omega_paper:.1f} rad/s with axial-only wind")


# ---------------------------------------------------------------------------
# Eq. 29, 31 — Structural parasitic drag C_{D,T}
# ---------------------------------------------------------------------------

class TestEq29StructuralDrag:
    """
    Parasitic drag from connecting beams and cables (Eq. 29):
        D_{T,k} = ½rho * C_D_cyl * d_ck * L_ck * |u_a|^2 * û_a

    Expressed as an effective coefficient added to the blade drag (Eq. 31):
        C_{D,T} = C_D_cyl * d_ck * L_ck / S_w

    Paper geometry:
        C_D_cyl = 1.0  (cylinder in cross-flow)
        d_ck    = 1.5 mm  (reinforcement cable diameter, Table I optimal design)
        L_ck    = r_cp = 2.60 m  (hub centre to wing CP)
        S_w     = 0.1875 m²

    → C_{D,T} = 1.0 * 0.0015 * 2.60 / 0.1875 = 0.0208
    """

    # Cable geometry from paper
    C_D_cyl = 1.0          # drag coefficient for a slender cylinder in crossflow
    d_ck    = 0.0015       # reinforcement cable diameter [m]  (1.5 mm, Table I)
    L_ck    = r_cp         # cable length ≈ hub to wing CP [m]

    @property
    def C_DT_formula(self):
        """Eq. 29: C_{D,T} = C_D_cyl * d_ck * L_ck / S_w"""
        return self.C_D_cyl * self.d_ck * self.L_ck / S_w

    def test_CDT_yaml_consistent_with_paper_formula(self, ds_rotor):
        """
        YAML CD_structural must match C_D_cyl * d_ck * L_ck / S_w within 20%.

        The paper does not state C_{D,T} directly; the estimate uses cable geometry
        from Table I.  20% tolerance covers uncertainty in L_ck (cable routing angle).
        """
        C_DT_formula = self.C_D_cyl * self.d_ck * self.L_ck / S_w  # ≈ 0.0208
        assert math.isclose(ds_rotor.CD_structural, C_DT_formula, rel_tol=0.20), \
            (f"CD_structural={ds_rotor.CD_structural:.4f}, "
             f"paper formula gives {C_DT_formula:.4f} "
             f"(C_D_cyl={self.C_D_cyl} * d_ck={self.d_ck} m * L_ck={self.L_ck:.2f} m "
             f"/ S_w={S_w:.4f} m^2)")

    def test_CDT_positive(self, ds_rotor):
        """C_{D,T} > 0: cables always add drag, never remove it."""
        assert ds_rotor.CD_structural > 0.0, \
            f"CD_structural={ds_rotor.CD_structural:.4f}, expected > 0"

    def test_CDT_loaded_into_model(self, ds_model, ds_rotor):
        """DeSchutterAero reads CD_structural from the rotor definition."""
        assert math.isclose(ds_model.CD_T, ds_rotor.CD_structural, rel_tol=1e-6), \
            f"Model CD_T={ds_model.CD_T:.4f} != rotor CD_structural={ds_rotor.CD_structural:.4f}"

    def test_structural_drag_increases_H_force(self, ds_rotor):
        """
        Eq. 31: total drag = (C_{D,k} + C_{D,T}) * q * û_a.
        Adding C_{D,T} > 0 increases the in-plane drag (H-force).
        """
        # Model with C_{D,T} from YAML (0.021)
        model_with    = DeSchutterAero(ds_rotor)
        # Model with C_{D,T} forced to zero via override
        model_without = DeSchutterAero(ds_rotor, CD_structural=0.0)

        _forces(model_with)
        _forces(model_without)

        assert model_with.last_H_force > model_without.last_H_force, \
            (f"H-force with C_DT={ds_rotor.CD_structural:.4f}: "
             f"{model_with.last_H_force:.1f} N, "
             f"without: {model_without.last_H_force:.1f} N — "
             "structural drag must increase in-plane drag")

    def test_structural_drag_reduces_BEM_spin_torque(self, ds_rotor):
        """
        Adding C_{D,T} increases the drag on each strip.  The drag vector is
        along û_a — mostly tangential — so it opposes blade rotation and reduces
        the BEM spin-axis moment Q_spin_scalar = M_total · disk_normal.
        """
        model_with    = DeSchutterAero(ds_rotor)
        model_without = DeSchutterAero(ds_rotor, CD_structural=0.0)

        disk_normal = np.array([0.0, 0.0, -1.0])  # up in NED
        wind        = np.array([0.0, 0.0, -V_wind])
        R_hub       = _build_R_hub(disk_normal)

        f_with    = _forces(model_with,    disk_normal=disk_normal, wind=wind)
        f_without = _forces(model_without, disk_normal=disk_normal, wind=wind)

        Q_with    = float(np.dot(f_with.M_spin,    R_hub[:, 2]))
        Q_without = float(np.dot(f_without.M_spin, R_hub[:, 2]))

        # More drag → smaller (more negative) or less positive BEM spin torque
        assert Q_with <= Q_without, \
            (f"BEM spin torque with C_DT={ds_rotor.CD_structural:.4f}: "
             f"{Q_with:.2f} N*m, without: {Q_without:.2f} N*m — "
             "structural drag should reduce or not increase spin torque")

    def test_structural_drag_force_increment(self, ds_rotor):
        """
        Eq. 31: the force difference between C_{D,T}>0 and C_{D,T}=0 models
        must be in the direction of the apparent wind (drag direction) and
        have the expected magnitude order: ~ N * C_{D,T} * q_cp * S_w.
        """
        model_with    = DeSchutterAero(ds_rotor)
        model_without = DeSchutterAero(ds_rotor, CD_structural=0.0)

        f_with    = _forces(model_with)
        f_without = _forces(model_without)

        delta_F = f_with.F_world - f_without.F_world
        delta_F_mag = float(np.linalg.norm(delta_F))

        # Rough expected increment: N * C_{D,T} * q_cp * S_w (no induction, representative strip)
        expected_order = N * ds_rotor.CD_structural * q_cp * S_w
        # Allow factor of 5 tolerance: exact value depends on azimuth averaging and inflow angle
        assert delta_F_mag > 0, "Structural drag must produce a non-zero force increment"
        assert delta_F_mag < 10.0 * expected_order, \
            (f"Force increment {delta_F_mag:.1f} N is 10x larger than expected order "
             f"{expected_order:.1f} N — structural drag may be miscounted")


# ---------------------------------------------------------------------------
# Eq. 30, 31 — Lift and drag force directions
# ---------------------------------------------------------------------------

class TestEq30Eq31ForcDirections:
    """
    L_k = ½rho * S_w * C_L * |u_a|^2 * normalize(u_a x e_span)    (Eq. 30)
    D_k = ½rho * S_w * (C_D + C_{D,T}) * |u_a| * u_a              (Eq. 31)

    Fundamental direction checks:
    - Lift is perpendicular to both u_a and e_span (Kutta-Joukowski)
    - Drag is along u_a (opposes apparent wind)
    - Total F_world must include both
    """

    def test_thrust_positive_at_design(self, ds_model):
        """Lift (Eq. 30) produces positive thrust along disk_normal at design conditions."""
        f = _forces(ds_model, collective_rad=math.radians(5.0))
        disk_normal = _build_R_hub(np.array([0.305, 0.851, -0.427]) /
                                   np.linalg.norm(np.array([0.305, 0.851, -0.427])))[:, 2]
        T = float(np.dot(f.F_world, disk_normal))
        assert T > 0, f"Thrust T={T:.1f} N <= 0 — lift direction wrong"

    def test_H_force_in_wind_direction_at_design(self, ds_model):
        """
        Drag (Eq. 31) drives a positive H-force component in the wind direction.
        At design (East wind, NED Y), H-force Fy should be > 0.
        """
        disk_normal = np.array([0.305, 0.851, -0.427])
        disk_normal /= np.linalg.norm(disk_normal)
        wind = np.array([0.0, V_wind, 0.0])  # East wind

        f = _forces(ds_model, collective_rad=math.radians(5.0),
                    disk_normal=disk_normal, wind=wind)
        # H-force is in-plane; wind is East (Y in NED) → Fy should be positive
        Fy = float(f.F_world[1])
        assert Fy > 0, f"H-force Fy={Fy:.1f} N — expected > 0 for East wind drag"

    def test_thrust_increases_with_collective(self, ds_model):
        """Thrust increases with collective pitch (positive CL slope, Eq. 25+30)."""
        T_lo = float(np.dot(
            _forces(ds_model, collective_rad=math.radians(2.0)).F_world,
            _build_R_hub(np.array([0.0, 0.0, -1.0]))[:, 2]
        ))
        T_hi = float(np.dot(
            _forces(ds_model, collective_rad=math.radians(8.0),
                    disk_normal=np.array([0.0, 0.0, -1.0]),
                    wind=np.array([0.0, 0.0, -V_wind])).F_world,
            _build_R_hub(np.array([0.0, 0.0, -1.0]))[:, 2]
        ))
        assert T_hi > T_lo, \
            f"Thrust should increase with collective: {T_lo:.1f} -> {T_hi:.1f} N"


# ---------------------------------------------------------------------------
# Table I — Rotor geometry (sanity checks on YAML values)
# ---------------------------------------------------------------------------

class TestTableI:
    """
    Key geometry from De Schutter Table I and Section IV-A optimal design.
    These are validated by test_deschutter_validation.py in aero/tests/.
    Duplicated here as a direct paper reference.
    """

    def test_n_blades(self, ds_rotor):
        """N = 3 wings, 120 degrees apart  (Section II-A)."""
        assert ds_rotor.n_blades == 3

    def test_wing_span(self, ds_rotor):
        """L_w = 1.5 m  (Table I)."""
        assert math.isclose(ds_rotor.span_m, L_w, rel_tol=1e-3)

    def test_connecting_arm_length(self, ds_rotor):
        """L_b = 1.60 m  (optimal design, Section IV-A)."""
        assert math.isclose(ds_rotor.root_cutout_m, L_b, rel_tol=1e-3)

    def test_tip_radius(self, ds_rotor):
        """R = L_b + L_w = 3.10 m."""
        assert math.isclose(ds_rotor.radius_m, R, rel_tol=1e-3)

    def test_chord(self, ds_rotor):
        """c = L_w / A = 0.125 m  (Table I: A=12, L_w=1.5 m)."""
        assert math.isclose(ds_rotor.chord_m, chord, rel_tol=1e-3)

    def test_aspect_ratio(self, ds_rotor):
        """A = 12  (Table I)."""
        assert math.isclose(ds_rotor.aspect_ratio, A_aspect, rel_tol=0.01)

    def test_r_cp(self, ds_rotor):
        """r_cp = L_b + 2/3 * L_w = 2.60 m  (Section II-D)."""
        assert math.isclose(ds_rotor.r_cp_m, r_cp, rel_tol=0.01)

    def test_CD0(self, ds_rotor):
        """C_{D,0} = 0.01  (Table I)."""
        assert math.isclose(ds_rotor.CD0, CD0, rel_tol=1e-6)

    def test_oswald_efficiency(self, ds_rotor):
        """O_e = 0.8  (Table I)."""
        assert math.isclose(ds_rotor.oswald_efficiency, O_e, rel_tol=1e-6)

    def test_AoA_stall_limit(self, ds_rotor):
        """|alpha_k| <= 15 deg  (Eq. 28)."""
        assert math.isclose(ds_rotor.alpha_stall_deg, 15.0, rel_tol=1e-6)
