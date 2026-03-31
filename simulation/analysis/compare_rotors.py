"""
compare_rotors.py — Static + dynamic comparison of beaupoil_2026 vs de_schutter_2018.

Two sections:
  1. Static sweep:  thrust, CL, v_inplane, v_axial as xi (disk tilt from wind) varies
                    from 5° to 85° at fixed wind speed.  Shows where models diverge
                    and highlights the near-edgewise regime.
  2. Pumping cycle: run test_deschutter_cycle._run_deschutter_cycle() with each rotor.
                    Compare reel-out / reel-in tension and net energy.

Usage:
    python simulation/analysis/compare_rotors.py           # print tables
    python simulation/analysis/compare_rotors.py --plot    # also produce PNG plots
"""

import math
import sys
import argparse
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))   # analysis/ dir

import rotor_definition as rd
from aero import create_aero
from frames import build_orb_frame
from aero_strip import StripThrust, from_rotor as strip_from_rotor

# ── common parameters ────────────────────────────────────────────────────────
WIND_SPEED   = 10.0          # m/s
OMEGA_SPIN   = 20.0          # rad/s (both rotors — equilibrium for beaupoil)
COLLECTIVE   = math.radians(-5.7)   # equilibrium collective [rad]
TETHER_ELEV  = math.radians(30.0)   # elevation angle of tether above horizontal

XI_DEGREES   = list(range(5, 91, 5))   # ξ sweep: disk tilt from wind direction


# ── geometry helpers ─────────────────────────────────────────────────────────

def _build_R_from_bz(body_z: np.ndarray) -> np.ndarray:
    """Build a rotation matrix whose third column is body_z."""
    bz = body_z / np.linalg.norm(body_z)
    return build_orb_frame(bz)


def _body_z_for_xi(xi_deg: float, wind_dir: np.ndarray, tether_elev_rad: float) -> np.ndarray:
    """
    body_z unit vector at disk-tilt angle xi from the horizontal wind direction.

    xi=0°  → body_z parallel to wind (propeller-mode — degenerate but shown for reference)
    xi=30° → typical reel-out tilt (natural tether equilibrium at beta=30°)
    xi=90° → body_z perpendicular to wind (rotor axis vertical, edgewise flow)

    Rotation is in the vertical plane containing the wind direction.
    body_z = cos(xi)·wind_dir_hat + sin(xi)·[0,0,1]
    """
    wd = wind_dir / np.linalg.norm(wind_dir)
    xi = math.radians(xi_deg)
    bz = math.cos(xi) * wd + math.sin(xi) * np.array([0.0, 0.0, 1.0])
    return bz / np.linalg.norm(bz)


# ── static sweep ─────────────────────────────────────────────────────────────

def _static_sweep(rotor, label: str) -> list[dict]:
    """Sweep xi 5°→90°, compute aero forces at fixed collective and omega."""
    aero    = create_aero(rotor)
    wind    = np.array([WIND_SPEED, 0.0, 0.0])
    v_hub   = np.zeros(3)

    rows = []
    for xi_deg in XI_DEGREES:
        bz   = _body_z_for_xi(xi_deg, wind, TETHER_ELEV)
        R    = _build_R_from_bz(bz)

        # Use t >> ramp_time so ramp=1
        forces = aero.compute_forces(
            collective_rad = COLLECTIVE,
            tilt_lon       = 0.0,
            tilt_lat       = 0.0,
            R_hub          = R,
            v_hub_world    = v_hub,
            omega_rotor    = OMEGA_SPIN,
            wind_world     = wind,
            t              = 1000.0,
        )

        T_thrust  = aero.last_T
        v_axial   = aero.last_v_axial
        v_inplane = aero.last_v_inplane
        v_i       = aero.last_v_i

        # Thrust component along tether direction (tether at elevation angle)
        tether_dir = np.array([math.cos(TETHER_ELEV), 0.0, math.sin(TETHER_ELEV)])
        T_tether   = float(np.dot(forces[:3], tether_dir))

        # Thrust component along vertical (holds hub up)
        T_vert     = float(forces[2])

        rows.append({
            "xi_deg":     xi_deg,
            "T_thrust_N": T_thrust,
            "T_tether_N": T_tether,
            "T_vert_N":   T_vert,
            "v_axial":    v_axial,
            "v_inplane":  v_inplane,
            "v_i":        v_i,
            "Q_spin":     aero.last_Q_spin,
        })

    return rows


def _strip_sweep(rotor, label: str) -> list[dict]:
    """Sweep xi using the independent StripThrust model (Glauert oblique-flow, radial integration)."""
    st   = strip_from_rotor(rotor, n_strips=20)
    wind = WIND_SPEED

    rows = []
    for xi_deg in XI_DEGREES:
        xi_rad   = math.radians(xi_deg)
        v_axial  = wind * math.cos(xi_rad)   # component along disk normal
        v_inplane= wind * math.sin(xi_rad)   # component in disk plane

        res = st.thrust(v_axial, v_inplane, OMEGA_SPIN, COLLECTIVE)

        # Betz limit for this flow condition
        v_tot    = math.sqrt(v_axial**2 + v_inplane**2)
        betz_max = 0.5 * 1.22 * v_tot**2 * st.disk_area * 0.89

        rows.append({
            "xi_deg":    xi_deg,
            "T_N":       res["T_N"],
            "v_i":       res["v_i"],
            "CL_mean":   res["CL_mean"],
            "betz_frac": res["T_N"] / max(betz_max, 0.1),
            "betz_max":  betz_max,
        })

    return rows


def _print_three_way(rows_b, rows_d, rows_strip_b, rows_strip_d) -> None:
    """Three-way comparison: lumped BEM vs strip-theory for both rotors."""
    print("\n── Three-way comparison: lumped BEM vs strip-theory with Glauert induction ──")
    print("  (StripThrust = Glauert oblique-flow induction + 20-strip radial integration)")
    print()
    hdr = (f"{'xi':>4}  {'BEM_B':>7}  {'Str_B':>7}  {'Betz_B%':>7}  "
           f"{'BEM_DS':>7}  {'Str_DS':>7}  {'Betz_DS%':>8}  BetzMax")
    print(f"{'':>4}  {'N':>7}  {'N':>7}  {'':>7}  "
          f"{'N':>7}  {'N':>7}  {'':>8}  N")
    print("  " + "─" * len(hdr))

    for rb, rd_, sb, sd in zip(rows_b, rows_d, rows_strip_b, rows_strip_d):
        xi = rb["xi_deg"]
        print(f"{xi:4d}  {rb['T_thrust_N']:7.1f}  {sb['T_N']:7.1f}  {sb['betz_frac']*100:6.0f}%  "
              f"{rd_['T_thrust_N']:7.1f}  {sd['T_N']:7.1f}  {sd['betz_frac']*100:7.0f}%  "
              f"{sb['betz_max']:6.0f}")
    print()
    print("  Betz% = T_thrust / (0.89 × 0.5 ρ V² A) — values > 100% are physically implausible")
    print()

    # Flag violations
    violations_b = [(s["xi_deg"], rb["T_thrust_N"], s["betz_frac"])
                    for rb, s in zip(rows_b, rows_strip_b) if s["betz_frac"] > 1.0]
    if violations_b:
        print(f"  !! beaupoil_2026 lumped BEM exceeds Betz limit at "
              f"{len(violations_b)}/{len(rows_b)} ξ values.")
        print(f"     Max overshoot at ξ=5°: {rows_strip_b[0]['betz_frac']*100:.0f}% of Betz limit")
        print(f"     Root cause: lumped BEM gives {rows_b[0]['T_thrust_N']/max(rows_strip_b[0]['T_N'],0.01):.1f}× "
              f"strip-theory value — single-point r_cp overcounts outer-strip thrust")
    print()


def _print_sweep(rows_b: list[dict], rows_d: list[dict]) -> None:
    """Print side-by-side comparison table."""
    hdr = (f"{'ξ°':>4}  "
           f"{'T_thrust_B':>10}  {'T_thrust_DS':>11}  {'ratio':>5}  "
           f"{'T_tether_B':>10}  {'T_tether_DS':>11}  "
           f"{'v_axial_B':>9}  {'v_axial_DS':>10}  "
           f"{'v_inplane':>9}")
    print("\n── Static thrust sweep  (collective=−5.7°, ω=20 rad/s, V_wind=10 m/s) ──")
    print(hdr)
    print("─" * len(hdr))

    for rb, rd_ in zip(rows_b, rows_d):
        xi      = rb["xi_deg"]
        T_b     = rb["T_thrust_N"]
        T_d     = rd_["T_thrust_N"]
        ratio   = T_b / T_d if T_d > 1e-3 else float("inf")
        Tt_b    = rb["T_tether_N"]
        Tt_d    = rd_["T_tether_N"]
        va_b    = rb["v_axial"]
        va_d    = rd_["v_axial"]
        vip     = rb["v_inplane"]
        print(f"{xi:4d}  {T_b:10.1f}  {T_d:11.1f}  {ratio:5.1f}  "
              f"{Tt_b:10.1f}  {Tt_d:11.1f}  "
              f"{va_b:9.3f}  {va_d:10.3f}  {vip:9.3f}")

    print()
    print("Columns: T_thrust = raw BEM thrust along disk normal  [N]")
    print("         T_tether = thrust projected along tether     [N]")
    print("         v_axial  = wind component along disk normal  [m/s]")
    print("         v_inplane= wind component in disk plane      [m/s]")
    print()
    print("Near-edgewise regime (ξ > 70°):")
    for rb, rd_ in zip(rows_b, rows_d):
        if rb["xi_deg"] >= 70:
            print(f"  ξ={rb['xi_deg']}°  v_axial_B={rb['v_axial']:.3f}  "
                  f"v_i_B={rb['v_i']:.3f}  T_thrust_B={rb['T_thrust_N']:.1f}N  "
                  f"T_thrust_DS={rd_['T_thrust_N']:.1f}N")
    print()


# ── pumping cycle comparison ──────────────────────────────────────────────────

def _run_pumping_cycle(rotor, label: str) -> dict:
    """Run one De Schutter pumping cycle with the given rotor definition."""
    import math as _math
    from dynamics   import RigidBodyDynamics
    from tether     import TetherModel
    from controller import compute_swashplate_from_state, TensionPI, orbit_tracked_body_z_eq
    from planner    import DeschutterPlanner, Q_IDENTITY, quat_apply, quat_is_identity

    DT            = 1.0 / 400.0
    ANCHOR        = np.zeros(3)
    POS0          = np.array([46.258, 14.241, 12.530])
    VEL0          = np.array([-0.257,  0.916, -0.093])
    BODY_Z0       = np.array([0.851018, 0.305391, 0.427206])
    OMEGA_SPIN0   = 20.148
    REST_LENGTH0  = 49.949
    T_AERO_OFFSET = 45.0
    I_SPIN_KGMS2  = 10.0
    OMEGA_SPIN_MIN= 0.5
    WIND          = np.array([10.0, 0.0, 0.0])
    T_REEL_OUT    = 30.0
    T_REEL_IN     = 30.0
    T_TRANSITION  =  5.0
    V_REEL_OUT    =  0.4
    V_REEL_IN     =  0.4
    TENSION_OUT   = 200.0
    TENSION_IN    =  20.0
    BODY_Z_SLEW   = 0.12

    dyn_kwargs = rotor.dynamics_kwargs()
    dyn = RigidBodyDynamics(
        mass    = dyn_kwargs["mass"],
        I_body  = dyn_kwargs["I_body"],
        I_spin  = 0.0,
        pos0    = POS0.tolist(),
        vel0    = VEL0.tolist(),
        R0      = build_orb_frame(BODY_Z0),
        omega0  = [0.0, 0.0, 0.0],
        z_floor = 1.0,
    )
    aero_obj = create_aero(rotor)
    tether   = TetherModel(anchor_enu=ANCHOR, rest_length=REST_LENGTH0,
                           axle_attachment_length=0.0)

    trajectory = DeschutterPlanner(
        t_reel_out      = T_REEL_OUT,
        t_reel_in       = T_REEL_IN,
        t_transition    = T_TRANSITION,
        v_reel_out      = V_REEL_OUT,
        v_reel_in       = V_REEL_IN,
        tension_out     = TENSION_OUT,
        tension_in      = TENSION_IN,
        wind_enu        = WIND,
    )

    tension_ctrl     = TensionPI(setpoint_n=TENSION_OUT)
    ic_tether_dir0   = POS0 / np.linalg.norm(POS0)
    ic_body_z_eq0    = BODY_Z0.copy()
    body_z_eq_slewed = BODY_Z0.copy()

    hub_state    = dyn.state
    omega_spin   = OMEGA_SPIN0
    t_total      = T_REEL_OUT + T_REEL_IN
    n_steps      = int(t_total / DT)

    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = 0.0
    body_z_eq      = BODY_Z0.copy()
    cmd            = {"attitude_q": Q_IDENTITY.copy(), "thrust": 1.0,
                      "winch_speed_ms": 0.0, "phase": "reel-out"}

    tensions_out = []
    tensions_in  = []
    altitudes    = []
    floor_hits   = 0
    energy_out   = 0.0
    energy_in    = 0.0
    xi_outs      = []
    xi_ins       = []

    for i in range(n_steps):
        t = i * DT

        state_pkt = {
            "pos_enu":    hub_state["pos"],
            "vel_enu":    hub_state["vel"],
            "tension_n":  tension_now,
            "omega_spin": omega_spin,
            "t_free":     t,
        }
        cmd = trajectory.step(state_pkt, DT)

        ws = cmd["winch_speed_ms"]
        if ws < 0.0:
            tether.rest_length = max(REST_LENGTH0, tether.rest_length + ws * DT)
        else:
            tether.rest_length += ws * DT

        tension_ctrl.setpoint = cmd["thrust"] * TENSION_OUT
        collective_rad = tension_ctrl.update(tension_now, DT)

        bz_tether = orbit_tracked_body_z_eq(hub_state["pos"], ic_tether_dir0, ic_body_z_eq0)
        _aq = cmd["attitude_q"]
        if quat_is_identity(_aq):
            body_z_eq_slewed = bz_tether.copy()
            body_z_eq        = bz_tether
        else:
            bz_target = quat_apply(_aq, np.array([0.0, 0.0, 1.0]))
            _cos_a    = float(np.clip(np.dot(body_z_eq_slewed, bz_target), -1.0, 1.0))
            _angle    = _math.acos(_cos_a)
            _max_step = BODY_Z_SLEW * DT
            if _angle > 1e-6:
                _alpha_slew = min(1.0, _max_step / _angle)
                _sin_a = _math.sin(_angle)
                if _sin_a > 1e-9:
                    body_z_eq_slewed = (
                        _math.sin((1.0 - _alpha_slew) * _angle) / _sin_a * body_z_eq_slewed
                        + _math.sin(_alpha_slew * _angle) / _sin_a * bz_target
                    )
                    body_z_eq_slewed = body_z_eq_slewed / np.linalg.norm(body_z_eq_slewed)
            body_z_eq = body_z_eq_slewed

        sw = compute_swashplate_from_state(hub_state=hub_state, anchor_pos=ANCHOR,
                                           body_z_eq=body_z_eq)

        forces = aero_obj.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = sw["tilt_lon"],
            tilt_lat       = sw["tilt_lat"],
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t,
        )

        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        forces[0:3] += tf
        forces[3:6] += tm
        tension_now = tether._last_info.get("tension", 0.0)

        bz_cur = hub_state["R"][:, 2]
        xi_cur = float(np.degrees(np.arccos(
            np.clip(np.dot(bz_cur, WIND / np.linalg.norm(WIND)), -1.0, 1.0))))

        if cmd["phase"] == "reel-out":
            energy_out += tension_now * V_REEL_OUT * DT
            tensions_out.append(tension_now)
            xi_outs.append(xi_cur)
        else:
            energy_in  += tension_now * V_REEL_IN * DT
            tensions_in.append(tension_now)
            xi_ins.append(xi_cur)

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero_obj.last_Q_spin / I_SPIN_KGMS2 * DT)
        M_orbital  = forces[3:6] - aero_obj.last_M_spin
        M_orbital += -50.0 * hub_state["omega"]
        hub_state  = dyn.step(forces[0:3], M_orbital, DT, omega_spin=omega_spin)

        if hub_state["pos"][2] <= 1.05:
            floor_hits += 1
        altitudes.append(hub_state["pos"][2])

    return {
        "label":          label,
        "floor_hits":     floor_hits,
        "min_alt_m":      float(np.min(altitudes)),
        "mean_T_out_N":   float(np.mean(tensions_out)) if tensions_out else 0.0,
        "mean_T_in_N":    float(np.mean(tensions_in))  if tensions_in  else 0.0,
        "max_T_out_N":    float(np.max(tensions_out))  if tensions_out else 0.0,
        "energy_out_J":   energy_out,
        "energy_in_J":    energy_in,
        "net_energy_J":   energy_out - energy_in,
        "mean_xi_out":    float(np.mean(xi_outs)) if xi_outs else 0.0,
        "mean_xi_in":     float(np.mean(xi_ins))  if xi_ins  else 0.0,
        "omega_spin_end": omega_spin,
    }


def _print_pumping(results: list[dict]) -> None:
    print("── Pumping cycle comparison (1 cycle, De Schutter tilt strategy) ──")
    for r in results:
        print(f"\n  Rotor: {r['label']}")
        print(f"    Floor hits:         {r['floor_hits']}")
        print(f"    Min altitude:       {r['min_alt_m']:.2f} m")
        print(f"    Reel-out mean T:    {r['mean_T_out_N']:.1f} N   (peak {r['max_T_out_N']:.1f} N)   ξ={r['mean_xi_out']:.1f}°")
        print(f"    Reel-in  mean T:    {r['mean_T_in_N']:.1f} N   ξ={r['mean_xi_in']:.1f}°")
        print(f"    Tension ratio:      {r['mean_T_in_N']/r['mean_T_out_N']:.3f}  (< 1.0 = asymmetry achieved)")
        print(f"    Energy out:         {r['energy_out_J']:.0f} J")
        print(f"    Energy in:          {r['energy_in_J']:.0f} J")
        print(f"    Net energy:         {r['net_energy_J']:.0f} J  {'✓ POSITIVE' if r['net_energy_J'] > 0 else '✗ NEGATIVE'}")
        print(f"    Final ω_spin:       {r['omega_spin_end']:.2f} rad/s")
    print()


# ── edgewise flow commentary ──────────────────────────────────────────────────

def _print_edgewise_analysis(rows_b: list[dict], rows_d: list[dict]) -> None:
    print("── Near-edgewise flow (ξ > 70°) — model validity analysis ──")
    print("""
The BEM actuator-disk model assumes primarily axial inflow.  It uses:
  T  = 2 ρ A v_i (|v_axial| + v_i)    [momentum theory, De Schutter Eq. 17]
  v_eff = v_axial + v_i                [axial effective velocity for BEM strip]

This breaks down when v_axial → 0 (wind perpendicular to disk):

  Problem 1 — Turbulent wake state:
    At ξ ≈ 90° the rotor is in edgewise flight, not axial flow.  Real thrust
    no longer obeys the actuator-disk equation.  The model underestimates v_i
    and overestimates thrust (the disk does not sweep through clean air).

  Problem 2 — High-advance-ratio effects:
    μ = v_inplane / (ω·R_tip) at ξ=90° is large (~10·0/20·2.5 = 0 but
    v_inplane = 10 m/s → μ = 10/(20×2.5) = 0.20).  At μ > 0.15 blade
    unloading on the retreating side is significant.  The lumped single-point
    BEM ignores this asymmetry.

  Problem 3 — AoA clamp becomes load-bearing:
    With v_axial ≈ 0 and v_i ≈ 0, v_eff ≈ 0 → inflow_angle → 0 →
    AoA = collective alone.  The BEM strip effectively runs at a constant
    AoA = collective_rad.  The AoA clamp (±12° / ±15°) therefore clamps
    the entire collective range, not just extreme conditions.

  Problem 4 — Induced velocity iteration diverges:
    With T_raw ≈ 0 (v_eff ≈ 0), the quadratic gives v_i ≈ 0.
    Three-pass iteration converges but to an unphysical zero-thrust solution.
""")

    print("Observed v_axial and v_i values near ξ=90° (beaupoil_2026):")
    for rb in rows_b:
        if rb["xi_deg"] >= 65:
            print(f"  ξ={rb['xi_deg']}°  v_axial={rb['v_axial']:+.3f}  "
                  f"v_i={rb['v_i']:.3f}  T={rb['T_thrust_N']:.1f}N  "
                  f"v_inplane={rb['v_inplane']:.2f}")

    print("""
  Physically valid regime for the lumped-BEM model: ξ < 70° (v_axial > ~1 m/s).
  The reel-in tilt of 55° used in test_deschutter_cycle.py is intentionally
  chosen to stay inside this regime.

Possible improvements for edgewise flow:
  1. Vortex ring correction (Coleman et al.):
       For v_axial between −v_i and 0 (vortex ring state) and
       for v_axial between 0 and −v_i (turbulent wake state) use:
         k_w = (v_axial/v_i)² correction to v_i estimate.
       Already used in some helicopter BEM codes.

  2. Lifting-line / free-wake model (BEM → FWM):
       Replace lumped single-strip BEM with blade-element integration
       over 10–20 strips using a prescribed or free wake model.
       Captures radial load distribution and retreating-blade unloading.
       Implemented in CAMRAD II, RCAS, and open-source CACTUS.

  3. CFD lookup table (C81/C8-4):
       Pre-compute thrust coefficient C_T vs (μ, C_T_prev) sweep in
       a 2D lookup table from CFD/OpenFOAM.  Interpolate in real-time.
       Would require C81 lookup tables pre-computed from CFD/OpenFOAM.

  4. Empirical power correction factor (quick fix):
       Multiply T by a correction factor f(ξ) that smoothly goes to
       zero as ξ → 90°.  f(ξ) = sin(ξ)^p, p ≈ 0.5–1.0.  Zero theory
       basis but prevents the model producing physically implausible
       values without a full aerodynamic model replacement.
""")


# ── plotting ──────────────────────────────────────────────────────────────────

def _plot(rows_b, rows_d, pump_b, pump_d):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib not available — skipping plots")
        return

    xi  = [r["xi_deg"]     for r in rows_b]
    T_b = [r["T_thrust_N"] for r in rows_b]
    T_d = [r["T_thrust_N"] for r in rows_d]
    Tt_b= [r["T_tether_N"] for r in rows_b]
    Tt_d= [r["T_tether_N"] for r in rows_d]
    va_b= [r["v_axial"]    for r in rows_b]
    va_d= [r["v_axial"]    for r in rows_d]

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Rotor Model Comparison: beaupoil_2026 vs de_schutter_2018\n"
                 f"V_wind={WIND_SPEED} m/s, ω={OMEGA_SPIN} rad/s, col={math.degrees(COLLECTIVE):.1f}°",
                 fontsize=12)
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    # 1. Thrust vs xi
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(xi, T_b, 'b-o', ms=4, label="beaupoil_2026")
    ax1.plot(xi, T_d, 'r-s', ms=4, label="de_schutter_2018")
    ax1.axvspan(70, 90, alpha=0.12, color='orange', label="edgewise regime")
    ax1.set_xlabel("ξ — disk tilt from wind [°]")
    ax1.set_ylabel("T (disk normal) [N]")
    ax1.set_title("Thrust vs. Disk Tilt")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 2. Thrust along tether vs xi
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(xi, Tt_b, 'b-o', ms=4, label="beaupoil_2026")
    ax2.plot(xi, Tt_d, 'r-s', ms=4, label="de_schutter_2018")
    ax2.axhline(0, color='k', lw=0.8, ls='--')
    ax2.axvspan(70, 90, alpha=0.12, color='orange')
    ax2.set_xlabel("ξ [°]")
    ax2.set_ylabel("T·tether_dir [N]")
    ax2.set_title("Thrust Component Along Tether")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # 3. v_axial vs xi
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(xi, va_b, 'b-o', ms=4, label="beaupoil_2026")
    ax3.plot(xi, va_d, 'r-s', ms=4, label="de_schutter_2018")
    ax3.axhline(0, color='k', lw=0.8, ls='--')
    ax3.axvspan(70, 90, alpha=0.12, color='orange', label="edgewise regime")
    ax3.set_xlabel("ξ [°]")
    ax3.set_ylabel("v_axial [m/s]")
    ax3.set_title("Axial Wind Component")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 4. Thrust ratio
    ax4 = fig.add_subplot(gs[1, 0])
    ratio = [b / d if d > 0.1 else float("nan") for b, d in zip(T_b, T_d)]
    ax4.plot(xi, ratio, 'g-^', ms=5)
    ax4.axhline(1.0, color='k', lw=0.8, ls='--')
    ax4.axvspan(70, 90, alpha=0.12, color='orange')
    ax4.set_xlabel("ξ [°]")
    ax4.set_ylabel("T_beaupoil / T_deschutter")
    ax4.set_title("Thrust Ratio (beaupoil / deschutter)")
    ax4.grid(True, alpha=0.3)

    # 5. Pumping bar chart
    ax5 = fig.add_subplot(gs[1, 1])
    labels   = ["beaupoil_2026", "de_schutter_2018"]
    t_out    = [pump_b["mean_T_out_N"], pump_d["mean_T_out_N"]]
    t_in     = [pump_b["mean_T_in_N"],  pump_d["mean_T_in_N"]]
    x = np.arange(2)
    w = 0.35
    ax5.bar(x - w/2, t_out, w, label="Reel-out mean T", color='steelblue')
    ax5.bar(x + w/2, t_in,  w, label="Reel-in mean T",  color='tomato')
    ax5.set_xticks(x)
    ax5.set_xticklabels(labels, fontsize=8)
    ax5.set_ylabel("Mean tension [N]")
    ax5.set_title("Pumping Cycle Tensions")
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3, axis='y')

    # 6. Energy bar chart
    ax6 = fig.add_subplot(gs[1, 2])
    net  = [pump_b["net_energy_J"], pump_d["net_energy_J"]]
    e_o  = [pump_b["energy_out_J"], pump_d["energy_out_J"]]
    e_i  = [pump_b["energy_in_J"],  pump_d["energy_in_J"]]
    ax6.bar(x - w/2, e_o, w, label="Energy out",  color='steelblue')
    ax6.bar(x + w/2, e_i, w, label="Energy in",   color='tomato')
    ax6.bar(x, net,          0.15, label="Net", color='green', alpha=0.8)
    ax6.set_xticks(x)
    ax6.set_xticklabels(labels, fontsize=8)
    ax6.set_ylabel("Energy [J]")
    ax6.set_title("Pumping Cycle Energy")
    ax6.legend(fontsize=8)
    ax6.grid(True, alpha=0.3, axis='y')

    out_path = Path(__file__).parent.parent / "logs" / "rotor_comparison.png"
    fig.savefig(str(out_path), dpi=130, bbox_inches="tight")
    print(f"Plot saved: {out_path}")
    plt.close(fig)


# ── key parameter diff table ─────────────────────────────────────────────────

def _print_param_table(rotor_b, rotor_d) -> None:
    p_b = rotor_b.aero_kwargs()
    p_d = rotor_d.aero_kwargs()

    print("── Rotor parameter comparison ──")
    rows = [
        ("n_blades",       "–",      "n_blades"),
        ("radius",         "m",      "r_tip"),
        ("root_cutout",    "m",      "r_root"),
        ("chord",          "m",      "chord"),
        ("blade_area S_w", "m²",     None),    # computed below
        ("disk_area A",    "m²",     None),
        ("solidity σ",     "–",      None),
        ("CL0",            "–",      "CL0"),
        ("CL_alpha",       "/rad",   "CL_alpha"),
        ("CD0",            "–",      "CD0"),
        ("Oswald e",       "–",      "oswald_eff"),
        ("AR",             "–",      "aspect_ratio"),
        ("alpha_stall",    "°",      None),
        ("mass",           "kg",     None),
    ]

    span_b = p_b["r_tip"] - p_b["r_root"]
    span_d = p_d["r_tip"] - p_d["r_root"]
    Sw_b   = p_b["n_blades"] * p_b["chord"] * span_b
    Sw_d   = p_d["n_blades"] * p_d["chord"] * span_d
    A_b    = math.pi * (p_b["r_tip"]**2 - p_b["r_root"]**2)
    A_d    = math.pi * (p_d["r_tip"]**2 - p_d["r_root"]**2)
    sig_b  = p_b["n_blades"] * p_b["chord"] / (math.pi * p_b["r_tip"])
    sig_d  = p_d["n_blades"] * p_d["chord"] / (math.pi * p_d["r_tip"])
    stall_b= math.degrees(p_b["aoa_limit"])
    stall_d= math.degrees(p_d["aoa_limit"])
    mass_b = rotor_b.mass_kg
    mass_d = rotor_d.mass_kg

    extras = {
        "blade_area S_w": (Sw_b, Sw_d),
        "disk_area A":    (A_b,  A_d),
        "solidity σ":     (sig_b,sig_d),
        "alpha_stall":    (stall_b, stall_d),
        "mass":           (mass_b, mass_d),
    }

    print(f"  {'Parameter':<20} {'Unit':<5}  {'beaupoil_2026':>14}  {'de_schutter_2018':>16}  {'ratio B/DS':>10}")
    print("  " + "─" * 74)
    for name, unit, key in rows:
        if key is not None:
            vb = float(p_b[key]); vd = float(p_d[key])
        else:
            vb, vd = extras[name]
        ratio = vb / vd if abs(vd) > 1e-9 else float("inf")
        print(f"  {name:<20} {unit:<5}  {vb:>14.4g}  {vd:>16.4g}  {ratio:>10.3f}")
    print()

    print("  Key derived quantities for thrust comparison:")
    print(f"    CL_alpha × S_w:  B={p_b['CL_alpha']*Sw_b:.3f}  DS={p_d['CL_alpha']*Sw_d:.3f}"
          f"  ratio={p_b['CL_alpha']*Sw_b/(p_d['CL_alpha']*Sw_d):.1f}×")
    print(f"    T/W at design pt (T≈200N):  B={200/mass_b:.1f}  DS={200/mass_d:.2f}")
    print()


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plot", action="store_true", help="Generate comparison PNG")
    parser.add_argument("--no-pumping", action="store_true",
                        help="Skip pumping cycle (fast static-sweep only)")
    args = parser.parse_args()

    rotor_b = rd.load("beaupoil_2026")
    rotor_d = rd.load("de_schutter_2018")

    _print_param_table(rotor_b, rotor_d)

    print("Running static sweep (lumped BEM)...")
    rows_b = _static_sweep(rotor_b, "beaupoil_2026")
    rows_d = _static_sweep(rotor_d, "de_schutter_2018")
    _print_sweep(rows_b, rows_d)

    print("Running static sweep (Glauert strip-theory)...")
    rows_strip_b = _strip_sweep(rotor_b, "beaupoil_2026")
    rows_strip_d = _strip_sweep(rotor_d, "de_schutter_2018")
    _print_three_way(rows_b, rows_d, rows_strip_b, rows_strip_d)

    _print_edgewise_analysis(rows_b, rows_d)

    pump_b = pump_d = None
    if not args.no_pumping:
        print("Running pumping cycle — beaupoil_2026  (~60 s sim)...")
        pump_b = _run_pumping_cycle(rotor_b, "beaupoil_2026")
        print("Running pumping cycle — de_schutter_2018  (~60 s sim)...")
        pump_d = _run_pumping_cycle(rotor_d, "de_schutter_2018")
        _print_pumping([pump_b, pump_d])

    if args.plot and pump_b and pump_d:
        _plot(rows_b, rows_d, pump_b, pump_d)
    elif args.plot:
        # static-only plot
        _plot(rows_b, rows_d,
              {"mean_T_out_N": 0, "mean_T_in_N": 0, "max_T_out_N": 0,
               "energy_out_J": 0, "energy_in_J": 0, "net_energy_J": 0},
              {"mean_T_out_N": 0, "mean_T_in_N": 0, "max_T_out_N": 0,
               "energy_out_J": 0, "energy_in_J": 0, "net_energy_J": 0})


if __name__ == "__main__":
    main()
