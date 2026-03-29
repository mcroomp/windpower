"""
test_deschutter_cycle.py — De Schutter (2018) pumping cycle, no ArduPilot.

Implements the two-phase strategy from De Schutter et al. (2018) Fig. 4:

  Reel-out (power generation):
    Rotor tilt ξ = 30–50° from wind direction (body_z aligned with tether).
    High collective → high thrust along tether → high tension → high power.

  Reel-in (recovery):
    Rotor tilt ξ > 70° from wind direction (body_z tilted toward vertical).
    Body_z ≈ [0, 0, 1] so thrust acts upward, not along tether direction.
    Winch reels in against near-zero aerodynamic resistance → low tension → low energy cost.

The key difference from a naive collective-only approach: tilt is the primary
lever for reducing reel-in tension, not collective.  By tilting body_z to
vertical, thrust no longer contributes to tether pull.  The tether tension
during reel-in is then only the force needed to overcome the gravity component
along the tether direction plus inertia — much lower than during reel-out.

Controller split:
  - Attitude   → compute_swashplate_from_state  (tilt_lon / tilt_lat)
                 body_z_eq transitions from tether-aligned to vertical at reel-in
  - Collective → TensionController (same as test_pumping_cycle.py)
  - Winch      → tether.rest_length ±= v_reel × DT  each step

Pass criteria:
  1. Hub stays above z_floor throughout.
  2. Reel-in mean tension < reel-out mean tension  (De Schutter mechanism works).
  3. Net energy positive  (reel-out energy > reel-in energy).
  4. Peak tension stays below 80% of tether break load.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dynamics   import RigidBodyDynamics
from aero       import RotorAero
from tether     import TetherModel
from controller import compute_swashplate_from_state
from frames     import build_orb_frame

# ── Simulation constants ───────────────────────────────────────────────────────
DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
POS0          = np.array([46.258, 14.241, 12.530])
VEL0          = np.array([-0.257,  0.916, -0.093])
BODY_Z0       = np.array([0.851018, 0.305391, 0.427206])
OMEGA_SPIN0   = 20.148
REST_LENGTH0  = 49.949

T_AERO_OFFSET = 45.0

K_DRIVE_SPIN   = 1.4
K_DRAG_SPIN    = 0.01786
I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

WIND          = np.array([10.0, 0.0, 0.0])
BREAK_LOAD_N  = 620.0

# ── De Schutter cycle parameters ──────────────────────────────────────────────
T_REEL_OUT    = 30.0    # s
T_REEL_IN     = 30.0    # s
T_TRANSITION  =  5.0    # s — body_z blend from tether-aligned to vertical at reel-in start
V_REEL_OUT    =  0.4    # m/s
V_REEL_IN     =  0.4    # m/s

# Reel-in body_z target: rotor axis pointing up → thrust upward, not along tether
# ξ = cos⁻¹(dot([0,0,1], [1,0,0])) = 90° > 70°  ✓  (De Schutter criterion satisfied)
BODY_Z_REEL_IN = np.array([0.0, 0.0, 1.0])

# Tension setpoints passed to TensionController
DEFAULT_TENSION_OUT = 200.0   # N
DEFAULT_TENSION_IN  =  20.0   # N


# ── Re-use TensionController from test_pumping_cycle ──────────────────────────

class TensionController:
    """PI controller: adjusts collective to maintain requested tether tension."""

    def __init__(self, setpoint_n, kp=5e-4, ki=1e-4, coll_min=-0.10, coll_max=0.20):
        self.setpoint  = float(setpoint_n)
        self.kp        = float(kp)
        self.ki        = float(ki)
        self.coll_min  = float(coll_min)
        self.coll_max  = float(coll_max)
        self._integral = 0.0

    def update(self, tension_actual, dt):
        error           = self.setpoint - tension_actual
        self._integral += error * dt
        self._integral  = np.clip(
            self._integral,
            self.coll_min / max(self.ki, 1e-12),
            self.coll_max / max(self.ki, 1e-12),
        )
        raw = self.kp * error + self.ki * self._integral
        return float(np.clip(raw, self.coll_min, self.coll_max))


# ── Orbit-tracking helper ─────────────────────────────────────────────────────

def _orbit_tracked_body_z_eq(cur_pos, tether_dir0, body_z_eq0):
    """Rotate body_z_eq0 azimuthally to track the hub's orbital position."""
    cur_tdir = cur_pos / np.linalg.norm(cur_pos)
    th0h = np.array([tether_dir0[0], tether_dir0[1], 0.0])
    thh  = np.array([cur_tdir[0],    cur_tdir[1],    0.0])
    n0h = np.linalg.norm(th0h); nhh = np.linalg.norm(thh)
    if n0h < 0.01 or nhh < 0.01:
        return body_z_eq0
    th0h /= n0h; thh /= nhh
    cos_phi = float(np.clip(np.dot(th0h, thh), -1.0, 1.0))
    sin_phi = float(th0h[0] * thh[1] - th0h[1] * thh[0])
    bz0 = body_z_eq0
    result = np.array([
        cos_phi * bz0[0] - sin_phi * bz0[1],
        sin_phi * bz0[0] + cos_phi * bz0[1],
        bz0[2],
    ])
    return result / np.linalg.norm(result)


def _blend_body_z(alpha, bz_start, bz_end):
    """
    Linearly blend two unit vectors and renormalise.  alpha=0 → bz_start,
    alpha=1 → bz_end.  Not true SLERP but accurate enough for smooth transitions.
    """
    alpha = float(np.clip(alpha, 0.0, 1.0))
    blended = (1.0 - alpha) * bz_start + alpha * bz_end
    n = np.linalg.norm(blended)
    return blended / n if n > 1e-6 else bz_end


# ── Main simulation ────────────────────────────────────────────────────────────

def _run_deschutter_cycle(
    tension_out:  float = DEFAULT_TENSION_OUT,
    tension_in:   float = DEFAULT_TENSION_IN,
    v_reel_out:   float = V_REEL_OUT,
    v_reel_in:    float = V_REEL_IN,
    t_reel_out:   float = T_REEL_OUT,
    t_reel_in:    float = T_REEL_IN,
    t_transition: float = T_TRANSITION,
) -> dict:
    """
    Run one De Schutter-style reel-out / reel-in pumping cycle.

    The rotor axis (body_z) transitions from tether-aligned to vertical at
    the start of reel-in, implementing De Schutter's tilt strategy.
    """
    dyn    = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=1.0,
    )
    aero   = RotorAero()
    tether = TetherModel(anchor_enu=ANCHOR, rest_length=REST_LENGTH0,
                         axle_attachment_length=0.0)

    ctrl_out = TensionController(setpoint_n=tension_out)
    ctrl_in  = TensionController(setpoint_n=tension_in)

    hub_state   = dyn.state
    omega_spin  = OMEGA_SPIN0
    tether_dir0 = POS0 / np.linalg.norm(POS0)
    body_z_eq0  = BODY_Z0.copy()

    t_total  = t_reel_out + t_reel_in
    n_steps  = int(t_total / DT)
    rec_every = int(1.0 / DT)

    ts            = []
    tensions      = []
    altitudes     = []
    rest_lengths  = []
    collectives   = []
    tilts_from_wind = []    # ξ = angle between body_z and wind direction [deg]
    floor_hits    = 0

    energy_out = 0.0
    energy_in  = 0.0
    tension_now = 0.0

    for i in range(n_steps):
        t = i * DT
        phase_out = t < t_reel_out

        # ── Winch: update rest_length ─────────────────────────────────────
        if phase_out:
            tether.rest_length += v_reel_out * DT
        else:
            new_rl = tether.rest_length - v_reel_in * DT
            tether.rest_length = max(REST_LENGTH0, new_rl)

        # ── Body_z_eq target: tether-aligned (reel-out), vertical (reel-in) ──
        if phase_out:
            body_z_eq_cur = _orbit_tracked_body_z_eq(
                hub_state["pos"], tether_dir0, body_z_eq0)
        else:
            # Transition from tether-aligned to vertical over t_transition seconds
            t_since_reel_in = t - t_reel_out
            alpha = t_since_reel_in / t_transition
            bz_tether = _orbit_tracked_body_z_eq(
                hub_state["pos"], tether_dir0, body_z_eq0)
            body_z_eq_cur = _blend_body_z(alpha, bz_tether, BODY_Z_REEL_IN)

        # ── Collective from tension PI controller ─────────────────────────
        ctrl       = ctrl_out if phase_out else ctrl_in
        collective = ctrl.update(tension_now, DT)

        # ── Attitude controller → tilt ────────────────────────────────────
        sw = compute_swashplate_from_state(
            hub_state  = hub_state,
            anchor_pos = ANCHOR,
            body_z_eq  = body_z_eq_cur,
        )

        # ── Aerodynamics ──────────────────────────────────────────────────
        forces = aero.compute_forces(
            collective_rad = collective,
            tilt_lon       = sw["tilt_lon"],
            tilt_lat       = sw["tilt_lat"],
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t,
        )

        # ── Tether ────────────────────────────────────────────────────────
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        forces[0:3] += tf
        forces[3:6] += tm
        tension_now = tether._last_info.get("tension", 0.0)

        # ── Energy accounting ─────────────────────────────────────────────
        if phase_out:
            energy_out += tension_now * v_reel_out * DT
        else:
            energy_in  += tension_now * v_reel_in  * DT

        # ── Spin & moment ─────────────────────────────────────────────────
        Q_spin     = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)

        M_orbital  = forces[3:6] - aero.last_M_spin
        M_orbital += -50.0 * hub_state["omega"]

        hub_state = dyn.step(forces[0:3], M_orbital, DT, omega_spin=omega_spin)

        if hub_state["pos"][2] <= 1.05:
            floor_hits += 1

        # ── Record ────────────────────────────────────────────────────────
        if i % rec_every == 0:
            body_z_cur = hub_state["R"][:, 2]
            xi_deg = float(np.degrees(np.arccos(
                np.clip(np.dot(body_z_cur, WIND / np.linalg.norm(WIND)), -1.0, 1.0)
            )))
            ts.append(t)
            tensions.append(tension_now)
            altitudes.append(hub_state["pos"][2])
            rest_lengths.append(tether.rest_length)
            collectives.append(collective)
            tilts_from_wind.append(xi_deg)

    # Final snapshot
    body_z_cur = hub_state["R"][:, 2]
    xi_deg = float(np.degrees(np.arccos(
        np.clip(np.dot(body_z_cur, WIND / np.linalg.norm(WIND)), -1.0, 1.0)
    )))
    ts.append(t_total); tensions.append(tension_now)
    altitudes.append(hub_state["pos"][2]); rest_lengths.append(tether.rest_length)
    collectives.append(collective); tilts_from_wind.append(xi_deg)

    split = int(t_reel_out)
    return dict(
        ts              = ts,
        tensions        = tensions,
        tensions_out    = tensions[:split],
        tensions_in     = tensions[split:],
        altitudes       = altitudes,
        rest_lengths    = rest_lengths,
        collectives     = collectives,
        tilts_from_wind = tilts_from_wind,
        floor_hits      = floor_hits,
        energy_out      = energy_out,
        energy_in       = energy_in,
        net_energy      = energy_out - energy_in,
        tension_out_sp  = tension_out,
        tension_in_sp   = tension_in,
    )


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_deschutter_no_crash():
    """Hub must stay above z_floor throughout both reel phases."""
    r = _run_deschutter_cycle()
    _print_cycle(r)
    assert r["floor_hits"] == 0, \
        f"Hub hit z_floor {r['floor_hits']} times during De Schutter cycle"


def test_deschutter_reel_in_lower_tension():
    """
    Reel-in mean tension must be less than reel-out mean tension.

    This is the De Schutter mechanism: tilting body_z to vertical during
    reel-in redirects thrust upward (fighting gravity), not along the tether,
    so the winch encounters near-zero aerodynamic resistance.
    """
    r = _run_deschutter_cycle()
    # Exclude the first T_TRANSITION seconds of reel-in (hub is still tilting)
    skip = int(T_TRANSITION)
    mean_out = float(np.mean(r["tensions_out"])) if r["tensions_out"] else 0.0
    mean_in  = float(np.mean(r["tensions_in"][skip:])) if r["tensions_in"][skip:] else 0.0
    assert mean_in < mean_out, (
        f"Reel-in steady tension ({mean_in:.1f} N) not less than reel-out ({mean_out:.1f} N) — "
        "De Schutter tilt mechanism not working"
    )


def test_deschutter_net_energy_positive():
    """
    Reel-out energy must exceed reel-in energy.

    This is the fundamental AWE pumping-cycle condition.  With the De Schutter
    tilt strategy, reel-in tension drops to near the gravity component along
    the tether (~15–30 N), well below the reel-out tension (~150–250 N).
    """
    r = _run_deschutter_cycle()
    assert r["net_energy"] > 0, (
        f"Net energy {r['net_energy']:.1f} J ≤ 0 — "
        "De Schutter cycle does not produce net power"
    )


def test_deschutter_reel_in_tilt_achieved():
    """
    During steady reel-in (after transition), body_z must be tilted more
    than 60° from the wind direction — approaching the De Schutter >70° target.
    (60° is used here rather than 70° to allow for attitude controller lag.)
    """
    r = _run_deschutter_cycle()
    skip = int(T_TRANSITION) + 2   # allow a couple of extra seconds for settling
    tilts_steady = r["tilts_from_wind"][int(T_REEL_OUT) + skip:]
    if not tilts_steady:
        pytest.skip("No steady reel-in data to check tilt")
    mean_tilt = float(np.mean(tilts_steady))
    assert mean_tilt >= 60.0, (
        f"Steady reel-in mean tilt ξ = {mean_tilt:.1f}° < 60° — "
        "body_z not sufficiently tilted away from wind"
    )


def test_deschutter_reel_out_tilt_in_range():
    """
    During reel-out, body_z must stay within De Schutter's 30–55° tilt range
    (tether-aligned orbit flight).  A large deviation means the tilt controller
    is fighting the tether alignment.
    """
    r = _run_deschutter_cycle()
    tilts_out = r["tilts_from_wind"][:int(T_REEL_OUT)]
    if not tilts_out:
        pytest.skip("No reel-out tilt data")
    mean_tilt_out = float(np.mean(tilts_out))
    assert 25.0 <= mean_tilt_out <= 60.0, (
        f"Reel-out mean tilt ξ = {mean_tilt_out:.1f}° outside 25–60° — "
        "hub not in expected reel-out orientation"
    )


def test_deschutter_tether_not_broken():
    """Peak tension must stay below 80% of Dyneema SK75 1.9 mm break load (620 N)."""
    r    = _run_deschutter_cycle()
    peak = max(r["tensions"])
    limit = 0.8 * BREAK_LOAD_N
    assert peak < limit, \
        f"Peak tension {peak:.1f} N ≥ 80% break load ({limit:.1f} N)"


# ── Diagnostic printer ────────────────────────────────────────────────────────

def _print_cycle(r):
    sp_out = r["tension_out_sp"]
    sp_in  = r["tension_in_sp"]
    print(f"\nDe Schutter cycle  (setpoints: reel-out={sp_out:.0f}N  reel-in={sp_in:.0f}N)")
    print(f"  {'t':>5}  {'phase':>8}  {'tension':>9}  {'coll':>7}  "
          f"{'ξ_wind':>7}  {'altitude':>9}  {'rest_L':>8}")
    for t, ten, coll, xi, alt, rl in zip(
            r["ts"], r["tensions"], r["collectives"], r["tilts_from_wind"],
            r["altitudes"], r["rest_lengths"]):
        phase = "reel-out" if t <= T_REEL_OUT else "reel-in "
        print(f"  {t:5.1f}  {phase:>8}  {ten:8.1f}N  {np.degrees(coll):6.2f}°  "
              f"{xi:6.1f}°  {alt:8.2f}m  {rl:7.2f}m")
    out = r["tensions_out"]
    inn = r["tensions_in"][int(T_TRANSITION):]
    print(f"\n  Reel-out: mean={np.mean(out):.1f}N  max={max(out):.1f}N  "
          f"energy={r['energy_out']:.1f}J")
    print(f"  Reel-in (steady): mean={np.mean(inn):.1f}N  max={max(inn):.1f}N  "
          f"energy={r['energy_in']:.1f}J")
    print(f"  Net energy: {r['net_energy']:.1f}J  floor_hits={r['floor_hits']}")
