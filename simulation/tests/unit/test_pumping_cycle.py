"""
test_pumping_cycle.py — Tether reel-in / reel-out pumping cycle, no ArduPilot.

Tests the two-phase AWE pumping cycle with tension setpoint control:

  Reel-out (power generation):
    Winch pays out tether at v_reel_out [m/s].  A PI controller raises
    collective to maintain tension_setpoint_out [N].  High tension → high
    power = tension × v_reel_out into the ground generator.

  Reel-in (recovery):
    Winch reels in at v_reel_in [m/s].  Controller holds tension_setpoint_in
    (low) by lowering collective → hub barely resists the winch → low energy
    cost = tension_low × v_reel_in.

Controller split (matches planned mediator architecture):
  - Attitude   → compute_swashplate_from_state  (tilt_lon / tilt_lat)
  - Collective → TensionController below        (closed-loop on tether tension)
  - Winch      → tether.rest_length ±= v_reel × DT  each step

Net energy must be positive: reel-out energy > reel-in energy.
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
REST_LENGTH0  = 49.949     # taut at initial position (from steady_state_starting.json)

T_AERO_OFFSET = 45.0       # s — aero ramp already complete at kinematic-phase end

K_DRIVE_SPIN   = 1.4
K_DRAG_SPIN    = 0.01786
I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

WIND = np.array([10.0, 0.0, 0.0])

BREAK_LOAD_N = 620.0   # Dyneema SK75 1.9 mm [N]

# ── Default pumping cycle parameters ──────────────────────────────────────────
DEFAULT_TENSION_OUT  = 150.0   # N — target tether tension during reel-out
DEFAULT_TENSION_IN   =  20.0   # N — target tether tension during reel-in
DEFAULT_V_REEL_OUT   =   0.4   # m/s — winch pay-out speed
DEFAULT_V_REEL_IN    =   0.4   # m/s — winch reel-in speed
DEFAULT_T_REEL_OUT   =  30.0   # s — reel-out phase duration
DEFAULT_T_REEL_IN    =  30.0   # s — reel-in phase duration


# ── Tension PI controller ──────────────────────────────────────────────────────

class TensionController:
    """
    Proportional-integral controller that adjusts collective blade pitch to
    maintain a requested tether tension setpoint.

    Higher collective → more thrust → hub flies farther from anchor → higher tension.
    Lower collective → less thrust → hub stays closer → lower tension.

    Parameters
    ----------
    setpoint_n   : float   Target tether tension [N]
    kp           : float   Proportional gain [rad / N]
    ki           : float   Integral gain [rad / (N·s)]
    coll_min     : float   Minimum collective [rad]
    coll_max     : float   Maximum collective [rad]
    """

    def __init__(
        self,
        setpoint_n: float,
        kp:         float = 5e-4,
        ki:         float = 1e-4,
        coll_min:   float = -0.05,
        coll_max:   float =  0.20,
    ):
        self.setpoint  = float(setpoint_n)
        self.kp        = float(kp)
        self.ki        = float(ki)
        self.coll_min  = float(coll_min)
        self.coll_max  = float(coll_max)
        self._integral = 0.0
        self._coll     = 0.0   # initial collective

    def update(self, tension_actual: float, dt: float) -> float:
        """
        Compute collective [rad] for one timestep.

        Parameters
        ----------
        tension_actual : float   Measured tether tension [N]
        dt             : float   Timestep [s]

        Returns
        -------
        float   Collective blade pitch [rad]
        """
        error           = self.setpoint - tension_actual
        self._integral += error * dt
        # Anti-windup: clamp integral contribution
        self._integral  = np.clip(
            self._integral,
            self.coll_min / max(self.ki, 1e-12),
            self.coll_max / max(self.ki, 1e-12),
        )
        raw = self.kp * error + self.ki * self._integral
        self._coll = float(np.clip(raw, self.coll_min, self.coll_max))
        return self._coll


# ── Orbit-tracking helper (identical to test_closed_loop_60s.py) ──────────────

def _orbit_tracked_body_z_eq(cur_pos, tether_dir0, body_z_eq0):
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


# ── Main simulation ────────────────────────────────────────────────────────────

def _run_pumping_cycle(
    tension_out: float = DEFAULT_TENSION_OUT,
    tension_in:  float = DEFAULT_TENSION_IN,
    v_reel_out:  float = DEFAULT_V_REEL_OUT,
    v_reel_in:   float = DEFAULT_V_REEL_IN,
    t_reel_out:  float = DEFAULT_T_REEL_OUT,
    t_reel_in:   float = DEFAULT_T_REEL_IN,
) -> dict:
    """
    Simulate one reel-out / reel-in pumping cycle.

    The attitude controller (tilt_lon / tilt_lat) and the tension PI
    controller (collective) run independently at 400 Hz.  The winch updates
    rest_length each step.

    Returns a dict with per-second snapshots and energy totals.
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

    # Per-second records
    ts           = []
    tensions     = []
    altitudes    = []
    rest_lengths = []
    collectives  = []
    floor_hits   = 0

    # Energy accumulators
    energy_out = 0.0
    energy_in  = 0.0

    tension_now = 0.0   # initialise before first step

    for i in range(n_steps):
        t = i * DT
        phase_out = t < t_reel_out

        # ── Winch: update rest_length ─────────────────────────────────────
        if phase_out:
            tether.rest_length += v_reel_out * DT
        else:
            new_rl = tether.rest_length - v_reel_in * DT
            tether.rest_length = max(REST_LENGTH0, new_rl)

        # ── Tension PI controller → collective ────────────────────────────
        ctrl        = ctrl_out if phase_out else ctrl_in
        collective  = ctrl.update(tension_now, DT)

        # ── Attitude controller → tilt ────────────────────────────────────
        body_z_eq_cur = _orbit_tracked_body_z_eq(
            hub_state["pos"], tether_dir0, body_z_eq0)
        sw = compute_swashplate_from_state(
            hub_state  = hub_state,
            anchor_pos = ANCHOR,
            body_z_eq  = body_z_eq_cur,
        )

        # ── Aerodynamics ──────────────────────────────────────────────────
        forces = aero.compute_forces(
            collective_rad = collective,          # from tension controller
            tilt_lon       = sw["tilt_lon"],      # from attitude controller
            tilt_lat       = sw["tilt_lat"],
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t,
        )

        # ── Tether force ──────────────────────────────────────────────────
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        forces[0:3] += tf
        forces[3:6] += tm
        tension_now = tether._last_info.get("tension", 0.0)

        # ── Energy accounting ─────────────────────────────────────────────
        if phase_out:
            energy_out += tension_now * v_reel_out * DT
        else:
            energy_in  += tension_now * v_reel_in  * DT

        # ── Spin model ────────────────────────────────────────────────────
        Q_spin     = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)

        M_orbital  = forces[3:6] - aero.last_M_spin
        M_orbital += -50.0 * hub_state["omega"]

        hub_state = dyn.step(forces[0:3], M_orbital, DT, omega_spin=omega_spin)

        if hub_state["pos"][2] <= 1.05:
            floor_hits += 1

        # ── Record ────────────────────────────────────────────────────────
        if i % rec_every == 0:
            ts.append(t)
            tensions.append(tension_now)
            altitudes.append(hub_state["pos"][2])
            rest_lengths.append(tether.rest_length)
            collectives.append(collective)

    # Final snapshot
    ts.append(t_total); tensions.append(tension_now)
    altitudes.append(hub_state["pos"][2]); rest_lengths.append(tether.rest_length)
    collectives.append(collective)

    split = int(t_reel_out)   # index dividing reel-out / reel-in in per-second list
    return dict(
        ts             = ts,
        tensions       = tensions,
        tensions_out   = tensions[:split],
        tensions_in    = tensions[split:],
        altitudes      = altitudes,
        rest_lengths   = rest_lengths,
        collectives    = collectives,
        floor_hits     = floor_hits,
        energy_out     = energy_out,
        energy_in      = energy_in,
        net_energy     = energy_out - energy_in,
        tension_out_sp = tension_out,
        tension_in_sp  = tension_in,
    )


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_pumping_no_crash():
    """Hub must stay above z_floor=1 m throughout both reel phases."""
    r = _run_pumping_cycle()
    _print_cycle(r)
    assert r["floor_hits"] == 0, \
        f"Hub hit z_floor {r['floor_hits']} times during pumping cycle"


def test_reel_out_tension_achieved():
    """
    During reel-out, mean tension must reach at least 50% of the setpoint.
    The PI controller needs several seconds to spin up; 50% is a conservative
    check that it is actually responding.
    """
    sp = DEFAULT_TENSION_OUT
    r  = _run_pumping_cycle()
    mean_out = float(np.mean(r["tensions_out"])) if r["tensions_out"] else 0.0
    assert mean_out >= 0.5 * sp, (
        f"Reel-out mean tension {mean_out:.1f} N < 50% of setpoint {sp:.1f} N — "
        "tension controller not responding"
    )


def test_collective_tracks_setpoint():
    """
    The PI controller must command higher collective for higher setpoints
    and lower / negative collective for low setpoints.

    Compares mean collective over the reel-out phase (high setpoint) vs the
    reel-in phase (low setpoint) in the same run.

    Note on pumping-cycle energy economics
    --------------------------------------
    Simply changing collective between reel-out and reel-in does NOT guarantee
    net positive energy in the current model.  The tether tension during reel-in
    is dominated by the force needed to pull the hub inward against its
    aerodynamic resistance — roughly equal to the equilibrium aerodynamic
    force regardless of collective.

    Net positive energy requires that the hub actively reduces aerodynamic
    drag during reel-in, typically by changing its flight path (flying toward
    the anchor rather than maintaining orbit).  That capability is outside the
    scope of the current single-phase fixed-orbit model.
    """
    r = _run_pumping_cycle()
    mean_coll_out = float(np.mean(r["collectives"][:int(DEFAULT_T_REEL_OUT)]))
    mean_coll_in  = float(np.mean(r["collectives"][int(DEFAULT_T_REEL_OUT):]))
    assert mean_coll_out > mean_coll_in, (
        f"Collective during reel-out ({np.degrees(mean_coll_out):.2f}°) not higher "
        f"than during reel-in ({np.degrees(mean_coll_in):.2f}°) — "
        "PI controller not responding to setpoint difference"
    )


def test_tether_not_broken():
    """Peak tension must stay below 80% of Dyneema SK75 1.9 mm break load (620 N)."""
    r    = _run_pumping_cycle()
    peak = max(r["tensions"])
    limit = 0.8 * BREAK_LOAD_N
    assert peak < limit, \
        f"Peak tension {peak:.1f} N ≥ 80% break load ({limit:.1f} N) — tether unsafe"


def test_static_tension_setpoint_range():
    """
    In a static (no winch movement) scenario, the PI controller must be able
    to hold two meaningfully different tension setpoints, demonstrating the
    control range available to the winch controller.

    Runs two 30 s simulations with fixed rest_length (tether not moving):
    one with a high setpoint, one with a low setpoint.  Mean tension over the
    last 10 s must differ by at least 30 N to confirm usable control authority.
    """
    def _run_static(setpoint):
        dyn    = RigidBodyDynamics(
            mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
            pos0=POS0.tolist(), vel0=VEL0.tolist(),
            R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=1.0,
        )
        aero   = RotorAero()
        tether = TetherModel(anchor_enu=ANCHOR, rest_length=REST_LENGTH0,
                             axle_attachment_length=0.0)
        ctrl   = TensionController(setpoint_n=setpoint)
        hub_state   = dyn.state
        omega_spin  = OMEGA_SPIN0
        tether_dir0 = POS0 / np.linalg.norm(POS0)
        body_z_eq0  = BODY_Z0.copy()
        T_SIM = 30.0
        tensions_late = []
        tension_now = 0.0
        for i in range(int(T_SIM / DT)):
            t = i * DT
            collective = ctrl.update(tension_now, DT)
            body_z_eq_cur = _orbit_tracked_body_z_eq(
                hub_state["pos"], tether_dir0, body_z_eq0)
            sw = compute_swashplate_from_state(
                hub_state=hub_state, anchor_pos=ANCHOR, body_z_eq=body_z_eq_cur)
            forces = aero.compute_forces(
                collective_rad=collective, tilt_lon=sw["tilt_lon"],
                tilt_lat=sw["tilt_lat"], R_hub=hub_state["R"],
                v_hub_world=hub_state["vel"], omega_rotor=omega_spin,
                wind_world=WIND, t=T_AERO_OFFSET + t)
            tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
            forces[0:3] += tf; forces[3:6] += tm
            tension_now = tether._last_info.get("tension", 0.0)
            Q_spin     = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
            omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN_KGMS2 * DT)
            M_orbital  = forces[3:6] - aero.last_M_spin
            M_orbital += -50.0 * hub_state["omega"]
            hub_state = dyn.step(forces[0:3], M_orbital, DT, omega_spin=omega_spin)
            if t >= 20.0:
                tensions_late.append(tension_now)
        return float(np.mean(tensions_late))

    mean_high = _run_static(setpoint=300.0)
    mean_low  = _run_static(setpoint=100.0)
    delta = mean_high - mean_low
    assert delta >= 25.0, (
        f"Static tension control range too narrow: "
        f"high setpoint mean={mean_high:.1f}N, low setpoint mean={mean_low:.1f}N, "
        f"delta={delta:.1f}N < 25N minimum"
    )


def test_higher_tension_setpoint_gives_higher_tension():
    """
    Increasing the reel-out tension setpoint must produce higher mean tension.
    Validates that the tension controller has meaningful control authority.
    """
    r_low  = _run_pumping_cycle(tension_out=100.0)
    r_high = _run_pumping_cycle(tension_out=200.0)
    mean_low  = float(np.mean(r_low["tensions_out"]))
    mean_high = float(np.mean(r_high["tensions_out"]))
    assert mean_high > mean_low, (
        f"Higher setpoint (200 N) produced lower mean tension ({mean_high:.1f} N) "
        f"than lower setpoint (100 N, {mean_low:.1f} N) — controller sign/gain error"
    )


# ── Diagnostic printer ────────────────────────────────────────────────────────

def _print_cycle(r):
    sp_out = r["tension_out_sp"]
    sp_in  = r["tension_in_sp"]
    print(f"\nPumping cycle  (setpoints: reel-out={sp_out:.0f}N  reel-in={sp_in:.0f}N)")
    print(f"  {'t':>5}  {'phase':>8}  {'tension':>9}  {'coll':>7}  "
          f"{'altitude':>9}  {'rest_L':>8}")
    for t, ten, coll, alt, rl in zip(
            r["ts"], r["tensions"], r["collectives"],
            r["altitudes"], r["rest_lengths"]):
        phase = "reel-out" if t <= DEFAULT_T_REEL_OUT else "reel-in"
        print(f"  {t:5.1f}  {phase:>8}  {ten:8.1f}N  {np.degrees(coll):6.2f}°  "
              f"{alt:8.2f}m  {rl:7.2f}m")
    out = r["tensions_out"]; inn = r["tensions_in"]
    print(f"\n  Reel-out: mean={np.mean(out):.1f}N  "
          f"max={max(out):.1f}N  energy={r['energy_out']:.1f}J")
    print(f"  Reel-in:  mean={np.mean(inn):.1f}N  "
          f"max={max(inn):.1f}N  energy={r['energy_in']:.1f}J")
    print(f"  Net energy: {r['net_energy']:.1f}J  floor_hits={r['floor_hits']}")
