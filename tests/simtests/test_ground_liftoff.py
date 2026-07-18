"""
test_ground_liftoff.py -- rawes.lua GUIDED mode lifts off from near-ground altitude.

Physical scenario
-----------------
  - Hub starts at altitude 1 m (NED pos = [0, 0, -1]).
  - Anchor at the mock EKF origin, i.e. NED offset [0, 0, 0] (see rawes_modes.send_anchor_ned).
  - Rotor disk is horizontal: body_z = [0, 0, +1] (NED; FRD body_z points toward anchor).
  - Wind: constant 10 m/s upward, i.e. wind_ned = [0, 0, -10] m/s.
  - Constant 200 N downward load modelling a pre-tensioned tether.
    Implemented as a mock tether that always returns F = [0, 0, +200] N (NED down).
  - rawes.lua mode=1 (STEADY), GUIDED, RAWES_ALT = 5.0 → commands a climb.
  - Floor at altitude 0 m: hub cannot go underground.

Wind blows upward through the horizontal disk, driving autorotation and
generating upward thrust.  The Lua VZ-PI collective controller commands a
climb toward 5 m.  Liftoff criterion: hub altitude exceeds 2 m within 60 s.

Coordinate frame: NED -- X=North, Y=East, Z=Down.  Altitude = -pos_z.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

from tests.simtests.simtest_runner import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from simulation.rawes_lua_harness import RawesLua
from groundstation.gcs import NamedValueFloat
from groundstation.rawes_modes import MODE_STEADY, send_anchor_ned
from tests.simtests._rotor_helpers import load_default_rotor

# ── Physical constants ────────────────────────────────────────────────────────
_ROTOR = load_default_rotor()

DT              = 2.5e-3          # 400 Hz
LUA_PERIOD      = 0.020           # 50 Hz Lua tick
LUA_EVERY       = round(LUA_PERIOD / DT)
WIND_NED        = np.array([0.0, 0.0, -10.0])   # 10 m/s upward (NED Z down)
TETHER_FORCE_N  = 200.0           # constant downward load [N]
TARGET_ALT_M    = 5.0             # RAWES_ALT command sent each tick
LIFTOFF_ALT_M   = 2.0             # altitude threshold that counts as liftoff [m]
LIFTOFF_TIMEOUT = 60.0            # must lift off within this many sim-seconds [s]

# ── Constant-downforce tether mock ────────────────────────────────────────────

class _ConstantDownforce:
    """
    Drop-in replacement for TetherModel that returns a constant downward force.
    Satisfies the interface physics_core._integrate reads:
        compute(pos, vel, R) -> (F_ned, M_ned)
        _last_info["tension"]  -> scalar [N]
    """
    def __init__(self, force_n: float) -> None:
        self._f = float(force_n)
        self._last_info: dict = {}
        self.rest_length: float = 0.0     # read by some callers; unused here

    def compute(
        self,
        hub_pos: np.ndarray,
        hub_vel: np.ndarray,
        R_hub: "np.ndarray | None" = None,
    ) -> tuple:
        self._last_info = {"tension": self._f, "slack": False,
                           "extension": 0.0, "length": 0.0}
        return np.array([0.0, 0.0, self._f]), np.zeros(3)


# ── IC construction ───────────────────────────────────────────────────────────

def _build_ic() -> SimpleNamespace:
    """
    Hub at altitude 1 m, horizontal disk.  Anchor at NED origin.

    omega_spin / eq_thrust from IC file if available.
    """
    try:
        from tests.simtests.simtest_ic import load_ic as _load_ic
        _ic = _load_ic()
        omega_spin  = float(_ic.omega_spin)
        eq_thrust   = float(_ic.eq_thrust)
    except FileNotFoundError:
        from simulation.param_defaults import load_collective_phys_range as _lr
        col_min, col_max = _lr()
        omega_spin  = 39.42
        eq_thrust   = (-0.18 - col_min) / (col_max - col_min)

    return SimpleNamespace(
        pos         = np.array([0.0, 0.0, -1.0]),  # altitude = 1 m
        vel         = np.zeros(3),
        R0          = np.eye(3),                    # horizontal disk; body_z = [0,0,1]
        rest_length = 1.0,
        eq_thrust   = eq_thrust,
        omega_spin  = omega_spin,
    )


# ── Test ──────────────────────────────────────────────────────────────────────

def test_ground_liftoff():
    """
    Hub at altitude 1 m, horizontal disk, 10 m/s upward wind, 200 N downforce.
    rawes.lua GUIDED mode, RAWES_ALT = 5.0.  Lua VZ-PI must command enough
    collective to climb above 2 m within 60 s.
    """
    ic = _build_ic()

    sim = RawesLua(mode=MODE_STEADY)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 4           # GUIDED
    sim.pos_ned      = ic.pos.tolist()
    sim.vel_ned      = ic.vel.tolist()
    sim.R            = ic.R0
    sim.gyro         = [0.0, 0.0, 0.0]

    runner = PhysicsRunner(
        _ROTOR, ic, WIND_NED,
        z_floor     = 0.0,
    )
    # Swap the elastic tether for a constant 200 N downward load.
    runner._core._tether = _ConstantDownforce(TETHER_FORCE_N)

    lua         = MockArdupilot.for_lua(sim, initial_thrust=ic.eq_thrust, wind=WIND_NED, dt=DT)
    total_steps = int(LIFTOFF_TIMEOUT / DT)
    liftoff_t   = None
    max_alt     = ic.pos[2] * -1.0   # start altitude

    send_anchor_ned(sim, 0.0, 0.0, 0.0)   # anchor at the mock EKF origin

    def _inject(s, r):
        s.send_message(NamedValueFloat("RAWES_TEN", TETHER_FORCE_N))
        s.send_message(NamedValueFloat("RAWES_ALT", TARGET_ALT_M))

    for i in range(total_steps):
        t = i * DT
        if i % LUA_EVERY == 0:
            lua.tick(t, runner, inject=_inject)
        sr = lua.step(runner, DT)

        alt = runner.altitude
        if alt > max_alt:
            max_alt = alt
        if liftoff_t is None and alt >= LIFTOFF_ALT_M:
            liftoff_t = runner.t_sim

    t_final    = runner.t_sim
    omega_final = runner.omega_spin
    print()
    print(f"  t_final      : {t_final:.1f} s")
    print(f"  max_altitude : {max_alt:.3f} m")
    print(f"  omega_spin   : {omega_final:.2f} rad/s  (started {ic.omega_spin:.2f})")
    if liftoff_t is not None:
        print(f"  liftoff_t    : {liftoff_t:.2f} s  [PASS]")
    else:
        print(f"  liftoff_t    : did not reach {LIFTOFF_ALT_M} m  [FAIL]")

    assert liftoff_t is not None, (
        f"Hub did not reach {LIFTOFF_ALT_M} m altitude within {LIFTOFF_TIMEOUT} s "
        f"(max altitude reached: {max_alt:.4f} m)"
    )
    assert liftoff_t < LIFTOFF_TIMEOUT, (
        f"Liftoff at {liftoff_t:.2f} s exceeds timeout {LIFTOFF_TIMEOUT} s"
    )
