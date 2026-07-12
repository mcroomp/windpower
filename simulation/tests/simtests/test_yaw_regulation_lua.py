"""
test_yaw_regulation_lua.py — Closed-loop Lua yaw trim observer simtest.

Plant
-----
torque_model.step() — the real GB4008 motor + ESC governor dynamics used by
mediator_torque.  No shortcuts: the motor speed responds via the ESC governor
(~40 ms tau), and the hub yaw rate is derived from motor speed vs rotor speed
through the 10:1 gear.

RAWES_YAW_SLP calibration
---------------------
rawes.lua reads RAWES_YAW_SLP [RPM/µs] and converts it to the observer slope YFF_A.
The test sets RAWES_YAW_SLP to the value derived from torque_model constants
(the result a bench calibration of d(shaft_RPM)/d(PWM_µs) would produce for
this motor + gear combination).  The test then verifies:

  1. YFF_A inside Lua equals the plant's static d(psi_dot)/d(throttle).
  2. The closed-loop observer drives |psi_dot| below MAX_DEG_S within SETTLE_S.

This makes the test honest: if in real life RAWES_YAW_SLP is set from an actual
bench measurement of the motor, the observer will work the same way.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(30)]

import torque_model as _m
from rawes_lua_harness import RawesLua
from rawes_modes import MODE_PASSIVE

# ---------------------------------------------------------------------------
# Plant constants (torque_model defaults — GB4008 66KV, 80:44 gear, 4S LiPo)
# ---------------------------------------------------------------------------
_PARAMS      = _m.HubParams()
_OMEGA_ROTOR = 200.0 * (2.0 * math.pi / 60.0)  # 200 RPM = 20.94 rad/s (design target)
# Motor equilibrium at this rotor speed: 200 RPM × 10:1 gear = 2000 RPM motor

# True static slope: d(psi_dot_ss)/d(throttle) = RPM_SCALE / GEAR_RATIO
_PLANT_SLOPE = _PARAMS.rpm_scale / _PARAMS.gear_ratio   # ≈ 57.8 rad/s per unit

# SERVO9 span that rawes.lua uses for normalisation
_SERVO9_SPAN_US = 1000.0   # SERVO9_MAX − SERVO9_MIN

# Correct RAWES_YAW_SLP derived from plant constants [RPM/µs]:
#   YFF_A_lua = RAWES_YAW_SLP × SERVO9_SPAN_US × 2π/60
#   set YFF_A_lua = plant slope  →  RAWES_YAW_SLP = plant_slope / (span × 2π/60)
_YAW_SLP = _PLANT_SLOPE / (_SERVO9_SPAN_US * (2.0 * math.pi / 60.0))

# Equilibrium throttle (psi_dot = 0):
#   0 = −omega_rotor + throttle × RPM_SCALE / GEAR_RATIO
#   throttle_eq = omega_rotor × GEAR_RATIO / RPM_SCALE
#   = 20.94 × 10 / 578 ≈ 0.362  →  PWM ≈ 1362 µs
_U_EQ = _OMEGA_ROTOR * _PARAMS.gear_ratio / _PARAMS.rpm_scale   # ≈ 0.362

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
DT        = 0.01    # 10 ms — rawes.lua BASE_PERIOD_MS
SETTLE_S  = 5.0     # settle window; 3×TAU(=0.9s) convergence + generous margin
OBSERVE_S = 2.0     # observation window after settle
MAX_DEG_S = 5.0     # |psi_dot| threshold [deg/s]

_SERVO9_MIN = 1000.0
_SERVO9_MAX = 2000.0

_IC_PITCH = -1.1102   # rad (≈ −63.6 deg, tethered-hover IC)
_IC_ROLL  =  0.0
_IC_COL   = -0.15     # rad


def test_yaw_regulation_lua():
    """
    rawes.lua yaw trim observer converges psi_dot to ≤ MAX_DEG_S
    when RAWES_YAW_SLP is set to the slope derived from the real plant.
    """
    sim = RawesLua(mode=MODE_PASSIVE, RAWES_YAW_SLP=_YAW_SLP)
    sim.armed   = True
    sim.healthy = True

    # IC orientation
    cp, sp = math.cos(_IC_PITCH), math.sin(_IC_PITCH)
    cr, sr = math.cos(_IC_ROLL),  math.sin(_IC_ROLL)
    sim.R = np.array([
        [ cp,       0,   sp      ],
        [ sr * sp,  cr, -sr * cp ],
        [-cr * sp,  sr,  cr * cp ],
    ])

    # Verify RAWES_YAW_SLP was applied: Lua YFF_A must equal the plant slope
    yff_a_lua = float(sim.fns.YFF_A)
    assert abs(yff_a_lua - _PLANT_SLOPE) < 0.5, (
        f"YFF_A mismatch after RAWES_YAW_SLP override: "
        f"Lua={yff_a_lua:.3f}  plant={_PLANT_SLOPE:.3f} rad/s/u"
    )

    # Seed IC so _ic_seeded becomes True on the first update() tick
    sim.send_named_float("RAWES_COL", _IC_COL)
    sim.send_named_float("RAWES_RIC", _IC_ROLL)
    sim.send_named_float("RAWES_PIC", _IC_PITCH)

    # ---------------------------------------------------------------------------
    # Closed-loop simulation
    # ---------------------------------------------------------------------------
    hub  = _m.HubState()   # psi=0, psi_dot=0, omega_motor=0
    trim = 0.0             # H_YAW_TRIM as written by observer (initially 0)
    t    = 0.0
    violations: list[float] = []

    n_ticks = round((SETTLE_S + OBSERVE_S) / DT)
    for _ in range(n_ticks):
        # Inject current plant state into Lua before the tick
        pwm = _SERVO9_MIN + trim * (_SERVO9_MAX - _SERVO9_MIN)
        sim.gyro = [0.0, 0.0, hub.psi_dot]
        sim.set_srv_out(36, pwm)

        # Lua reads gyro + SERVO9 output, updates H_YAW_TRIM
        sim.tick()

        # Read the new trim and advance the plant
        trim = sim.get_param("H_YAW_TRIM") or 0.0
        hub  = _m.step(hub, _OMEGA_ROTOR, trim, _PARAMS, DT)
        t   += DT

        if t > SETTLE_S:
            violations.append(abs(hub.psi_dot))

    if not violations:
        pytest.fail("No observation samples collected — check SETTLE_S vs total time")

    max_psi_dot_deg_s = math.degrees(max(violations))
    assert max_psi_dot_deg_s < MAX_DEG_S, (
        f"psi_dot did not converge after {SETTLE_S:.0f} s settle: "
        f"max={max_psi_dot_deg_s:.2f} deg/s  (limit {MAX_DEG_S:.1f} deg/s)\n"
        f"Check RAWES_YAW_SLP calibration: set to {_YAW_SLP:.4f} RPM/µs "
        f"(derived from RPM_SCALE={_PARAMS.rpm_scale} / GEAR_RATIO={_PARAMS.gear_ratio:.3f})"
    )
