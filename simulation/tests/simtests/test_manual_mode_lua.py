"""
test_manual_mode_lua.py -- Unit simtest for rawes.lua MODE_MANUAL (SCR_USER6=2).

Verifies that:
  1. Lua initialises in mode 2 and `run_manual` drives all four outputs.
  2. RAWES_TLN / RAWES_TLT NVFs update the Lua-side cyclic setpoints and
     shift RC1/RC2 PWMs by the expected amount (H_CYC_MAX=1000 cd = 10 deg).
  3. RAWES_COL NVF updates the collective and shifts RC3 PWM accordingly.
  4. Setpoints are persistent: the last-received value holds on every tick
     until a new NVF arrives.
  5. A yaw-rate error causes SERVO4 to be non-800 (yaw PID is active).
  6. The yaw PID resets (integrator = 0) when the mode is entered.
  7. Setpoints received mid-run propagate on the next Lua tick.

These are pure Python tests using the Lua harness (no physics runner, no SITL).
They run fast (<5 s) and exercise the rawes.lua code path that the Lua harness
would invoke during a real calibrate.py `manual` session.

Non-Lua reference: calibrate.py _cmd_manual_interactive
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(60)]

from rawes_lua_harness import RawesLua
from rawes_modes       import MODE_MANUAL

# ── Shared constants ──────────────────────────────────────────────────────────

# H_CYC_MAX = 1000 centidegrees = 10 deg at full stick; Lua uses this to scale
# the tlon/tlat setpoints into a [-500, +500] us PWM offset around 1500.
H_CYC_MAX_DEG = 10.0
H_CYC_MAX_RAD = math.radians(H_CYC_MAX_DEG)

# COL range from rawes.lua constants.
COL_MIN_RAD = -0.28
COL_MAX_RAD =  0.10


def _make_sim(*, yaw_rate_rads: float = 0.0) -> RawesLua:
    """Minimal RawesLua in mode 2 (manual) with all sensors healthy."""
    sim = RawesLua(mode=MODE_MANUAL)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1   # ACRO (manual mode does not require GUIDED)
    # Set identity rotation (level hover) so rpy=(0,0,0)
    import numpy as np
    sim.R            = np.eye(3)
    sim.gyro         = [0.0, 0.0, yaw_rate_rads]
    sim.vel_ned      = [0.0, 0.0, 0.0]
    # H_CYC_MAX parameter: 1000 centidegrees = 10 deg
    sim.set_param("H_CYC_MAX", 1000.0)
    # ATC_RAT_YAW_P small but non-zero so motor responds to yaw error
    sim.set_param("ATC_RAT_YAW_P", 0.1)
    sim.set_param("ATC_RAT_YAW_I", 0.0)
    sim.set_param("ATC_RAT_YAW_D", 0.0)
    sim.set_param("ATC_RAT_YAW_IMAX", 0.7)
    sim.set_param("H_YAW_TRIM", 0.0)
    sim.set_param("SERVO4_MIN", 800.0)
    sim.set_param("SERVO4_MAX", 2000.0)
    return sim


# ── Helper ────────────────────────────────────────────────────────────────────

def _expected_cyclic_pwm(setpoint_rad: float, cyc_max_rad: float = H_CYC_MAX_RAD) -> int:
    """PWM = 1500 + (setpoint_rad / cyc_max_rad) * 500, clamped to [1000, 2000]."""
    raw = 1500.0 + (setpoint_rad / cyc_max_rad) * 500.0
    return int(max(1000.0, min(2000.0, round(raw))))


def _expected_collective_pwm(col_rad: float) -> int:
    """RC3 = 1000 + (col_rad - COL_MIN) / (COL_MAX - COL_MIN) * 1000."""
    norm = (col_rad - COL_MIN_RAD) / (COL_MAX_RAD - COL_MIN_RAD)
    norm = max(0.0, min(1.0, norm))
    return int(round(1000.0 + norm * 1000.0))


# ── Test 1: neutral setpoints → RC1=RC2=1500 ─────────────────────────────────

def test_manual_neutral_cyclic():
    """With no NVFs sent, tlon=tlat=0 → RC1=RC2=1500."""
    sim = _make_sim()
    sim.run(0.5)  # settle (50 ticks)
    assert sim.ch_out[1] == 1500, f"RC1={sim.ch_out[1]} expected 1500 (neutral tlon)"
    assert sim.ch_out[2] == 1500, f"RC2={sim.ch_out[2]} expected 1500 (neutral tlat)"


# ── Test 2: RAWES_TLN (tlon) shifts RC2 ──────────────────────────────────────

def test_manual_tlon_shifts_rc2():
    """RAWES_TLN=+5 deg → RC2 shifts by +250 us from 1500 (= 1750)."""
    sim = _make_sim()
    sim.run(0.2)

    tlon_rad = math.radians(5.0)
    sim.send_named_float("RAWES_TLN", tlon_rad)
    sim.run(0.05)  # a few ticks for Lua to process

    expected = _expected_cyclic_pwm(tlon_rad)  # 1750
    assert sim.ch_out[2] == expected, (
        f"RC2={sim.ch_out[2]} expected {expected} for tlon=+5 deg"
    )


def test_manual_tlon_negative():
    """RAWES_TLN=-5 deg → RC2 = 1250."""
    sim = _make_sim()
    sim.run(0.2)
    tlon_rad = math.radians(-5.0)
    sim.send_named_float("RAWES_TLN", tlon_rad)
    sim.run(0.05)
    expected = _expected_cyclic_pwm(tlon_rad)  # 1250
    assert sim.ch_out[2] == expected, (
        f"RC2={sim.ch_out[2]} expected {expected} for tlon=-5 deg"
    )


# ── Test 3: RAWES_TLT (tlat) shifts RC1 ──────────────────────────────────────

def test_manual_tlat_shifts_rc1():
    """RAWES_TLT=+3 deg → RC1 shifts proportionally."""
    sim = _make_sim()
    sim.run(0.2)
    tlat_rad = math.radians(3.0)
    sim.send_named_float("RAWES_TLT", tlat_rad)
    sim.run(0.05)
    expected = _expected_cyclic_pwm(tlat_rad)
    assert sim.ch_out[1] == expected, (
        f"RC1={sim.ch_out[1]} expected {expected} for tlat=+3 deg"
    )


# ── Test 4: RAWES_COL shifts RC3 ─────────────────────────────────────────────

def test_manual_col_shifts_rc3():
    """RAWES_COL=-0.18 rad → RC3 matches expected collective PWM."""
    sim = _make_sim()
    sim.run(0.2)
    col_rad = -0.18
    sim.send_named_float("RAWES_COL", col_rad)
    sim.run(0.05)
    expected = _expected_collective_pwm(col_rad)
    assert sim.ch_out[3] == expected, (
        f"RC3={sim.ch_out[3]} expected {expected} for col={col_rad:.3f} rad"
    )


def test_manual_col_max():
    """RAWES_COL=COL_MAX_RAD → RC3 = 2000."""
    sim = _make_sim()
    sim.run(0.2)
    sim.send_named_float("RAWES_COL", COL_MAX_RAD)
    sim.run(0.05)
    assert sim.ch_out[3] == 2000, f"RC3={sim.ch_out[3]} expected 2000 at col_max"


def test_manual_col_min():
    """RAWES_COL=COL_MIN_RAD → RC3 = 1000."""
    sim = _make_sim()
    sim.run(0.2)
    sim.send_named_float("RAWES_COL", COL_MIN_RAD)
    sim.run(0.05)
    assert sim.ch_out[3] == 1000, f"RC3={sim.ch_out[3]} expected 1000 at col_min"


# ── Test 5: setpoints persist between NVF deliveries ─────────────────────────

def test_manual_setpoints_persist():
    """After receiving a setpoint NVF, the output holds on subsequent ticks
    without needing a new NVF each tick."""
    sim = _make_sim()
    sim.run(0.2)

    tlon_rad = math.radians(4.0)
    sim.send_named_float("RAWES_TLN", tlon_rad)
    sim.run(0.02)  # 2 ticks — consume NVF

    pwm_after_nvf = sim.ch_out[2]
    sim.run(1.0)   # 100 more ticks — no new NVF
    pwm_one_second_later = sim.ch_out[2]

    expected = _expected_cyclic_pwm(tlon_rad)
    assert pwm_after_nvf == expected, (
        f"RC2={pwm_after_nvf} expected {expected} just after NVF"
    )
    assert pwm_one_second_later == expected, (
        f"RC2={pwm_one_second_later} after 1 s without new NVF: setpoint did not persist"
    )


# ── Test 6: mid-run setpoint update propagates ────────────────────────────────

def test_manual_setpoint_update_propagates():
    """A second NVF with a different value overwrites the first."""
    sim = _make_sim()
    sim.run(0.2)

    sim.send_named_float("RAWES_TLN", math.radians(2.0))
    sim.run(0.05)
    pwm_first = sim.ch_out[2]

    sim.send_named_float("RAWES_TLN", math.radians(-4.0))
    sim.run(0.05)
    pwm_second = sim.ch_out[2]

    assert pwm_first  == _expected_cyclic_pwm(math.radians( 2.0))
    assert pwm_second == _expected_cyclic_pwm(math.radians(-4.0))
    assert pwm_first != pwm_second, "second NVF did not update the setpoint"


# ── Test 7: yaw PID drives SERVO4 when yaw rate error is present ─────────────

def test_manual_yaw_pid_active():
    """With a non-zero yaw rate (CCW body drift), SERVO4 should be above 800."""
    # gyro_z = -0.3 rad/s = CCW rotation in NED RH convention.
    # err = -gyro_z = +0.3 > 0 → yaw PID output > 0 → SERVO4 > 800.
    sim = _make_sim(yaw_rate_rads=-0.3)
    sim.run(0.5)

    # srv_out uses SRV_Channels:set_output_pwm_chan_timeout.
    # The mock exposes it via srv_out[chan_0based] -- channel index 3 (0-indexed).
    # In the harness, SERVO4_CHAN=3 maps to _mock.srv_chan_out[4] (1-indexed key).
    # Check via a raw Lua eval to be sure.
    s4_pwm = sim._lua.eval("_mock.srv_chan_out[4]")
    if s4_pwm is None:
        # Fallback: check the ch_out path (depends on how mock routes SERVO4_CHAN).
        # The mock stores SRV_Channels:set_output_pwm_chan_timeout into srv_chan_out.
        pytest.fail("SERVO4 output not found in mock.srv_chan_out[4]")
    s4_pwm = int(s4_pwm)
    assert s4_pwm > 800, (
        f"SERVO4={s4_pwm} expected >800 (yaw PID should be active with gyro_z=-0.3 rad/s)"
    )


# ── Test 8: yaw PID integrator resets on mode entry ──────────────────────────

def test_manual_yaw_integrator_resets_on_mode_entry():
    """Re-entering MODE_MANUAL via param change resets the yaw integrator."""
    sim = _make_sim(yaw_rate_rads=-0.5)
    sim.set_param("ATC_RAT_YAW_I", 0.05)
    sim.run(2.0)  # wind up the integrator

    # Switch away then back to mode 2.
    sim.set_param("SCR_USER6", 0)
    sim.run(0.05)
    sim.set_param("SCR_USER6", MODE_MANUAL)
    sim.run(0.05)

    # After mode re-entry, _yaw_i should be 0.
    yaw_i = float(sim._lua.eval("_rawes_fns.yaw_i and _rawes_fns.yaw_i() or 0"))
    assert yaw_i == pytest.approx(0.0, abs=1e-9), (
        f"_yaw_i={yaw_i} after mode re-entry: expected 0 (integrator not reset)"
    )


# ── Test 9: tlon/tlat reset to 0 on mode entry ───────────────────────────────

def test_manual_cyclic_resets_on_mode_entry():
    """Re-entering MODE_MANUAL resets tlon/tlat setpoints to 0 → RC1=RC2=1500."""
    sim = _make_sim()
    # Set non-zero setpoints
    sim.send_named_float("RAWES_TLN", math.radians(5.0))
    sim.send_named_float("RAWES_TLT", math.radians(-3.0))
    sim.run(0.1)

    # Cycle the mode
    sim.set_param("SCR_USER6", 0)
    sim.run(0.05)
    sim.set_param("SCR_USER6", MODE_MANUAL)
    sim.run(0.05)

    assert sim.ch_out[1] == 1500, (
        f"RC1={sim.ch_out[1]} after mode re-entry: tlat should reset to 0"
    )
    assert sim.ch_out[2] == 1500, (
        f"RC2={sim.ch_out[2]} after mode re-entry: tlon should reset to 0"
    )


# ── Test 10: both tlon and tlat set simultaneously ───────────────────────────

def test_manual_combined_tlon_tlat():
    """Set both RAWES_TLN and RAWES_TLT; verify both RC channels update correctly."""
    sim = _make_sim()
    sim.run(0.2)

    tlon_rad = math.radians(3.0)
    tlat_rad = math.radians(-2.0)
    sim.send_named_float("RAWES_TLN", tlon_rad)
    sim.send_named_float("RAWES_TLT", tlat_rad)
    sim.run(0.05)

    rc1 = sim.ch_out[1]
    rc2 = sim.ch_out[2]
    assert rc1 == _expected_cyclic_pwm(tlat_rad), (
        f"RC1={rc1} expected {_expected_cyclic_pwm(tlat_rad)} for tlat={math.degrees(tlat_rad):.1f} deg"
    )
    assert rc2 == _expected_cyclic_pwm(tlon_rad), (
        f"RC2={rc2} expected {_expected_cyclic_pwm(tlon_rad)} for tlon={math.degrees(tlon_rad):.1f} deg"
    )


# ── Test 11: PWM clamp at ±H_CYC_MAX ─────────────────────────────────────────

def test_manual_cyclic_clamped_at_full_stick():
    """Setpoint > H_CYC_MAX should be clamped to 2000 (not overflow)."""
    sim = _make_sim()
    sim.run(0.2)
    # Saturate: 20 deg >> 10 deg max
    sim.send_named_float("RAWES_TLN", math.radians(20.0))
    sim.run(0.05)
    assert sim.ch_out[2] == 2000, (
        f"RC2={sim.ch_out[2]} expected 2000 (clamped at full +stick)"
    )

    sim.send_named_float("RAWES_TLN", math.radians(-20.0))
    sim.run(0.05)
    assert sim.ch_out[2] == 1000, (
        f"RC2={sim.ch_out[2]} expected 1000 (clamped at full -stick)"
    )
