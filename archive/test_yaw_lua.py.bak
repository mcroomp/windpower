"""
test_yaw_lua.py -- Unit tests for rawes.lua yaw-trim controller logic.

Runs the actual rawes.lua Lua code via the RawesLua harness (lupa + Lua 5.4),
with a mock ArduPilot API.  Tests inject gyro values and read SRV_Channels
output to verify every branch of run_yaw_trim().

Covered:
  1.  PWM conversion (_set_throttle_pct)
  2.  Dead zone at startup: motor stays off below threshold
  3.  Dead zone exit: motor starts when movement exceeds threshold
  4.  Dead zone re-entry: dead zone re-applies after motor is cut to 0
  5.  I term integration: accumulates correctly over time
  6.  I term anti-windup: clamps at YAW_I_MAX
  7.  Closed-loop equilibrium: PI converges to < 5 deg/s with physics model
  8.  Stability watchdog: latches _yaw_stopped after 30 s of |gyro_z| > 5 deg/s
  9.  Stability watchdog resets while stable
  10. Wrong-direction guard: immediate motor cut at gyro_z < -30 deg/s
  11. Disarm: motor cuts and dead zone re-latches

No SITL, no Docker.  Runs natively with the unit-test venv.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import torque_model as m
from rawes_lua_harness import RawesLua
from rawes_modes import MODE_YAW_LUA

# ---------------------------------------------------------------------------
# Constants -- read directly from rawes.lua via _rawes_fns so they never
# drift out of sync.  rawes_test_surface.lua exposes them; if a constant is
# missing here, add it to the _rawes_fns table in that file first.
# ---------------------------------------------------------------------------

_c = RawesLua()
BASE_THROTTLE_PCT     = float(_c.fns.BASE_THROTTLE_PCT)
KP_YAW                = float(_c.fns.KP_YAW)
KI_YAW                = float(_c.fns.KI_YAW)
YAW_I_MAX             = float(_c.fns.YAW_I_MAX)
YAW_DEAD_ZONE_RAD_S   = float(_c.fns.YAW_DEAD_ZONE_RAD_S)
YAW_STABLE_RAD_S      = float(_c.fns.YAW_STABLE_RAD_S)
YAW_STABLE_TIMEOUT_MS = int(_c.fns.YAW_STABLE_TIMEOUT_MS)
YAW_SRV_FUNC          = int(_c.fns.YAW_SRV_FUNC)
DT_MS                 = int(_c.fns.BASE_PERIOD_MS)
DT_S                  = DT_MS / 1000.0
del _c


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sim(mode: int = MODE_YAW_LUA) -> RawesLua:
    sim = RawesLua(mode=mode)
    sim.armed   = True
    sim.healthy = True
    return sim


def _pct(sim: RawesLua) -> float:
    """Throttle percentage read from SRV_Channels output (0.0 when motor off)."""
    pwm = sim.srv_pwm(YAW_SRV_FUNC)
    return (pwm - 800) / 12.0 if pwm is not None else 0.0


def _step(sim: RawesLua, gyro_z: float) -> float:
    """Set gyro_z, advance one 10 ms tick, return throttle percentage."""
    sim.gyro = [0.0, 0.0, gyro_z]
    sim.tick()
    return _pct(sim)


def _run_n(sim: RawesLua, gyro_z: float, n: int) -> list[float]:
    return [_step(sim, gyro_z) for _ in range(n)]


# ===========================================================================
# 1. PWM conversion
# ===========================================================================

class TestPwmConversion:

    def test_zero_throttle_is_800(self):
        sim = _make_sim()
        assert int(sim.fns.pwm_for_pct(0.0)) == 800

    def test_full_throttle_is_2000(self):
        sim = _make_sim()
        assert int(sim.fns.pwm_for_pct(100.0)) == 2000

    def test_halfway_throttle(self):
        # 50% -> 800 + 50*12 = 1400
        sim = _make_sim()
        assert int(sim.fns.pwm_for_pct(50.0)) == 1400

    def test_range_monotonic(self):
        sim = _make_sim()
        prev = int(sim.fns.pwm_for_pct(0.0))
        for pct in range(1, 101):
            pwm = int(sim.fns.pwm_for_pct(float(pct)))
            assert pwm >= prev
            prev = pwm

    def test_range_within_servo_bounds(self):
        sim = _make_sim()
        for pct in [0.0, 25.0, 50.0, 75.0, 100.0]:
            pwm = int(sim.fns.pwm_for_pct(pct))
            assert 800 <= pwm <= 2000, f"PWM {pwm} out of range at {pct}%"


# ===========================================================================
# 2. Dead zone at startup
# ===========================================================================

class TestDeadZone:

    def test_zero_gyro_stays_off(self):
        sim = _make_sim()
        outputs = _run_n(sim, 0.0, 100)
        assert all(t == 0.0 for t in outputs), "Motor must stay off at gyro_z=0"

    def test_below_threshold_stays_off(self):
        sim = _make_sim()
        below = YAW_DEAD_ZONE_RAD_S * 0.99
        outputs = _run_n(sim, below, 100)
        assert all(t == 0.0 for t in outputs), \
            f"Motor must stay off below dead zone ({math.degrees(below):.2f} deg/s)"

    def test_at_threshold_starts(self):
        """At exactly the dead zone boundary the motor must start (strict < check in Lua)."""
        sim = _make_sim()
        out = _step(sim, YAW_DEAD_ZONE_RAD_S)
        assert out > 0.0, "Motor must start at dead zone boundary"

    def test_above_threshold_starts(self):
        sim = _make_sim()
        out = _step(sim, YAW_DEAD_ZONE_RAD_S * 1.5)
        assert out > 0.0

    def test_in_dead_zone_flag(self):
        sim = _make_sim()
        _step(sim, 0.0)
        assert bool(sim.fns.yaw_in_dead_zone()) is True

    def test_leaving_dead_zone_clears_flag(self):
        sim = _make_sim()
        _step(sim, YAW_DEAD_ZONE_RAD_S * 2.0)
        assert bool(sim.fns.yaw_in_dead_zone()) is False

    def test_dead_zone_reapplies_after_motor_cut(self):
        """
        After wrong-direction guard cuts motor (last_throttle=0),
        dead zone re-applies if gyro drops back below threshold.
        """
        sim = _make_sim()
        # Spin up: exit dead zone and let I build briefly
        for _ in range(50):
            _step(sim, math.radians(10.0))
        # Wrong direction: force cut
        _step(sim, -math.radians(31.0))
        # Small positive gyro -- must be back in dead zone
        out = _step(sim, math.radians(1.0))
        assert out == 0.0, "Dead zone must re-latch after motor cut"


# ===========================================================================
# 3. I term integration
# ===========================================================================

class TestIntegrator:

    def test_i_term_grows_with_positive_gyro(self):
        sim = _make_sim()
        _step(sim, YAW_DEAD_ZONE_RAD_S * 2.0)   # exit dead zone
        i_before = float(sim.fns.yaw_i())
        for _ in range(20):
            _step(sim, math.radians(10.0))
        assert float(sim.fns.yaw_i()) > i_before, "I term must grow with positive gyro_z"

    def test_i_term_decreases_with_negative_gyro(self):
        sim = _make_sim()
        for _ in range(200):
            _step(sim, math.radians(10.0))
        i_peak = float(sim.fns.yaw_i())
        # Apply negative gyro (but above wrong-direction threshold)
        for _ in range(50):
            _step(sim, -math.radians(4.0))
        assert float(sim.fns.yaw_i()) < i_peak, "I term must decrease with negative gyro_z"

    def test_i_term_clamps_at_maximum(self):
        sim = _make_sim()
        for _ in range(10000):
            _step(sim, math.radians(20.0))
        assert float(sim.fns.yaw_i()) <= YAW_I_MAX + 1e-9, \
            f"I term {float(sim.fns.yaw_i()):.2f} exceeded YAW_I_MAX={YAW_I_MAX}"

    def test_i_term_clamps_at_minimum(self):
        sim = _make_sim()
        for _ in range(500):
            _step(sim, math.radians(10.0))
        for _ in range(10000):
            _step(sim, -math.radians(4.0))
        assert float(sim.fns.yaw_i()) >= -YAW_I_MAX - 1e-9, \
            f"I term {float(sim.fns.yaw_i()):.2f} below -YAW_I_MAX={-YAW_I_MAX}"

    def test_i_term_reset_on_wrong_direction(self):
        sim = _make_sim()
        for _ in range(300):
            _step(sim, math.radians(10.0))
        assert float(sim.fns.yaw_i()) > 1.0
        _step(sim, -math.radians(31.0))
        assert float(sim.fns.yaw_i()) == 0.0, "I term must reset on wrong-direction cut"

    def test_i_term_reset_on_mode_entry(self):
        """Mode re-entry via SCR_USER6 change must reset I term (_on_mode_enter)."""
        sim = _make_sim()
        for _ in range(300):
            _step(sim, math.radians(10.0))
        assert float(sim.fns.yaw_i()) > 1.0
        # Zero gyro so run_yaw_trim() stays in dead zone on the re-entry tick
        # (otherwise the PI would immediately accumulate a tiny I after _on_mode_enter resets it)
        sim.gyro = [0.0, 0.0, 0.0]
        # Switch to mode 0 then back to mode 2 to trigger two _on_mode_enter calls
        sim.set_param("mode", 0)
        sim.tick()
        sim.set_param("mode", MODE_YAW_LUA)
        sim.tick()
        assert float(sim.fns.yaw_i()) == 0.0


# ===========================================================================
# 4. Throttle output formula
# ===========================================================================

class TestThrottleFormula:

    def test_output_at_zero_gyro_equals_base_plus_i(self):
        """At gyro_z=0, throttle = BASE + I (P=0, KI*0=no I change)."""
        sim = _make_sim()
        for _ in range(100):
            _step(sim, math.radians(10.0))
        i_before = float(sim.fns.yaw_i())
        _step(sim, 0.0)
        expected_pct = max(0.0, min(100.0, BASE_THROTTLE_PCT + i_before))
        expected_pwm = int(sim.fns.pwm_for_pct(expected_pct))
        assert sim.srv_pwm(YAW_SRV_FUNC) == expected_pwm, \
            f"throttle at gyro_z=0: got {_pct(sim):.3f}%, expected {expected_pct:.3f}%"

    def test_p_term_reduces_throttle_for_negative_gyro(self):
        """Negative gyro_z reduces throttle via P term."""
        sim = _make_sim()
        for _ in range(100):
            _step(sim, math.radians(10.0))
        i_before = float(sim.fns.yaw_i())
        pct_neg = _step(sim, -math.radians(1.0))
        # Without P contribution: BASE + I; with neg P: lower
        out_zero_ref = BASE_THROTTLE_PCT + i_before
        assert pct_neg < out_zero_ref, "Negative gyro must reduce throttle"

    def test_p_term_increases_throttle_for_positive_gyro(self):
        """Positive gyro_z above zero raises throttle above BASE + I."""
        sim = _make_sim()
        for _ in range(100):
            _step(sim, math.radians(10.0))
        i_before = float(sim.fns.yaw_i())
        pct_pos = _step(sim, math.radians(3.0))
        out_base = max(0.0, min(100.0, BASE_THROTTLE_PCT + i_before))
        assert pct_pos > out_base, "Positive gyro must increase throttle"

    def test_throttle_clamped_to_zero(self):
        """Throttle never goes below 0."""
        sim = _make_sim()
        for _ in range(50):
            _step(sim, math.radians(10.0))
        # Large negative gyro (above wrong-direction threshold of -30 deg/s)
        out = _step(sim, -math.radians(29.0))
        assert out >= 0.0

    def test_throttle_clamped_to_100(self):
        """Throttle never exceeds 100%."""
        sim = _make_sim()
        for _ in range(10000):
            _step(sim, math.radians(20.0))
        out = _step(sim, math.radians(20.0))
        assert out <= 100.0


# ===========================================================================
# 5. Stability watchdog
# ===========================================================================

class TestStabilityWatchdog:

    def test_no_latch_below_duration(self):
        """Instability shorter than 30 s must not latch _yaw_stopped."""
        sim = _make_sim()
        steps_29s = int(29_900 / DT_MS)
        _run_n(sim, math.radians(6.0), steps_29s)
        assert not bool(sim.fns.yaw_stopped()), \
            "watchdog must not latch before 30 s of instability"

    def test_latches_after_30s(self):
        """Sustained |gyro_z| > 5 deg/s for >= 30 s must latch _yaw_stopped."""
        sim = _make_sim()
        steps_31s = int(31_000 / DT_MS)
        _run_n(sim, math.radians(6.0), steps_31s)
        assert bool(sim.fns.yaw_stopped()), \
            "watchdog must latch after 30 s of continuous instability"
        assert not sim.armed, "arming:disarm() must be called when watchdog fires"

    def test_motor_off_after_latch(self):
        sim = _make_sim()
        _run_n(sim, math.radians(6.0), int(31_000 / DT_MS))
        assert bool(sim.fns.yaw_stopped())
        out = _step(sim, math.radians(10.0))
        assert out == 0.0, "Motor must be off after watchdog latch"

    def test_watchdog_resets_while_stable(self):
        """Stability within band resets the watchdog timer."""
        sim = _make_sim()
        # 20 s of instability
        _run_n(sim, math.radians(6.0), int(20_000 / DT_MS))
        assert sim.fns.yaw_not_stable_ms() is not None
        # 1 step within stable band -> timer clears
        _step(sim, math.radians(1.0))
        assert sim.fns.yaw_not_stable_ms() is None, \
            "Watchdog timer must clear when gyro returns within stable band"
        # Another 20 s of instability -- must NOT latch (timer restarted)
        _run_n(sim, math.radians(6.0), int(20_000 / DT_MS))
        assert not bool(sim.fns.yaw_stopped()), \
            "Watchdog must not latch when stability was recovered mid-sequence"


# ===========================================================================
# 6. Wrong-direction guard
# ===========================================================================

class TestWrongDirectionGuard:

    def test_cut_at_neg30(self):
        sim = _make_sim()
        for _ in range(50):
            _step(sim, math.radians(10.0))
        out = _step(sim, -math.radians(31.0))
        assert out == 0.0, "Motor must cut at gyro_z < -30 deg/s"

    def test_i_reset_at_neg30(self):
        sim = _make_sim()
        for _ in range(50):
            _step(sim, math.radians(10.0))
        assert float(sim.fns.yaw_i()) > 0.0
        _step(sim, -math.radians(31.0))
        assert float(sim.fns.yaw_i()) == 0.0

    def test_no_cut_just_above_threshold(self):
        """gyro_z = -29 deg/s is above the wrong-direction threshold; I is not reset."""
        sim = _make_sim()
        for _ in range(100):
            _step(sim, math.radians(10.0))
        assert float(sim.fns.yaw_i()) > 0.0
        _step(sim, -math.radians(29.0))
        # I must NOT have been forcibly reset (that only happens at < -30 deg/s)
        assert float(sim.fns.yaw_i()) != 0.0, \
            "I term must NOT be reset at -29 deg/s (only at < -30 deg/s)"
        assert not bool(sim.fns.yaw_stopped())


# ===========================================================================
# 7. Disarm
# ===========================================================================

class TestDisarm:

    def test_disarm_cuts_motor(self):
        sim = _make_sim()
        for _ in range(50):
            _step(sim, math.radians(10.0))
        sim.armed = False
        out = _step(sim, math.radians(10.0))
        assert out == 0.0

    def test_disarm_resets_dead_zone_flag(self):
        """arming:disarm() must set _yaw_in_dead_zone=true."""
        sim = _make_sim()
        for _ in range(50):
            _step(sim, math.radians(10.0))
        assert bool(sim.fns.yaw_in_dead_zone()) is False
        sim.armed = False
        _step(sim, math.radians(10.0))
        assert bool(sim.fns.yaw_in_dead_zone()) is True

    def test_disarm_motor_stays_off(self):
        """After disarm, motor stays off regardless of gyro."""
        sim = _make_sim()
        for _ in range(50):
            _step(sim, math.radians(10.0))
        sim.armed = False
        for _ in range(10):
            out = _step(sim, math.radians(20.0))
            assert out == 0.0


# ===========================================================================
# 9. Closed-loop equilibrium with physics model
# ===========================================================================

class TestClosedLoopEquilibrium:
    """
    Run the Lua PI controller in closed loop with torque_model.py physics.
    Physics at 400 Hz; Lua ticks at 100 Hz (1 tick per 4 physics steps).
    Mirrors the SITL test_lua_yaw_trim: settle 65 s, observe 10 s, 5 deg/s limit.
    """

    SETTLE_S  = 65.0
    OBSERVE_S = 10.0
    THRESHOLD = math.radians(5.0)

    def _run_loop(self, sim, t_end):
        params = m.HubParams()
        state  = m.HubState()
        dt_phy = 1.0 / 400.0
        yaw_accum = 0.0
        t = 0.0
        throttle = 0.0
        observe  = []

        while t < t_end:
            state = m.step(state, m.OMEGA_ROTOR_NOMINAL,
                           throttle, params, dt_phy)
            t         += dt_phy
            yaw_accum += dt_phy

            if yaw_accum >= DT_S - 1e-9:
                sim.gyro = [0.0, 0.0, state.psi_dot]
                sim.tick()
                pwm      = sim.srv_pwm(YAW_SRV_FUNC) or 800
                throttle = (pwm - 800) / 1200.0
                yaw_accum = 0.0

            if t >= self.SETTLE_S:
                observe.append(abs(state.psi_dot))

        return state, throttle, observe

    def test_pi_converges_within_threshold(self):
        """
        Lua PI must hold |psi_dot| < 5 deg/s after 65 s settle.
        """
        sim = _make_sim()
        _, _, observe = self._run_loop(sim, self.SETTLE_S + self.OBSERVE_S)
        max_psi_dot = max(observe)
        assert max_psi_dot < self.THRESHOLD, (
            f"PI did not converge: max |psi_dot|={math.degrees(max_psi_dot):.2f} deg/s "
            f"> threshold={math.degrees(self.THRESHOLD):.1f} deg/s"
        )

    def test_i_term_reaches_equilibrium(self):
        """After 65 s, the I term must be within 10 pp of the equilibrium throttle."""
        sim = _make_sim()
        self._run_loop(sim, self.SETTLE_S)
        params = m.HubParams()
        eq_pct  = m.equilibrium_throttle(m.OMEGA_ROTOR_NOMINAL, params) * 100.0
        i_pct   = float(sim.fns.yaw_i())
        assert abs(i_pct - eq_pct) < 10.0, (
            f"I term {i_pct:.1f}% not near equilibrium {eq_pct:.1f}%"
        )

    def test_rotor_stop_drains_motor(self):
        """
        If the rotor stops mid-run, the I term drains and throttle returns to ~0.
        """
        params = m.HubParams()
        state  = m.HubState()
        sim    = _make_sim()
        dt_phy = 1.0 / 400.0

        # Run 65 s at nominal RPM
        yaw_accum = 0.0
        t = 0.0
        throttle = 0.0
        while t < self.SETTLE_S:
            state = m.step(state, m.OMEGA_ROTOR_NOMINAL,
                           throttle, params, dt_phy)
            t         += dt_phy
            yaw_accum += dt_phy
            if yaw_accum >= DT_S - 1e-9:
                sim.gyro = [0.0, 0.0, state.psi_dot]
                sim.tick()
                pwm      = sim.srv_pwm(YAW_SRV_FUNC) or 800
                throttle = (pwm - 800) / 1200.0
                yaw_accum = 0.0

        assert float(sim.fns.yaw_i()) > 10.0, "I term should be significant after 65 s"

        # Stop rotor; run another 120 s with omega_rotor=0
        yaw_accum = 0.0
        t_stop = t
        while t < t_stop + 120.0:
            state = m.step(state, 0.0, throttle, params, dt_phy)
            t         += dt_phy
            yaw_accum += dt_phy
            if yaw_accum >= DT_S - 1e-9:
                sim.gyro = [0.0, 0.0, state.psi_dot]
                sim.tick()
                pwm      = sim.srv_pwm(YAW_SRV_FUNC) or 800
                throttle = (pwm - 800) / 1200.0
                yaw_accum = 0.0

        assert _pct(sim) < 10.0, (
            f"After rotor stop, throttle should drain to ~0; got {_pct(sim):.1f}%"
        )
