"""Unit tests for the RAWES_ARM disarm-timer behaviour in rawes.lua.

Current semantics (run_armon):
1. RAWES_ARM sets an absolute disarm deadline (`now + ms`) and stores seconds.
2. Arming/disarming itself is handled externally (GCS/ArduPilot).
3. Expiry only acts when currently armed: disarm + STATUSTEXT.
"""
from __future__ import annotations

import sys
from pathlib import Path


from simulation.rawes_lua_harness import RawesLua
from groundstation.gcs import NamedValueFloat


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _sim() -> RawesLua:
    """Fresh RawesLua instance in mode 0 (none) — ARMON runs in all modes."""
    return RawesLua()


def _armon_deadline_ms(sim: RawesLua):
    v = sim.fns.armon_deadline_ms()
    return float(v) if v is not None else None


def _send_arm(sim: RawesLua, ms: float):
    sim.send_message(NamedValueFloat("RAWES_ARM", ms))


# ---------------------------------------------------------------------------
# 1. Deadline programming and status text
# ---------------------------------------------------------------------------

class TestArmonTimerProgramming:
    def test_set_timer_while_armed_sets_deadline_and_seconds(self):
        sim = _sim()
        sim.armed = True
        _send_arm(sim, 60_000.0)
        sim.tick()
        assert _armon_deadline_ms(sim) == sim.t_ms + 60_000.0
        assert int(sim.fns.armon_secs()) == 60
        assert sim.has_message("RAWES disarm timer set: 60s")

    def test_set_timer_while_unarmed_emits_warning(self):
        sim = _sim()
        sim.armed = False
        _send_arm(sim, 5_000.0)
        sim.tick()
        assert _armon_deadline_ms(sim) == sim.t_ms + 5_000.0
        assert sim.has_message("RAWES disarm timer set while unarmed")


# ---------------------------------------------------------------------------
# 2. Expiry behaviour
# ---------------------------------------------------------------------------

class TestArmonExpiry:
    def test_expiry_disarms_and_clears_deadline_when_armed(self):
        sim = _sim()
        sim.armed = True
        _send_arm(sim, 100.0)
        sim.tick()
        sim.run(0.2)
        assert sim.armed is False
        assert _armon_deadline_ms(sim) is None
        assert sim.has_message("RAWES disarm timer expired, disarmed")

    def test_no_expiry_before_deadline(self):
        sim = _sim()
        sim.armed = True
        _send_arm(sim, 1_000.0)
        sim.tick()
        sim.run(0.5)
        assert sim.armed is True
        assert _armon_deadline_ms(sim) is not None
        assert not sim.has_message("expired")

    def test_expiry_disarms_in_passive_before_ic_seed(self):
        """RAWES_ARM expiry must still disarm in MODE_PASSIVE before IC seed."""
        sim = RawesLua(mode=3)  # MODE_PASSIVE
        sim.armed = True
        _send_arm(sim, 100.0)
        sim.tick()
        sim.run(0.2)
        assert sim.armed is False
        assert _armon_deadline_ms(sim) is None
        assert sim.has_message("RAWES disarm timer expired, disarmed")


# ---------------------------------------------------------------------------
# 3. Deadline refresh and deferred armed expiry
# ---------------------------------------------------------------------------

class TestArmonDeadlineRefresh:
    def test_resend_extends_deadline(self):
        sim = _sim()
        sim.armed = True
        _send_arm(sim, 200.0)
        sim.tick()
        first_deadline = _armon_deadline_ms(sim)
        sim.run(0.1)
        _send_arm(sim, 1_000.0)
        sim.tick()
        assert _armon_deadline_ms(sim) > first_deadline
        sim.run(0.2)
        assert sim.armed is True

    def test_expiry_is_only_enforced_when_armed(self):
        sim = _sim()
        sim.armed = False
        _send_arm(sim, 100.0)
        sim.tick()
        sim.run(0.2)
        # Deadline remains programmed while unarmed.
        assert _armon_deadline_ms(sim) is not None
        sim.armed = True
        sim.tick()
        # Once armed after deadline has passed, next tick expires immediately.
        assert sim.armed is False
        assert _armon_deadline_ms(sim) is None


# ---------------------------------------------------------------------------
# 8. CH4 yaw always neutral
# ---------------------------------------------------------------------------

class TestCh4AlwaysNeutral:
    def test_ch4_neutral_in_mode0_before_arm(self):
        """CH4 must be overridden to 1500 even in passive mode 0 before arming."""
        sim = _sim()
        sim.tick()
        assert sim.ch_out[4] == 1500, f"CH4={sim.ch_out[4]}, expected 1500"

    def test_ch4_neutral_in_mode1(self):
        """CH4 stays neutral during steady flight (mode 1)."""
        sim = RawesLua(mode=1)
        sim.tick()
        assert sim.ch_out[4] == 1500

    def test_ch4_neutral_while_armed(self):
        """CH4 stays neutral throughout RAWES_ARM armed window."""
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.run(0.5)  # reach armed state
        assert sim.ch_out[4] == 1500

    def test_ch4_set_every_tick(self):
        """CH4=1500 must be written on every tick, not just once."""
        sim = _sim()
        for _ in range(20):
            sim.tick()
            assert sim.ch_out[4] == 1500
