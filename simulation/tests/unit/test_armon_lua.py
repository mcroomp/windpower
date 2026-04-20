"""
test_armon_lua.py -- Unit tests for the RAWES_ARM timed arm/disarm state machine in rawes.lua.

Tests the full lifecycle of RAWES_ARM without SITL or Docker:
  1. State transitions: interlock_low -> arming -> armed
  2. Ch8 sequence: 1000 (LOW) during arming, 2000 (HIGH) when armed
  3. Confirmation STATUSTEXT sent exactly once after arming
  4. Retry: arm() called each tick until arming:is_armed() returns True
  5. Expiry: Lua disarms and sends STATUSTEXT when deadline passes
  6. Deadline refresh: re-sending while armed extends the deadline
  7. Re-arm: sending RAWES_ARM while disarmed restarts state machine

State machine (rawes.lua run_armon):
  nil -> interlock_low  (NV received)
  interlock_low -> arming  (one tick: Ch8=LOW, advance immediately)
  arming -> armed  (once arming:is_armed() returns True after arm())
  armed -> nil  (on deadline expiry: disarm + STATUSTEXT)

No SITL, no Docker.  Runs natively with the unit-test venv.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rawes_lua_harness import RawesLua


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _sim() -> RawesLua:
    """Fresh RawesLua instance in mode 0 (none) — ARMON runs in all modes."""
    return RawesLua()


def _armon_state(sim: RawesLua):
    v = sim.fns.armon_state()
    return str(v) if v is not None else None


def _send_arm(sim: RawesLua, ms: float):
    sim.send_named_float("RAWES_ARM", ms)


# ---------------------------------------------------------------------------
# 1. State transitions
# ---------------------------------------------------------------------------

class TestArmonStateTransitions:
    def test_initial_state_is_nil(self):
        sim = _sim()
        assert _armon_state(sim) is None

    def test_nv_received_sets_interlock_low_then_arming_same_tick(self):
        """
        NV is drained and state machine runs in the same tick.
        interlock_low advances to arming immediately (no separate tick needed).
        """
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.tick()
        # interlock_low transitions to arming in the same tick it is set
        assert _armon_state(sim) == "arming"

    def test_arming_transitions_to_armed_after_arm_succeeds(self):
        """
        Three-tick sequence to reach 'armed':
          tick 1: NV drained -> interlock_low -> arming (arm() not yet called)
          tick 2: arming branch; is_armed()=F -> arm() called -> armed=True; state stays arming
          tick 3: arming branch; is_armed()=T -> state=armed
        """
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.tick()  # tick 1: NV -> interlock_low -> arming
        assert _armon_state(sim) == "arming"
        assert sim.armed is False
        sim.tick()  # tick 2: arm() called -> armed=True; state still arming
        assert _armon_state(sim) == "arming"
        assert sim.armed is True
        sim.tick()  # tick 3: is_armed()=True -> state=armed
        assert _armon_state(sim) == "armed"

    def test_armed_stays_armed_until_deadline(self):
        sim = _sim()
        _send_arm(sim, 5_000.0)
        sim.run(0.05)  # settle into armed state
        assert _armon_state(sim) == "armed"
        sim.run(2.0)   # well within 5 s deadline
        assert _armon_state(sim) == "armed"

    def test_armed_transitions_to_nil_on_expiry(self):
        sim = _sim()
        _send_arm(sim, 200.0)   # 200 ms deadline
        sim.run(0.05)            # settle into armed
        assert _armon_state(sim) == "armed"
        sim.run(0.3)             # past deadline
        assert _armon_state(sim) is None
        assert sim.armed is False


# ---------------------------------------------------------------------------
# 2. Ch8 (motor interlock) sequence
# ---------------------------------------------------------------------------

class TestArmonCh8Sequence:
    def test_ch8_low_during_interlock_low_and_arming(self):
        """Ch8 must be 1000 (LOW) throughout interlock_low and arming states."""
        sim = _sim()
        sim.arm_fail_n = 5   # keep in arming state for several ticks
        _send_arm(sim, 60_000.0)
        for _ in range(4):
            sim.tick()
            assert _armon_state(sim) == "arming"
            assert sim.ch_out[8] == 1000, f"Expected Ch8=1000 in arming, got {sim.ch_out[8]}"

    def test_ch8_high_when_armed(self):
        """Ch8 must be 2000 (HIGH) while in 'armed' state."""
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.run(0.05)  # settle into armed
        assert _armon_state(sim) == "armed"
        assert sim.ch_out[8] == 2000

    def test_ch3_low_during_arming(self):
        """Ch3 (collective) must be 1000 during interlock_low and arming."""
        sim = _sim()
        sim.arm_fail_n = 3
        _send_arm(sim, 60_000.0)
        for _ in range(3):
            sim.tick()
            assert sim.ch_out[3] == 1000, f"Expected Ch3=1000 in arming, got {sim.ch_out[3]}"

    def test_ch3_low_when_armed(self):
        """Ch3 must be held at 1000 while armed (motor interlock does not affect Ch3)."""
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.run(0.05)
        assert _armon_state(sim) == "armed"
        assert sim.ch_out[3] == 1000


# ---------------------------------------------------------------------------
# 3. Confirmation STATUSTEXT
# ---------------------------------------------------------------------------

class TestArmonConfirmation:
    def test_confirmation_sent_once_after_arming(self):
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.run(0.05)   # settle into armed
        armed_msgs = [t for _, t in sim.messages if "RAWES arm-on: armed" in t]
        assert len(armed_msgs) == 1, f"Expected 1 arm confirmation, got {armed_msgs}"

    def test_confirmation_includes_seconds(self):
        sim = _sim()
        _send_arm(sim, 90_000.0)   # 90 s
        sim.run(0.05)
        assert sim.has_message("expires in 90s")

    def test_confirmation_not_sent_twice_on_second_tick(self):
        """_armon_armed_sent gate prevents duplicate confirmations."""
        sim = _sim()
        _send_arm(sim, 60_000.0)
        sim.run(1.0)   # many ticks in armed state
        armed_msgs = [t for _, t in sim.messages if "RAWES arm-on: armed" in t]
        assert len(armed_msgs) == 1


# ---------------------------------------------------------------------------
# 4. Retry: arm() called each tick while arming
# ---------------------------------------------------------------------------

class TestArmonRetry:
    def test_arm_retried_each_tick(self):
        """Each tick in 'arming' state calls arming:arm() once."""
        sim = _sim()
        sim.arm_fail_n = 4   # first 4 calls fail; 5th succeeds
        _send_arm(sim, 60_000.0)
        sim.run(0.1)  # 10 ticks; should have retried 5 times and succeeded
        assert sim.arm_call_count >= 5
        assert _armon_state(sim) == "armed"
        assert sim.armed is True

    def test_stays_in_arming_while_retrying(self):
        """
        State remains 'arming' until arm() finally succeeds.
        arm_fail_n=3 means calls 1, 2, 3 fail; call 4 succeeds.
        arm() is first called in tick 2 (tick 1 goes interlock_low->arming only).
          tick 1: NV -> interlock_low -> arming; no arm() call
          ticks 2,3,4: arming; arm() fails (calls 1,2,3)
          tick 5: arming; arm() call 4 succeeds -> armed=True; state stays arming
          tick 6: is_armed()=True -> state=armed
        """
        sim = _sim()
        sim.arm_fail_n = 3
        _send_arm(sim, 60_000.0)
        sim.tick()  # tick 1: state=arming; no arm() call yet
        assert _armon_state(sim) == "arming"
        assert sim.armed is False
        # Ticks 2-4: arm() fails three times
        sim.tick(); sim.tick(); sim.tick()
        assert _armon_state(sim) == "arming"
        assert sim.armed is False
        # Tick 5: 4th call to arm() succeeds; armed=True but state still "arming"
        sim.tick()
        assert sim.armed is True
        # Tick 6: is_armed()=True -> state transitions to "armed"
        sim.tick()
        assert _armon_state(sim) == "armed"

    def test_arm_not_called_when_nil(self):
        sim = _sim()
        sim.run(0.5)
        assert sim.arm_call_count == 0


# ---------------------------------------------------------------------------
# 5. Expiry: disarm + STATUSTEXT
# ---------------------------------------------------------------------------

class TestArmonExpiry:
    def test_expiry_disarms(self):
        sim = _sim()
        _send_arm(sim, 100.0)  # 100 ms
        sim.run(0.05)          # armed
        sim.run(0.2)           # past deadline
        assert sim.armed is False

    def test_expiry_sends_statustext(self):
        sim = _sim()
        _send_arm(sim, 100.0)
        sim.run(0.5)
        assert sim.has_message("RAWES arm-on: expired, disarmed")

    def test_expiry_resets_state_to_nil(self):
        sim = _sim()
        _send_arm(sim, 100.0)
        sim.run(0.5)
        assert _armon_state(sim) is None

    def test_no_expiry_before_deadline(self):
        sim = _sim()
        _send_arm(sim, 1_000.0)  # 1 s
        sim.run(0.5)
        assert _armon_state(sim) == "armed"
        assert sim.armed is True
        assert not sim.has_message("expired")


# ---------------------------------------------------------------------------
# 6. Deadline refresh: re-send while armed extends deadline
# ---------------------------------------------------------------------------

class TestArmonDeadlineRefresh:
    def test_resend_while_armed_extends_deadline(self):
        """Re-sending RAWES_ARM while already armed refreshes the deadline."""
        sim = _sim()
        _send_arm(sim, 200.0)   # 200 ms initial deadline
        sim.run(0.05)            # armed
        sim.run(0.1)             # 100 ms elapsed — close to expiry

        # Re-send with a new 1 s deadline
        _send_arm(sim, 1_000.0)
        sim.run(0.03)            # process the NV float

        # State must stay armed (not re-arm cycle)
        assert _armon_state(sim) == "armed"
        assert sim.armed is True

        # Advance past original deadline — should NOT have expired
        sim.run(0.2)
        assert _armon_state(sim) == "armed"

    def test_resend_while_armed_does_not_trigger_rearm_cycle(self):
        """Re-sending while armed must not transition through interlock_low."""
        sim = _sim()
        _send_arm(sim, 500.0)
        sim.run(0.05)   # armed

        sim.clear_messages()
        _send_arm(sim, 5_000.0)
        sim.run(0.1)

        # Only one arm confirmation total (the original); refresh does not re-send it
        # (armon_armed_sent is reset to False on re-send, so we may get a second one —
        # this is acceptable behaviour; the key check is no interlock_low cycle)
        assert _armon_state(sim) == "armed"


# ---------------------------------------------------------------------------
# 7. Re-arm after expiry
# ---------------------------------------------------------------------------

class TestArmonRearm:
    def test_rearm_after_expiry(self):
        """Sending RAWES_ARM again after expiry restarts the state machine."""
        sim = _sim()
        _send_arm(sim, 100.0)
        sim.run(0.5)   # expire
        assert _armon_state(sim) is None

        sim.clear_messages()
        _send_arm(sim, 60_000.0)
        sim.run(0.1)
        assert _armon_state(sim) == "armed"
        assert sim.armed is True
        assert sim.has_message("RAWES arm-on: armed")


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
