"""
rawes_modes.py — SCR_USER6 mode constants and NAMED_VALUE_FLOAT substate constants for rawes.lua.

SCR_USER6 encoding: plain integer, valid values 0,1,4,5 (mode only).
Substate is delivered via NAMED_VALUE_FLOAT("RAWES_SUB", N) -- never encoded in SCR_USER6.

Keep this file in sync with the constant definitions in rawes.lua.
Used by simtests, SITL stack tests, and calibrate.py.

Usage
-----
    from rawes_modes import MODE_STEADY, MODE_PUMPING, PUMP_REEL_OUT

    gcs.set_param("SCR_USER6", MODE_PUMPING)           # set mode
    gcs.send_named_float("RAWES_SUB", PUMP_REEL_OUT)   # set substate
"""

# ── Mode numbers (written directly to SCR_USER6) ──────────────────────────────

MODE_NONE     = 0   # script passive: no RC overrides; logs every 5 s + any NV message
MODE_STEADY   = 1   # bz_altitude_hold cyclic + VZ-PI collective
# modes 2,3 reserved
MODE_LANDING  = 4   # (reserved, not yet implemented)
MODE_PUMPING  = 5   # bz_altitude_hold cyclic + TensionPI collective; planner sends RAWES_SUB

# ── Named-float control values ────────────────────────────────────────────────

NV_ARMON_KEY   = "RAWES_ARM"    # named-float key: arm vehicle and start disarm countdown
                                  # value = countdown milliseconds; re-send to refresh

# ── Landing substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_LANDING) ─

LAND_DESCEND    = 0   # VZ controller active; ground planner reels in tether
LAND_FINAL_DROP = 1   # collective → 0, neutral cyclic, drop to floor

# ── Pumping substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_PUMPING) ─

PUMP_HOLD            = 0  # waiting; TensionPI setpoint = TEN_REEL_OUT (435 N)
PUMP_REEL_OUT        = 1  # paying out tether; TensionPI setpoint = TEN_REEL_OUT (435 N)
PUMP_TRANSITION      = 2  # winch reversing; TensionPI setpoint = TEN_REEL_IN (226 N)
PUMP_REEL_IN         = 3  # reeling in; TensionPI setpoint = TEN_REEL_IN (226 N)
PUMP_TRANSITION_BACK = 4  # winch accelerating; TensionPI setpoint = TEN_REEL_OUT (435 N)
