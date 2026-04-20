"""
rawes_modes.py — SCR_USER6 mode constants and NAMED_VALUE_FLOAT substate constants for rawes.lua.

SCR_USER6 encoding: plain integer, valid values 0,1,2,4,5 (mode only).
Substate is delivered via NAMED_VALUE_FLOAT("RAWES_SUB", N) -- never encoded in SCR_USER6.
Yaw trim alongside a flight mode is enabled via NAMED_VALUE_FLOAT("RAWES_YAW", 1.0).

Keep this file in sync with the constant definitions in rawes.lua.
Used by simtests, SITL stack tests, and calibrate.py.

Usage
-----
    from rawes_modes import MODE_STEADY, MODE_PUMPING, PUMP_REEL_OUT

    gcs.set_param("SCR_USER6", MODE_PUMPING)           # set mode
    gcs.send_named_float("RAWES_SUB", PUMP_REEL_OUT)   # set substate
    gcs.send_named_float("RAWES_YAW", 1.0)             # enable yaw trim alongside flight
"""

# ── Mode numbers (written directly to SCR_USER6) ──────────────────────────────

MODE_NONE     = 0   # script passive: no RC overrides; logs every 5 s + any NV message
MODE_STEADY   = 1   # cyclic orbit-tracking
MODE_YAW_LUA  = 2   # counter-torque yaw trim only (Lua PI on Ch9/SERVO9)
# mode 3 reserved
MODE_LANDING  = 4   # cyclic + VZ descent; ground planner sends RAWES_SUB substate
MODE_PUMPING  = 5   # De Schutter pumping; ground planner sends RAWES_SUB substate

# ── Named-float control values ────────────────────────────────────────────────

NV_YAW_ENABLE  = 1.0   # send as RAWES_YAW to enable yaw trim alongside any flight mode
NV_YAW_DISABLE = 0.0   # send as RAWES_YAW to disable yaw trim (default after mode change)

NV_ARMON_KEY   = "RAWES_ARM"    # named-float key: arm vehicle and start disarm countdown
                                  # value = countdown milliseconds; re-send to refresh

# ── Landing substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_LANDING) ─

LAND_DESCEND    = 0   # VZ controller active; ground planner reels in tether
LAND_FINAL_DROP = 1   # collective → 0, neutral cyclic, drop to floor

# ── Pumping substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_PUMPING) ─

PUMP_HOLD            = 0  # waiting; reel-out collective, orbit tracking
PUMP_REEL_OUT        = 1  # paying out tether; reel-out collective, orbit tracking
PUMP_TRANSITION      = 2  # winch reversing; collective ramps up, body_z slews to xi=80
PUMP_REEL_IN         = 3  # reeling in; reel-in collective, body_z at xi=80 deg
PUMP_TRANSITION_BACK = 4  # winch accelerating; collective ramps down, body_z slews back to orbit
