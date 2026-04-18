"""
rawes_modes.py — SCR_USER6 mode constants and NAMED_VALUE_FLOAT substate constants for rawes.lua.

SCR_USER6 encoding: plain integer 0..8 (mode only).
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

MODE_NONE       = 0   # scripting loaded but both subsystems idle
MODE_STEADY     = 1   # cyclic orbit-tracking only (no yaw)
MODE_YAW        = 2   # counter-torque yaw trim only
MODE_STEADY_YAW = 3   # cyclic + yaw trim simultaneously
MODE_LANDING    = 4   # cyclic + VZ descent; ground planner sends RAWES_SUB substate
MODE_PUMPING    = 5   # De Schutter pumping; ground planner sends RAWES_SUB substate
MODE_ARM_HOLD   = 6   # hold Ch3=1000 Ch8=2000 keepalive only
MODE_YAW_TEST   = 7   # motor at 25% for 20 s (bench verification)
MODE_YAW_LTD    = 8   # yaw PI with 30 s motor hard-stop

# ── Landing substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_LANDING) ─

LAND_DESCEND    = 0   # VZ controller active; ground planner reels in tether
LAND_FINAL_DROP = 1   # collective → 0, neutral cyclic, drop to floor

# ── Pumping substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_PUMPING) ─

PUMP_HOLD            = 0  # waiting; reel-out collective, orbit tracking
PUMP_REEL_OUT        = 1  # paying out tether; reel-out collective, orbit tracking
PUMP_TRANSITION      = 2  # winch reversing; collective ramps up, body_z slews to xi=80
PUMP_REEL_IN         = 3  # reeling in; reel-in collective, body_z at xi=80 deg
PUMP_TRANSITION_BACK = 4  # winch accelerating; collective ramps down, body_z slews back to orbit
