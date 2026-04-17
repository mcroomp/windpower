"""
rawes_modes.py — SCR_USER6 mode/substate constants for rawes.lua.

SCR_USER6 encoding:  mode_num * 1000 + substate

Keep this file in sync with the constant definitions in rawes.lua.
Used by simtests, SITL stack tests, and calibrate.py.

Usage
-----
    from rawes_modes import MODE_STEADY, MODE_PUMPING, PUMP_REEL_OUT

    sim.set_param("mode", MODE_PUMPING + PUMP_REEL_OUT)   # SCR_USER6 = 5001
    gcs.set_param("SCR_USER6", MODE_LANDING + LAND_FINAL_DROP)
"""

# ── Mode numbers (multiply by 1000 to get SCR_USER6 base value) ──────────────

MODE_NONE       = 0     * 1000   # scripting loaded but both subsystems idle
MODE_STEADY     = 1     * 1000   # cyclic orbit-tracking only (no yaw)
MODE_YAW        = 2     * 1000   # counter-torque yaw trim only
MODE_STEADY_YAW = 3     * 1000   # cyclic + yaw trim simultaneously
MODE_LANDING    = 4     * 1000   # cyclic + VZ descent; ground planner sets substate
MODE_PUMPING    = 5     * 1000   # De Schutter pumping; ground planner sets substate
MODE_ARM_HOLD   = 6     * 1000   # hold Ch3=1000 Ch8=2000 keepalive only
MODE_YAW_TEST   = 7     * 1000   # motor at 25% for 20 s (bench verification)
MODE_YAW_LTD    = 8     * 1000   # yaw PI with 30 s motor hard-stop

# ── Landing substates (mode=MODE_LANDING, substate = SCR_USER6 % 1000) ───────

LAND_DESCEND    = 0     # VZ controller active; ground planner reels in tether
LAND_FINAL_DROP = 1     # collective → 0, neutral cyclic, drop to floor

# ── Pumping substates (mode=MODE_PUMPING, substate = SCR_USER6 % 1000) ───────

PUMP_HOLD            = 0  # waiting; reel-out collective, orbit tracking
PUMP_REEL_OUT        = 1  # paying out tether; reel-out collective, orbit tracking
PUMP_TRANSITION      = 2  # winch reversing; collective ramps up, body_z slews to xi=80
PUMP_REEL_IN         = 3  # reeling in; reel-in collective, body_z at xi=80 deg
PUMP_TRANSITION_BACK = 4  # winch accelerating; collective ramps down, body_z slews back to orbit
