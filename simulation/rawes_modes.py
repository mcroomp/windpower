"""
rawes_modes.py — RAWES_MODE constants and NAMED_VALUE_FLOAT constants for rawes.lua.

RAWES_MODE is a script-generated parameter (registered by rawes.lua via param:add_table).
Valid values: 0=none, 1=steady, 3=passive, 4=landing.
Every other input (substate, tuning, anchor) is delivered via NAMED_VALUE_FLOAT --
never encoded in a parameter. Substate is delivered via NAMED_VALUE_FLOAT("RAWES_SUB", N);
the slew rate and anchor position use RAWES_SLW (param) / RAWES_ANN / RAWES_ANE / RAWES_AND.

Keep this file in sync with the constant definitions in rawes.lua.
Used by simtests, SITL stack tests, and calibrate.py.

Usage
-----
    from rawes_modes import MODE_STEADY, PUMP_REEL_OUT, NV_ANCHOR_N_KEY

    gcs.set_param("RAWES_MODE", MODE_STEADY)            # set mode (pumping runs in steady)
    gcs.send_named_float("RAWES_SUB", PUMP_REEL_OUT)    # set substate
    gcs.send_named_float(NV_ANCHOR_N_KEY, -pos0[0])     # anchor North (EKF frame)
"""

# ── Mode numbers (RAWES_MODE script-generated param; 0=none 1=steady 3=passive 4=landing) ──

MODE_NONE     = 0   # script passive: no control-channel overrides (CH8 interlock hold still applies while armed)
MODE_STEADY   = 1   # bz_altitude_hold cyclic (commanded tension) + altitude-PID collective
# mode 2 reserved (unused)
MODE_PASSIVE  = 3   # kinematic capture helper: hold IC attitude during release
MODE_LANDING  = 4   # (reserved, not yet implemented)
# mode 5 removed: pumping now runs in MODE_STEADY (same control law); the ground
# schedule varies the commanded tension (RAWES_TEN) and RAWES_SUB phase over time.

# ── Named-float control values ────────────────────────────────────────────────

NV_ARMON_KEY   = "RAWES_ARM"    # named-float key: arm vehicle and start disarm countdown
                                  # value = countdown milliseconds; re-send to refresh

# ── Named-float tuning + anchor keys ─────────────────────────────────────────────────────
# rawes.lua gates altitude-hold capture on all three anchor floats arriving.

NV_SLEW_KEY     = "RAWES_SLW"    # body_z / elevation slew rate limit [rad/s] — also a RAWES_* param (override via NVF for runtime changes)
NV_ANCHOR_N_KEY = "RAWES_ANN"    # anchor North from EKF origin [m]
NV_ANCHOR_E_KEY = "RAWES_ANE"    # anchor East  from EKF origin [m]
NV_ANCHOR_D_KEY = "RAWES_AND"    # anchor Down  from EKF origin [m]

# ── Landing substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" when mode=MODE_LANDING) ─

LAND_DESCEND    = 0   # VZ controller active; ground planner reels in tether
LAND_FINAL_DROP = 1   # collective → 0, neutral cyclic, drop to floor

# ── Pumping substates (sent as NAMED_VALUE_FLOAT "RAWES_SUB" during the ground
#    pumping schedule; the vehicle stays in MODE_STEADY) ──────────────────────

PUMP_HOLD            = 0  # waiting; commanded tension RAWES_TEN = TEN_REEL_OUT (435 N)
PUMP_REEL_OUT        = 1  # paying out tether; commanded tension = TEN_REEL_OUT (435 N)
PUMP_TRANSITION      = 2  # winch reversing; commanded tension = TEN_REEL_IN (226 N)
PUMP_REEL_IN         = 3  # reeling in; commanded tension = TEN_REEL_IN (226 N)
PUMP_TRANSITION_BACK = 4  # winch accelerating; commanded tension = TEN_REEL_OUT (435 N)
