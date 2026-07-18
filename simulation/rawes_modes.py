"""
rawes_modes.py — RAWES_MODE constants and NAMED_VALUE_FLOAT constants for rawes.lua.

RAWES_MODE is a script-generated parameter (registered by rawes.lua via param:add_table).
Valid values: 0=none, 1=steady, 3=passive, 4=landing.
Every other dynamic input (substate, tuning) is delivered via NAMED_VALUE_FLOAT --
never encoded in a parameter. Substate is delivered via NAMED_VALUE_FLOAT("RAWES_SUB", N).
The anchor location (RAWES_LAT/LON/AAL) is delivered via NAMED_VALUE_INT as an
absolute lat/lon/alt; rawes.lua resolves it on board to an EKF-local NED offset
via Location:get_vector_from_origin_NEU_m().

Keep this file in sync with the constant definitions in rawes.lua.
Used by simtests, SITL stack tests, and calibrate.py.

Usage
-----
    from simulation.rawes_modes import MODE_STEADY, PUMP_REEL_OUT, send_anchor_ned

    gcs.set_param("RAWES_MODE", MODE_STEADY)            # set mode (pumping runs in steady)
    gcs.send_named_float("RAWES_SUB", PUMP_REEL_OUT)    # set substate
    send_anchor_ned(gcs, 0.0, 0.0, 0.0)                 # anchor at the mock/SITL origin
"""

import math

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

# ── Named-float tuning key ────────────────────────────────────────────────────

NV_SLEW_KEY     = "RAWES_SLW"    # body_z / elevation slew rate limit [rad/s] — also a RAWES_* param (override via NVF for runtime changes)

# ── Named-int anchor keys ─────────────────────────────────────────────────────
# rawes.lua gates altitude-hold capture on all three anchor ints arriving AND
# the onboard lat/lon/alt -> NED conversion succeeding (see _try_resolve_anchor()
# in rawes.lua).  Sent as NAMED_VALUE_INT (not FLOAT) to preserve ArduPilot's
# own Location int32 precision (~1 cm) end-to-end.

NV_ANCHOR_LAT_KEY = "RAWES_LAT"    # anchor latitude  [deg * 1e7]
NV_ANCHOR_LON_KEY = "RAWES_LON"    # anchor longitude [deg * 1e7]
NV_ANCHOR_ALT_KEY = "RAWES_AAL"    # anchor altitude  [cm, AMSL]

# Fixed EKF/GPS origin used by the mock Lua harness (mock_ardupilot.lua) and
# the SITL stack tests (stack_utils.py re-exports these as HOME_LAT_DEG/
# HOME_LON_DEG/HOME_ALT_M -- this module is the single source of truth).
MOCK_ORIGIN_LAT_DEG = 51.5074
MOCK_ORIGIN_LON_DEG = -0.1278
MOCK_ORIGIN_ALT_M   = 50.0

# ArduPilot's flat-earth degrees<->metres scaling (AP_Math/definitions.h LATLON_TO_M).
_LATLON_TO_M = 0.011131884502145034


def anchor_ned_to_gps(dn_m: float, de_m: float, dd_m: float) -> tuple[int, int, int]:
    """Convert a desired anchor NED offset (from MOCK_ORIGIN_*) to (lat_e7, lon_e7, alt_cm).

    Inverse of ArduPilot's Location:get_vector_from_origin_NEU_m() flat-earth
    math (AP_Common/Location.cpp), so a value produced here and sent via
    RAWES_LAT/LON/AAL round-trips (up to int32 quantization, ~1 cm) back to
    the SAME (dn_m, de_m, dd_m) once rawes.lua resolves it on board -- as long
    as the receiving side's EKF/SITL origin is MOCK_ORIGIN_LAT_DEG/LON_DEG/ALT_M.
    """
    origin_lat_e7 = round(MOCK_ORIGIN_LAT_DEG * 1e7)
    origin_lon_e7 = round(MOCK_ORIGIN_LON_DEG * 1e7)
    origin_alt_cm = round(MOCK_ORIGIN_ALT_M * 100)

    lat_e7 = origin_lat_e7 + round(dn_m / _LATLON_TO_M)
    lon_scale = max(math.cos(math.radians(origin_lat_e7 * 1e-7)), 0.01)
    lon_e7 = origin_lon_e7 + round(de_m / (_LATLON_TO_M * lon_scale))
    alt_cm = origin_alt_cm - round(dd_m * 100)  # NED down -> AMSL alt (up)
    return lat_e7, lon_e7, alt_cm


def send_anchor_ned(sim, dn_m: float, de_m: float, dd_m: float) -> None:
    """Send the anchor at NED offset (dn_m, de_m, dd_m) from MOCK_ORIGIN_* via NAMED_VALUE_INT.

    `sim` must expose `send_named_int(name, value)` (RawesLua harness / gcs.py).
    """
    lat_e7, lon_e7, alt_cm = anchor_ned_to_gps(dn_m, de_m, dd_m)
    sim.send_named_int(NV_ANCHOR_LAT_KEY, lat_e7)
    sim.send_named_int(NV_ANCHOR_LON_KEY, lon_e7)
    sim.send_named_int(NV_ANCHOR_ALT_KEY, alt_cm)



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
