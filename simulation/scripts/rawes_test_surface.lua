--[[
rawes_test_surface.lua  --  Unit-test surface for rawes.lua.

This file is NOT loaded on hardware.  The Python test harness splices its
content in place of the  -- @@UNIT_TEST_HOOK  comment in rawes.lua before
executing the script.  Because the splice happens inside the anonymous
function wrapper the harness uses to load rawes.lua, every local defined
in rawes.lua is in scope here -- constants, helpers, and the private logic
extracted into testable wrappers below.

After load, the global _rawes_fns table gives tests direct access to the
internal functions without polluting the global namespace during normal flight.

Multi-value returns are wrapped in tables so lupa sees a single Lua value.
--]]

_rawes_fns = {
    -- ── Constants ────────────────────────────────────────────────────────

    ACRO_RP_RATE_DEG       = ACRO_RP_RATE_DEG,
    BASE_PERIOD_MS         = BASE_PERIOD_MS,
    FLIGHT_PERIOD_MS       = FLIGHT_PERIOD_MS,
    COL_CRUISE_FLIGHT_RAD  = COL_CRUISE_FLIGHT_RAD,
    COL_CRUISE_LAND_RAD    = COL_CRUISE_LAND_RAD,
    COL_MIN_RAD            = COL_MIN_RAD,
    COL_MAX_RAD            = COL_MAX_RAD,
    COL_SLEW_MAX           = COL_SLEW_MAX,
    MIN_TETHER_M           = MIN_TETHER_M,
    T_TRANSITION           = T_TRANSITION,
    XI_REEL_IN_DEG         = XI_REEL_IN_DEG,
    KP_VZ                  = KP_VZ,
    VZ_LAND_SP             = VZ_LAND_SP,

    -- ── Pure math / geometry ─────────────────────────────────────────────

    rodrigues              = rodrigues,
    orbit_track_azimuthal  = orbit_track_azimuthal,

    -- ── Vector3f helpers ────────────────────────────────────────────────

    v3_copy      = v3_copy,
    v3_normalize = v3_normalize,
    v3_body_z    = v3_body_z,

    -- ── Param / anchor ───────────────────────────────────────────────────

    p          = p,
    anchor_ned = anchor_ned,

    -- ── Subsystem entry points ───────────────────────────────────────────

    run_flight   = run_flight,
    run_armon    = run_armon,

    -- ── RAWES_ARM state accessors (for test_armon_lua.py) ─────────────
    armon_state       = function() return _armon_state end,
    armon_deadline_ms = function() return _armon_deadline_ms end,
    armon_armed_sent  = function() return _armon_armed_sent end,
    armon_secs        = function() return _armon_secs end,

    -- ── Cyclic / slerp helpers (defined in rawes.lua; referenced directly) ──

    slerp_step        = slerp_step,
    cyclic_error_body = cyclic_error_body,
    cyclic_rates      = cyclic_rates,
    output_rate_limit = output_rate_limit,
    rate_to_pwm       = rate_to_pwm,
}
