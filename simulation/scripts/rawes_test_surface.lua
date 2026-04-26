--[[
rawes_test_surface.lua  --  Unit-test surface for rawes.lua.

Spliced in place of  -- @@UNIT_TEST_HOOK  before the script executes.
All rawes.lua module-level locals are in scope here.
--]]

_rawes_fns = {
    -- ── Constants ────────────────────────────────────────────────────────────

    ACRO_RP_RATE_DEG        = ACRO_RP_RATE_DEG,
    BASE_PERIOD_MS          = BASE_PERIOD_MS,
    FLIGHT_PERIOD_MS        = FLIGHT_PERIOD_MS,
    COL_CRUISE_FLIGHT_RAD   = COL_CRUISE_FLIGHT_RAD,
    COL_MIN_RAD             = COL_MIN_RAD,
    COL_MAX_RAD             = COL_MAX_RAD,
    COL_SLEW_MAX            = COL_SLEW_MAX,
    COL_REEL_OUT            = COL_REEL_OUT,
    T_PUMP_TRANSITION       = T_PUMP_TRANSITION,
    MIN_TETHER_M            = MIN_TETHER_M,
    MASS_KG                 = MASS_KG,
    G_ACCEL                 = G_ACCEL,
    KP_VZ                   = KP_VZ,
    KI_VZ                   = KI_VZ,
    KP_TEN                  = KP_TEN,
    KI_TEN                  = KI_TEN,
    COL_MAX_TEN             = COL_MAX_TEN,

    -- ── Pure geometry ────────────────────────────────────────────────────────

    bz_altitude_hold  = bz_altitude_hold,

    -- ── Vector3f helpers ─────────────────────────────────────────────────────

    v3_copy    = v3_copy,
    v3_body_z  = v3_body_z,

    -- ── Param / anchor ───────────────────────────────────────────────────────

    p          = p,
    anchor_ned = anchor_ned,

    -- ── Cyclic helpers ───────────────────────────────────────────────────────

    cyclic_error_body = cyclic_error_body,
    output_rate_limit = output_rate_limit,
    rate_to_pwm       = rate_to_pwm,

    -- ── Subsystem entry points ───────────────────────────────────────────────

    run_flight = run_flight,
    run_armon  = run_armon,

    -- ── RAWES_ARM state accessors ─────────────────────────────────────────────

    armon_state       = function() return _armon_state end,
    armon_deadline_ms = function() return _armon_deadline_ms end,
    armon_armed_sent  = function() return _armon_armed_sent end,
    armon_secs        = function() return _armon_secs end,

    -- ── Altitude hold state accessors ─────────────────────────────────────────

    el_initialized = function() return _el_initialized end,
    el_rad         = function() return _el_rad end,
    target_alt     = function() return _target_alt end,
    tension_n      = function() return _tension_n end,

    -- ── TensionPI state accessors ─────────────────────────────────────────────

    col_i_ten      = function() return _col_i_ten end,
    col_held       = function() return _col_held end,
    ten_setpoint   = function() return _ten_setpoint end,
    ten_sp_fresh   = function() return _ten_sp_fresh end,
}
