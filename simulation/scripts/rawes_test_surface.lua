--[[
rawes_test_surface.lua  --  Unit-test surface for rawes.lua.

Spliced in place of  -- @@UNIT_TEST_HOOK  before the script executes.
All rawes.lua module-level locals are in scope here.
--]]

_rawes_fns = {
    -- ── Constants ────────────────────────────────────────────────────────────

    BASE_PERIOD_MS          = BASE_PERIOD_MS,
    FLIGHT_PERIOD_MS        = FLIGHT_PERIOD_MS,
    COL_CRUISE_FLIGHT_RAD   = COL_CRUISE_FLIGHT_RAD,
    COL_MIN_RAD             = COL_MIN_RAD,
    COL_MAX_RAD             = COL_MAX_RAD,
    COL_SLEW_MAX            = COL_SLEW_MAX,
    MIN_TETHER_M            = MIN_TETHER_M,
    MASS_KG                 = MASS_KG,
    G_ACCEL                 = G_ACCEL,
    KP_ALT                  = KP_ALT,
    KI_ALT                  = KI_ALT,
    KD_VZ                   = KD_VZ,
    RATE_KP_OUTER           = RATE_KP_OUTER,
    RATE_ACCEL_MAX_RADSS    = RATE_ACCEL_MAX_RADSS,
    AZ_REF_TAU_S            = AZ_REF_TAU_S,

    -- ── Pure geometry ───────────────────────────────────────────────────────

    bz_altitude_hold    = bz_altitude_hold,
    sqrt_rate_from_error = sqrt_rate_from_error,
    compute_rate_cmd_sqrt = compute_rate_cmd_sqrt,
    wrap_pi              = wrap_pi,
    update_plane_azimuth = update_plane_azimuth,

    -- ── Vector3f helpers ─────────────────────────────────────────────────────

    v3_copy    = v3_copy,

    -- ── Param / anchor ───────────────────────────────────────────────────────

    p          = p,
    anchor_ned = anchor_ned,

    -- ── GUIDED angle conversion ──────────────────────────────────────────────

    bz_ned_to_roll_pitch = bz_ned_to_roll_pitch,

    -- ── Subsystem entry points ───────────────────────────────────────────────

    run_flight = run_flight,
    run_armon  = run_armon,
    run_manual = run_manual,

    -- ── Manual mode state accessors ──────────────────────────────────────────

    MODE_MANUAL  = MODE_MANUAL,
    man_tlon_rad = function() return _man_tlon_rad end,
    man_tlat_rad = function() return _man_tlat_rad end,

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
    ic_col           = function() return _ic_col end,

    -- ── Altitude PID state accessors ─────────────────────────────────────────

    col_trim         = function() return _col_trim end,
    alt_i            = function() return _alt_i end,

    -- ── Vibration damper constants + function ─────────────────────────────────

    K_VIB           = K_VIB,
    VIB_HP_TAU      = VIB_HP_TAU,
    VIB_VEL_TAU     = VIB_VEL_TAU,
    VIB_COL_MAX     = VIB_COL_MAX,
    vib_damper_step = vib_damper_step,
    vib_vel_est     = function() return _vib_vel_est end,
    vib_corr_last   = function() return _vib_corr_last end,
}
